module Holo #(CLK_FREQ, OUT_FREQ, NUM_CHANNELS) (
	input clk,
	input nReset,
	input mathClk,
	
	output [49:0] t,
	output [2:0] LEDpwm,
	
	input mosi,
	input sck,
	input ncs,
	
	input syncin,
	input top,
	input left,
	
	output f_sck,
	inout [3:0] f_io,
	output f_ncs
 );

	/*
	Transformation matrix T=
	[	A	B	C	0]
	[	D	E	F	0]
	[	G	H	I	0]
	[	X	Y	Z	1]
	*/

	localparam FIFO_BIT_SIZE = 13;
	localparam SPI_BUF_SIZE = 72;
	localparam POS_BIT_SIZE = 13;
	localparam PHASE_BIT_SIZE = $clog2(CLK_FREQ/OUT_FREQ);
	localparam BLINK_ON_TIME = 100;
	localparam BLINK_OFF_TIME = 1900;
	localparam ROT_BIT_SIZE = 8;

	// ============================================================
	// 🔥 NEW: MULTI-PARTICLE SUPPORT
	// ============================================================
	localparam NUM_PARTICLES = 4;

	logic [POS_BIT_SIZE-1:0] particle_x [NUM_PARTICLES];
	logic [POS_BIT_SIZE-1:0] particle_y [NUM_PARTICLES];
	logic [POS_BIT_SIZE-1:0] particle_z [NUM_PARTICLES];

	logic [$clog2(NUM_PARTICLES)-1:0] writeParticleIdx;
	logic [$clog2(NUM_PARTICLES)-1:0] particleIdx;
	// ============================================================

	localparam CMD_NOOP					= 4'd0;
	localparam CMD_SET_COLORS			= 4'd1;
	localparam CMD_SET_SPEED			= 4'd2;
	localparam CMD_POS_COL_RAM			= 4'd3;
	localparam CMD_SET_HALF_HEIGHT	= 4'd4;
	localparam CMD_SET_RADIUS_SQR		= 4'd6;
	localparam CMD_ENABLE_FIFO_PROC	= 4'd7;
	localparam CMD_FLASH_WRITE 		= 4'd8;
	localparam CMD_FLASH_BLOCK_ERASE = 4'd9;
	localparam CMD_SET_MATRIX			= 4'd10;

	localparam FLASH_NOOP 				= 3'd0;
	localparam FLASH_WRITE 				= 3'd2;
	localparam FLASH_READ				= 3'd3;
	localparam FLASH_BLOCK_ERASE 		= 3'd4;

	logic [PHASE_BIT_SIZE-1:0] phase [NUM_CHANNELS];
	logic cycleStart;

	logic [7:0] colors [3];

	// 🔥 MODIFIED: now driven from particle buffer
	logic [POS_BIT_SIZE-1:0] x;
	logic [POS_BIT_SIZE-1:0] y;
	logic [POS_BIT_SIZE-1:0] z;

	logic [PHASE_BIT_SIZE-1:0] calculatedPhase [NUM_CHANNELS];
	logic calcDone;

	logic [POS_BIT_SIZE*3-1 + 7:0] ramWriteData;
	logic [POS_BIT_SIZE*3-1 + 7:0] ramReadData;
	logic wren;

	logic fifoRdAck;
	logic [6:0] fifoColors;

	logic [3:0] cmdReceived;
	logic loadCalcPhases;
	logic calcStart;
	logic processFifo;

	logic [FIFO_BIT_SIZE-1:0] rdaddress;
	logic [FIFO_BIT_SIZE-1:0] wraddress;

	logic blinkEnabled;
	logic LEDen;
	logic LEDoverride;

	logic phaseEnabled [NUM_CHANNELS];

	logic [POS_BIT_SIZE-1:0] halfHeight;
	logic [5:0] speed;
	logic [5:0] speedCnt;

	logic [22:0] XZradiusSquared;

	logic [7:0] flashWriteData;
	logic [7:0] flashReadData;
	logic flashReadRdy;
	logic flashFifoWrReq;
	logic flashFifoClr;

	logic flashBusy;
	logic [2:0] flashCmd;
	logic [15:0] flashPage;

	logic [POS_BIT_SIZE-1:0] Pos[3];
	logic [ROT_BIT_SIZE-1:0] Rot[9];
	logic [POS_BIT_SIZE-1:0] PosBuf[3];
	logic [ROT_BIT_SIZE-1:0] RotBuf[9];
	logic [POS_BIT_SIZE-1:0] CalcPosBuf[3];

	// ============================================================
	// 🔥 NEW: store particles from FIFO
	// ============================================================
	always @(posedge clk) begin
		if (!nReset)
			writeParticleIdx <= 0;
		else if (fifoRdAck) begin
			particle_x[writeParticleIdx] <= CalcPosBuf[0];
			particle_y[writeParticleIdx] <= CalcPosBuf[1];
			particle_z[writeParticleIdx] <= CalcPosBuf[2];

			writeParticleIdx <= writeParticleIdx + 1;
		end
	end

	// 🔥 NEW: particle scheduler
	always @(posedge clk) begin
		if (!nReset)
			particleIdx <= 0;
		else if (cycleStart)
			particleIdx <= particleIdx + 1;
	end

	assign x = particle_x[particleIdx];
	assign y = particle_y[particleIdx];
	assign z = particle_z[particleIdx];
	// ============================================================

	always@(posedge clk)
		if(loadCalcPhases)
			phase <= calculatedPhase;

	// ---------------- FIFO read + calc control ------------------

	logic [3:0] readState;
	always@(posedge clk) begin
		loadCalcPhases <= (cycleStart && calcDone);

		if(!processFifo)
			speedCnt <= '0;

		case(readState)
			0:
				begin
					fifoRdAck <= 0;
					calcStart <= 0;

					if(wraddress>0 && cycleStart && processFifo) begin
						speedCnt <= speedCnt==speed ? '0 : speedCnt + 1;
						if(speedCnt == 0)
							readState<= 1;
					end 
				end
			1: readState <=2;
			2: readState <=3;
			3: readState <=4;

			4:
				begin
					// 🔥 MODIFIED: removed direct x/y/z write
					fifoColors <= ramReadData[45:39];
					calcStart <= 1;
					fifoRdAck <= 1;
					readState <= 0;
				end
		endcase
	end
		
	
	
	//blink LEDs
	logic [$clog2(CLK_FREQ*(BLINK_ON_TIME+BLINK_OFF_TIME)/1000000)-1:0] blinkCnt;
	always@(posedge clk) begin
		if(!nReset) begin
			blinkCnt <= '0;
			LEDen <= '1;
		end else begin
			if(blinkEnabled) begin
				blinkCnt <= blinkCnt + 1;
				if(blinkCnt == (LEDen ? CLK_FREQ*BLINK_ON_TIME/1000000 : CLK_FREQ*BLINK_OFF_TIME/1000000) -1) begin
					blinkCnt <= '0;
					LEDen <= ~LEDen;
				end
			end else begin
				LEDen <= '1;
				blinkCnt <= '0;
			end
		end
	end
	
	
	logic ncsSync;
	logic ncsSync_pipe;
	always @(posedge clk)
		{ ncsSync, ncsSync_pipe } <= { ncsSync_pipe, ncs };
	
	logic spiByteAvailableSync;
	logic spiByteAvailableSync_pipe;
	always @(posedge clk)
		{ spiByteAvailableSync, spiByteAvailableSync_pipe } <= { spiByteAvailableSync_pipe, spiByteAvailable };
		
	logic spiByteAvailableSyncEdge;
	logic spiByteAvailableSync_dly;
	always @(posedge clk)
		spiByteAvailableSync_dly <= spiByteAvailableSync;
		
	assign spiByteAvailableSyncEdge = spiByteAvailableSync & ~spiByteAvailableSync_dly;
	
	logic [3:0] spiState;
	logic [3:0] cmdReceived_buf;
	logic [3:0] spiByteCnt;
	logic [15:0] spiParameter[8];
	//SPI processing
	always@(posedge clk) begin
		if(!nReset) begin
				spiState <= '0;
				cmdReceived <= CMD_NOOP;
		end else begin
			case(spiState)
				0:
					begin
						cmdReceived <= CMD_NOOP;
						if(ncsSync == 0) begin
							spiState <= 1;
							flashFifoClr <= '1;
						end
					end
				1:
					begin
						flashFifoClr <= '0;
						
						if(ncsSync == 1)
							spiState <= 0;
						
						if(spiByteAvailableSyncEdge == 1) begin
							cmdReceived_buf <= spiByteBuffer[4:1];
							spiState <= 2;
							spiByteCnt <= 0;
						end
					end
				2:
					begin
						if(ncsSync == 1)
							spiState <= 0;
						
						if(spiByteAvailableSyncEdge == 1) begin
							if(!spiByteCnt[0])
								spiParameter[spiByteCnt[3:1]][7:0] <= spiByteBuffer;
							else
								spiParameter[spiByteCnt[3:1]][15:8] <= spiByteBuffer;
							
							spiByteCnt <= spiByteCnt + 1;

							if(cmdReceived_buf != CMD_SET_MATRIX && spiByteCnt == 7)
								spiState <= 3;
							if(cmdReceived_buf == CMD_SET_MATRIX && spiByteCnt == 15)
								spiState <= 3;

						end
					end
				3:
					begin
						flashFifoWrReq <= '0;
						if(ncsSync == 1) begin
							spiState <= 4;
						end else if(spiByteAvailableSyncEdge == 1) begin
							if(cmdReceived_buf == CMD_FLASH_WRITE) begin
								flashWriteData <= spiByteBuffer;
								flashFifoWrReq <= '1;
							end else begin
								spiState <= 0;
							end
						end
					end
				4:
					begin
						cmdReceived <= cmdReceived_buf;
						
						spiState <= 0;
					end
			endcase
		
		end
	end
		
	logic [3:0] spiBitCnt;
	logic [7:0] spiByteBuffer;
	logic [7:0] spiByte;
	logic spiByteAvailable;
	
	always @(posedge sck or posedge ncs) begin
		if(ncs) begin
			spiByteAvailable <= 0;
			spiBitCnt <= 0;
		end else if(!ncs  && sck) begin
			spiByte <= {spiByte[6:0], mosi};
			spiBitCnt <= spiBitCnt + 1;
			spiByteAvailable <= 0;
			if(spiBitCnt==7) begin
				spiByteBuffer <= {spiByte[6:0], mosi};
				spiByteAvailable <= 1;
				spiBitCnt <= 0;
			end
		end
	end
	
	

	// ---------------- Calculation ------------------

	CalcPhase #(
		.NUM_CHANNELS(NUM_CHANNELS),
		.MAX_PHASE_CNT(CLK_FREQ/OUT_FREQ),
		.POS_BIT_SIZE(POS_BIT_SIZE),
		.PHASE_BIT_SIZE(PHASE_BIT_SIZE)
	) calcPhase(
		.clk(mathClk),
		.nReset,
		.x,
		.y,
		.z,
		.halfHeight,
		.XZradiusSquared,
		.start(calcStart),
		.phase(calculatedPhase),
		.phaseEnabled,
		.done(calcDone),
		.top,
		.left,
		.cycleStart
	);

	// ---------------- PWM ------------------

	PwmCtrl #(
		.NUM_CHANNELS(NUM_CHANNELS),
		.CLK_FREQ(CLK_FREQ),
		.OUT_FREQ(OUT_FREQ)
	) pwmCtrl(
		.clk,
		.nReset,
		.phase,
		.en(phaseEnabled),
		.out(t),
		.cycleStart,
		.syncin
	);

	// ---------------- LED ------------------

	LED led[2:0] (
		.clk,
		.nReset,
		.duty(colors),
		.LEDen,
		.out(LEDpwm)
	);

	// ---------------- RAM ------------------

	PosColRam posColRam(
		.clock(clk),
		.data(ramWriteData),
		.rdaddress,
		.wraddress,
		.wren,
		.q(ramReadData)
	);

	// ---------------- FLASH ------------------

	QPIFlash qpiFlash(
		.clk,
		.nReset,
		.f_sck,
		.f_io,
		.f_ncs,
		.writeData(flashWriteData),
		.readData(flashReadData),
		.rdRdy(flashReadRdy),
		.fifoWrReq(flashFifoWrReq),
		.fifoClr(flashFifoClr),
		.busy(flashBusy),
		.cmd(flashCmd),
		.page(flashPage)
	);

	// ---------------- TRANSFORMS ------------------

	Rotate1D #(.POS_BIT_SIZE(POS_BIT_SIZE)) transformX (
		.clk,
		.x(ramReadData[POS_BIT_SIZE*3-1:POS_BIT_SIZE*2]),
		.y(ramReadData[POS_BIT_SIZE*2-1:POS_BIT_SIZE]),
		.z(ramReadData[POS_BIT_SIZE-1:0]),
		.coeff1(Rot[0]),
		.coeff2(Rot[3]),
		.coeff3(Rot[6]),
		.translate(Pos[0]),
		.result(CalcPosBuf[0])
	);

	Rotate1D #(.POS_BIT_SIZE(POS_BIT_SIZE)) transformY (
		.clk,
		.x(ramReadData[POS_BIT_SIZE*3-1:POS_BIT_SIZE*2]),
		.y(ramReadData[POS_BIT_SIZE*2-1:POS_BIT_SIZE]),
		.z(ramReadData[POS_BIT_SIZE-1:0]),
		.coeff1(Rot[1]),
		.coeff2(Rot[4]),
		.coeff3(Rot[7]),
		.translate(Pos[1]),
		.result(CalcPosBuf[1])
	);

	Rotate1D #(.POS_BIT_SIZE(POS_BIT_SIZE)) transformZ (
		.clk,
		.x(ramReadData[POS_BIT_SIZE*3-1:POS_BIT_SIZE*2]),
		.y(ramReadData[POS_BIT_SIZE*2-1:POS_BIT_SIZE]),
		.z(ramReadData[POS_BIT_SIZE-1:0]),
		.coeff1(Rot[2]),
		.coeff2(Rot[5]),
		.coeff3(Rot[8]),
		.translate(Pos[2]),
		.result(CalcPosBuf[2])
	);

endmodule