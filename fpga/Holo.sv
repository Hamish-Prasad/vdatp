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

	localparam FIFO_BIT_SIZE = 13;
	localparam POS_BIT_SIZE = 13;
	localparam PHASE_BIT_SIZE = $clog2(CLK_FREQ/OUT_FREQ);

	localparam CMD_NOOP              = 4'd0;
	localparam CMD_SET_COLORS        = 4'd1;
	localparam CMD_SET_SPEED         = 4'd2;
	localparam CMD_POS_COL_RAM       = 4'd3;
	localparam CMD_SET_HALF_HEIGHT   = 4'd4;
	localparam CMD_SET_RADIUS_SQR    = 4'd6;
	localparam CMD_ENABLE_FIFO_PROC  = 4'd7;
	localparam CMD_SET_MATRIX        = 4'd10;

	// ✅ NEW
	localparam CMD_SET_PHASES        = 4'd11;

	logic [PHASE_BIT_SIZE-1:0] phase [NUM_CHANNELS];
	logic [PHASE_BIT_SIZE-1:0] calculatedPhase [NUM_CHANNELS];
	logic loadCalcPhases;

	// ==============================
	// SPI BUFFER (EXPANDED)
	// ==============================

	logic [15:0] spiParameter[64];

	logic [3:0] cmdReceived;

	// ==============================
	// DIRECT PHASE LOAD
	// ==============================

	integer i;

	always @(posedge clk) begin
		if(!nReset) begin
			for(i=0;i<NUM_CHANNELS;i=i+1)
				phase[i] <= 0;
		end
		else if(cmdReceived == CMD_SET_PHASES) begin
			for(i = 0; i < NUM_CHANNELS; i = i + 1) begin
				phase[i] <= spiParameter[i];
			end
		end
		else if(loadCalcPhases) begin
			phase <= calculatedPhase;
		end
	end

	// ==============================
	// SPI RECEIVER (MODIFIED)
	// ==============================

	logic [3:0] spiState;
	logic [3:0] cmdReceived_buf;
	logic [7:0] spiByteBuffer;
	logic [7:0] spiByte;
	logic spiByteAvailable;
	logic [7:0] spiByteCnt;

	always @(posedge clk) begin
		case(spiState)
			0: begin
				cmdReceived <= CMD_NOOP;
				if(ncs == 0)
					spiState <= 1;
			end

			1: begin
				if(ncs == 1)
					spiState <= 0;

				if(spiByteAvailable) begin
					cmdReceived_buf <= spiByteBuffer[4:1];
					spiState <= 2;
					spiByteCnt <= 0;
				end
			end

			2: begin
				if(ncs == 1)
					spiState <= 0;

				if(spiByteAvailable) begin
					if(!spiByteCnt[0])
						spiParameter[spiByteCnt[7:1]][7:0] <= spiByteBuffer;
					else
						spiParameter[spiByteCnt[7:1]][15:8] <= spiByteBuffer;

					spiByteCnt <= spiByteCnt + 1;

					// ✅ NEW CONDITION FOR PHASE STREAM
					if(cmdReceived_buf == CMD_SET_PHASES &&
					   spiByteCnt == (NUM_CHANNELS*2 -1))
						spiState <= 3;
					else if(cmdReceived_buf != CMD_SET_MATRIX &&
					        spiByteCnt == 7)
						spiState <= 3;
				end
			end

			3: begin
				if(ncs == 1)
					spiState <= 4;
			end

			4: begin
				cmdReceived <= cmdReceived_buf;
				spiState <= 0;
			end
		endcase
	end

	// ==============================
	// SPI SHIFT
	// ==============================

	reg [3:0] spiBitCnt;

	always @(posedge sck or posedge ncs) begin
		if(ncs) begin
			spiByteAvailable <= 0;
			spiBitCnt <= 0;
		end else begin
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

	// ==============================
	// ORIGINAL MODULES
	// ==============================

	logic cycleStart;

	CalcPhase #(
		.NUM_CHANNELS(NUM_CHANNELS),
		.MAX_PHASE_CNT(CLK_FREQ/OUT_FREQ),
		.POS_BIT_SIZE(POS_BIT_SIZE),
		.PHASE_BIT_SIZE(PHASE_BIT_SIZE)
	) calcPhase(
		.clk(mathClk),
		.nReset,
		.x(0), .y(0), .z(0),
		.halfHeight(0),
		.XZradiusSquared(0),
		.start(0),
		.phase(calculatedPhase),
		.phaseEnabled(),
		.done(),
		.top(top),
		.left(left),
		.cycleStart(cycleStart)
	);

	PwmCtrl #(
		.NUM_CHANNELS(NUM_CHANNELS),
		.CLK_FREQ(CLK_FREQ),
		.OUT_FREQ(OUT_FREQ)
	) pwmCtrl(
		.clk,
		.nReset,
		.phase,
		.en('{default:1'b1}),
		.out(t),
		.cycleStart,
		.syncin
	);

	assign LEDpwm = 3'b111;

endmodule