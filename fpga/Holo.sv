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

	localparam PHASE_BIT_SIZE = $clog2(CLK_FREQ/OUT_FREQ);

	localparam CMD_NOOP        = 4'd0;
	localparam CMD_SET_PHASES  = 4'd11;

	// =========================================================
	// PHASE STORAGE
	// =========================================================
	logic [PHASE_BIT_SIZE-1:0] phase [NUM_CHANNELS];

	// =========================================================
	// SPI COMMAND + BUFFER
	// =========================================================
	logic [3:0] cmdReceived;
	logic [3:0] cmd_buf;

	logic [15:0] spiParameter [NUM_CHANNELS];

	// =========================================================
	// APPLY PHASES
	// =========================================================
	integer i;

	always @(posedge clk) begin
		if (!nReset) begin
			for (i = 0; i < NUM_CHANNELS; i = i + 1)
				phase[i] <= 0;
		end
		else if (cmdReceived == CMD_SET_PHASES) begin
			for (i = 0; i < NUM_CHANNELS; i = i + 1)
				phase[i] <= spiParameter[i][PHASE_BIT_SIZE-1:0];
		end
	end

	// =========================================================
	// SPI BYTE RECEIVER
	// =========================================================
	logic [7:0] spiByte;
	logic [7:0] spiByteBuffer;
	logic spiByteAvailable;
	logic [3:0] spiBitCnt;

	always @(posedge sck or posedge ncs) begin
		if (ncs) begin
			spiBitCnt <= 0;
			spiByteAvailable <= 0;
		end else begin
			spiByte <= {spiByte[6:0], mosi};
			spiBitCnt <= spiBitCnt + 1;

			if (spiBitCnt == 7) begin
				spiByteBuffer <= {spiByte[6:0], mosi};
				spiByteAvailable <= 1;
				spiBitCnt <= 0;
			end else begin
				spiByteAvailable <= 0;
			end
		end
	end

	// =========================================================
	// SPI STATE MACHINE
	// =========================================================
	logic [3:0] spiState;
	logic [3:0] spiByteCnt;

	always @(posedge clk) begin
		if (!nReset) begin
			spiState <= 0;
			cmdReceived <= CMD_NOOP;
		end else begin
			case (spiState)

				0: begin
					cmdReceived <= CMD_NOOP;
					if (ncs == 0)
						spiState <= 1;
				end

				1: begin
					if (ncs == 1)
						spiState <= 0;

					if (spiByteAvailable) begin
						cmd_buf <= spiByteBuffer[3:0];
						spiByteCnt <= 0;
						spiState <= 2;
					end
				end

				2: begin
					if (ncs == 1)
						spiState <= 0;

					if (spiByteAvailable) begin
						spiParameter[spiByteCnt] <= spiByteBuffer;
						spiByteCnt <= spiByteCnt + 1;

						if (cmd_buf == CMD_SET_PHASES &&
						    spiByteCnt == NUM_CHANNELS - 1)
							spiState <= 3;
					end
				end

				3: begin
					if (ncs == 1)
						spiState <= 4;
				end

				4: begin
					cmdReceived <= cmd_buf;
					spiState <= 0;
				end

			endcase
		end
	end

	// =========================================================
	// OUTPUT (UNCHANGED PWM PIPELINE)
	// =========================================================
	PwmCtrl #(
		.NUM_CHANNELS(NUM_CHANNELS),
		.CLK_FREQ(CLK_FREQ),
		.OUT_FREQ(OUT_FREQ)
	) pwmCtrl (
		.clk(clk),
		.nReset(nReset),
		.phase(phase),
		.en('{default:1'b1}),
		.out(t),
		.cycleStart(),
		.syncin(syncin)
	);

	// =========================================================
	// SIMPLE LED (STATIC ON)
	// =========================================================
	assign LEDpwm = 3'b111;

	// UNUSED OUTPUTS TIED OFF
	assign f_sck = 0;
	assign f_ncs = 1;
	assign f_io  = 4'bzzzz;

endmodule