module HoloTop (
	// This module acts as a "hardware" connection point

	// There is a clock somewhere, 24.576 mHz
    input CLK_24M576,
	 // 50 outputs (one per transducer)
	 // wait what 50? don't we have 200??
    output [49:0] t,
	
	// to look at later
    output blue,
    output green,
    output red,

	 // I dont get it but:
	 // mosi -> master slave data
	 // sck -> clock
	 // ncs -> chip
    input mosi,
    input sck,
    input ncs,
	
	// used to sync timing across arrays
    output syncout,
    input syncin
);
	// FPGA clock frequency
    localparam CLK_FREQ = 20480000;
	 // output update rate
    localparam OUT_FREQ = 40000;
	 // amount of transducers TO LOOK AT AS WELL
    localparam NUM_CHANNELS = 50;

	 // I think this part is deprecated
    logic clk;
    logic mathClk;
    logic nReset;

    Pll pll(
        .inclk0(CLK_24M576),
        .c0(clk),
        .c1(mathClk)
    );

	 // resets clock
    Reset reset(
        .clk(clk),
        .nReset(nReset)
    );

    // sync signal
    localparam CLK_CNT_MAX = CLK_FREQ/OUT_FREQ;
    reg [$clog2(CLK_CNT_MAX)-1:0] cnt;

    always @(posedge clk) begin
        if(!nReset) begin
            cnt <= 0;
            syncout <= 0;
        end else begin
            cnt <= (cnt == CLK_CNT_MAX-1) ? 0 : cnt + 1;
            syncout <= (cnt < CLK_CNT_MAX/2);
        end
    end

    logic [2:0] LEDpwm;

    assign red   = LEDpwm[0];
    assign green = LEDpwm[1];
    assign blue  = LEDpwm[2];

    Holo #(
        .CLK_FREQ(CLK_FREQ),
        .OUT_FREQ(OUT_FREQ),
        .NUM_CHANNELS(NUM_CHANNELS)
    ) holo (
        .clk(clk),
        .nReset(nReset),

        .t(t),
        .LEDpwm(LEDpwm),

        .mosi(mosi),
        .sck(sck),
        .ncs(ncs),

        .syncin(syncin)
    );

endmodule