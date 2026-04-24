module HoloTop (
    input CLK_24M576,
    output [49:0] t,

    output blue,
    output green,
    output red,

    input mosi,
    input sck,
    input ncs,

    output syncout,
    input syncin
);

    localparam CLK_FREQ = 20480000;
    localparam OUT_FREQ = 40000;
    localparam NUM_CHANNELS = 50;

    logic clk;
    logic mathClk;
    logic nReset;

    Pll pll(
        .inclk0(CLK_24M576),
        .c0(clk),
        .c1(mathClk)
    );

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