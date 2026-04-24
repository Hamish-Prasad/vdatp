module HoloTop (
    input  CLK_24M576,

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

    logic clk, nReset;

    // PLL generates system clock
    Pll pll(
        .inclk0(CLK_24M576),
        .c0(clk)
    );

    Reset reset(
        .clk(clk),
        .nReset(nReset)
    );

    // =============================
    // SYNC GENERATOR (cycle start)
    // =============================
    localparam CNT_MAX = CLK_FREQ / OUT_FREQ;
    logic [$clog2(CNT_MAX)-1:0] cnt;

    always @(posedge clk) begin
        if(!nReset) begin
            cnt <= 0;
            syncout <= 0;
        end else begin
            cnt <= (cnt == CNT_MAX-1) ? 0 : cnt + 1;
            syncout <= (cnt < CNT_MAX/2);
        end
    end

    // LED output
    logic [2:0] LEDpwm;
    assign {red, green, blue} = LEDpwm;

    // =============================
    // MAIN MODULE
    // =============================
    Holo #(
        .NUM_CHANNELS(NUM_CHANNELS),
        .CLK_FREQ(CLK_FREQ),
        .OUT_FREQ(OUT_FREQ)
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