module HoloTop (
    input  CLK_24M576,

    // 4 boards × 50 channels (we replicate this module per board)
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

    logic clk, nReset;

    // PLL generates internal clock
    Pll pll(
        .inclk0(CLK_24M576),
        .c0(clk)
    );

    // Reset synchronisation
    Reset reset(
        .clk(clk),
        .nReset(nReset)
    );

    // Generate 40kHz sync signal
    localparam CNT_MAX = CLK_FREQ / OUT_FREQ;
    reg [$clog2(CNT_MAX)-1:0] cnt;

    always @(posedge clk) begin
        if(!nReset) begin
            cnt <= 0;
            syncout <= 0;
        end else begin
            cnt <= (cnt == CNT_MAX-1) ? 0 : cnt + 1;
            syncout <= (cnt < CNT_MAX/2);
        end
    end

    logic [2:0] LEDpwm;
    assign {red, green, blue} = LEDpwm;

    Holo holo (
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