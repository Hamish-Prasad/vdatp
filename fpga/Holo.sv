module Holo #(parameter NUM_CHANNELS = 50,
              parameter CLK_FREQ = 20480000,
              parameter OUT_FREQ = 40000)
(
    input clk,
    input nReset,

    output [NUM_CHANNELS-1:0] t,
    output [2:0] LEDpwm,

    input mosi,
    input sck,
    input ncs,

    input syncin
);

    // =============================
    // Phase format
    // =============================
    localparam PHASE_BITS = $clog2(CLK_FREQ / OUT_FREQ);

    // Double buffering prevents tearing mid-cycle
    logic [PHASE_BITS-1:0] phaseBuf [NUM_CHANNELS];
    logic [PHASE_BITS-1:0] phaseActive [NUM_CHANNELS];

    logic phaseEnabledBuf [NUM_CHANNELS];
    logic phaseEnabledActive [NUM_CHANNELS];

    // =============================
    // SPI (BYTE-BASED)
    // =============================
    logic [7:0] spiShift;
    logic [2:0] bitCnt;
    logic byteReady;

    always @(posedge sck or posedge ncs) begin
        if(ncs) begin
            bitCnt <= 0;
            byteReady <= 0;
        end else begin
            spiShift <= {spiShift[6:0], mosi};
            bitCnt <= bitCnt + 1;

            if(bitCnt == 7) begin
                byteReady <= 1;
                bitCnt <= 0;
            end else begin
                byteReady <= 0;
            end
        end
    end

    // =============================
    // Command parser
    // =============================
    localparam CMD_SET_FRAME = 8'hA5;

    integer idx;
    logic [7:0] rxBuf [NUM_CHANNELS*3]; // phaseL, phaseH, enable

    typedef enum logic [1:0] {IDLE, DATA} state_t;
    state_t state;

    always @(posedge clk) begin
        if(!nReset) begin
            state <= IDLE;
            idx <= 0;
        end else begin
            case(state)
                IDLE:
                    if(byteReady && spiShift == CMD_SET_FRAME) begin
                        idx <= 0;
                        state <= DATA;
                    end

                DATA:
                    if(byteReady) begin
                        rxBuf[idx] <= spiShift;
                        idx <= idx + 1;

                        if(idx == NUM_CHANNELS*3-1)
                            state <= IDLE;
                    end
            endcase
        end
    end

    // =============================
    // Load buffer → active at safe time
    // =============================
    logic cycleStart;

    always @(posedge clk) begin
        if(cycleStart) begin
            for(int i=0;i<NUM_CHANNELS;i++) begin
                phaseActive[i] <= {rxBuf[i*3+1], rxBuf[i*3]}[PHASE_BITS-1:0];
                phaseEnabledActive[i] <= rxBuf[i*3+2][0];
            end
        end
    end

    // =============================
    // PWM driver
    // =============================
    PwmCtrl #(
        .NUM_CHANNELS(NUM_CHANNELS),
        .CLK_FREQ(CLK_FREQ),
        .OUT_FREQ(OUT_FREQ)
    ) pwm (
        .clk(clk),
        .nReset(nReset),
        .phase(phaseActive),
        .en(phaseEnabledActive),
        .out(t),
        .cycleStart(cycleStart),
        .syncin(syncin)
    );

    assign LEDpwm = 3'b010;

endmodule