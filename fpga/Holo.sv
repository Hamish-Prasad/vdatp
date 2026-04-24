module Holo #(parameter NUM_CHANNELS = 50,
              parameter CLK_FREQ = 20480000,
              parameter OUT_FREQ = 40000)
(
    input clk,
    input nReset,

    output logic [NUM_CHANNELS-1:0] t,
    output logic [2:0] LEDpwm,

    input mosi,
    input sck,
    input ncs,

    input syncin
);

    localparam PHASE_BITS = $clog2(CLK_FREQ / OUT_FREQ);
    localparam CMD_FRAME  = 8'hA5;

    // =============================
    // BUFFER (written by SPI, used by PWM)
    // =============================
    logic [PHASE_BITS-1:0] phase [NUM_CHANNELS];
    logic en [NUM_CHANNELS];

    logic [PHASE_BITS-1:0] phase_next [NUM_CHANNELS];
    logic en_next [NUM_CHANNELS];

    // =============================
    // SPI BYTE RECEIVER
    // =============================
    logic [7:0] shift;
    logic [2:0] bitCnt;
    logic byteReady;

    always @(posedge sck or posedge ncs) begin
        if(ncs) begin
            bitCnt <= 0;
            byteReady <= 0;
        end else begin
            shift <= {shift[6:0], mosi};
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
    // FRAME PARSER
    // =============================
    logic [7:0] rx [NUM_CHANNELS*3];
    integer idx;

    typedef enum logic {WAIT, READ} state_t;
    state_t state;

    always @(posedge clk) begin
        if(!nReset) begin
            state <= WAIT;
            idx <= 0;
        end else begin
            case(state)

                WAIT:
                    if(byteReady && shift == CMD_FRAME) begin
                        idx <= 0;
                        state <= READ;
                    end

                READ:
                    if(byteReady) begin
                        rx[idx] <= shift;
                        idx <= idx + 1;

                        if(idx == NUM_CHANNELS*3-1)
                            state <= WAIT;
                    end
            endcase
        end
    end

    // =============================
    // COMMIT ON CYCLE START
    // =============================
    logic cycleStart;

    always @(posedge clk) begin
        if(cycleStart) begin
            for(int i=0;i<NUM_CHANNELS;i++) begin
                phase_next[i] <= {rx[i*3+1], rx[i*3]}[PHASE_BITS-1:0];
                en_next[i]    <= rx[i*3+2][0];
            end

            phase <= phase_next;
            en    <= en_next;
        end
    end

    // =============================
    // PWM DRIVER
    // =============================
    PwmCtrl #(
        .NUM_CHANNELS(NUM_CHANNELS),
        .CLK_FREQ(CLK_FREQ),
        .OUT_FREQ(OUT_FREQ)
    ) pwm (
        .clk(clk),
        .nReset(nReset),

        .phase(phase),
        .en(en),

        .out(t),
        .cycleStart(cycleStart),

        .syncin(syncin)
    );

    assign LEDpwm = 3'b001;

endmodule