module Holo #(parameter CLK_FREQ=20480000,
              parameter OUT_FREQ=40000,
              parameter NUM_CHANNELS=50)
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

    localparam PHASE_BITS = $clog2(CLK_FREQ/OUT_FREQ);
    localparam CMD_SET_PHASES = 4'd11;

    // =========================
    // Phase storage
    // =========================
    logic [PHASE_BITS-1:0] phase [NUM_CHANNELS];
    logic phaseEnabled [NUM_CHANNELS];

    initial begin
        for(int i=0;i<NUM_CHANNELS;i++)
            phaseEnabled[i] = 1;
    end

    // =========================
    // SPI receiver
    // =========================
    logic [7:0] spiByte;
    logic [2:0] bitCnt;
    logic byteReady;

    always @(posedge sck or posedge ncs) begin
        if(ncs) begin
            bitCnt <= 0;
            byteReady <= 0;
        end else begin
            spiByte <= {spiByte[6:0], mosi};
            bitCnt <= bitCnt + 1;
            byteReady <= 0;

            if(bitCnt == 7) begin
                byteReady <= 1;
                bitCnt <= 0;
            end
        end
    end

    // =========================
    // SPI command parser
    // =========================
    logic [3:0] cmd;
    logic [7:0] buffer [NUM_CHANNELS*2];
    integer idx;

    typedef enum logic [1:0] {
        IDLE,
        CMD,
        DATA
    } state_t;

    state_t state;

    always @(posedge clk) begin
        if(!nReset) begin
            state <= IDLE;
            idx <= 0;
        end else begin
            case(state)
                IDLE:
                    if(!ncs)
                        state <= CMD;

                CMD:
                    if(byteReady) begin
                        cmd <= spiByte[4:1];
                        idx <= 0;
                        state <= DATA;
                    end

                DATA:
                    if(byteReady) begin
                        buffer[idx] <= spiByte;
                        idx <= idx + 1;

                        if(idx == NUM_CHANNELS*2-1) begin
                            state <= IDLE;

                            if(cmd == CMD_SET_PHASES) begin
										logic [15:0] temp;  // adjust width if needed

										for(int i = 0; i < NUM_CHANNELS; i++) begin
											 temp = {buffer[i*2+1], buffer[i*2]};
											 phase[i] <= temp[PHASE_BITS-1:0];
										end
                            end
                        end
                    end
            endcase
        end
    end

    // =========================
    // PWM output
    // =========================
    logic cycleStart;

    PwmCtrl #(
        .NUM_CHANNELS(NUM_CHANNELS),
        .CLK_FREQ(CLK_FREQ),
        .OUT_FREQ(OUT_FREQ)
    ) pwm (
        .clk(clk),
        .nReset(nReset),
        .phase(phase),
        .en(phaseEnabled),
        .out(t),
        .cycleStart(cycleStart),
        .syncin(syncin)
    );

    // =========================
    // Simple LED
    // =========================
    assign LEDpwm = 3'b101;

endmodule