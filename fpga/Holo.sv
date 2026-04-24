module Holo #(
    parameter NUM_CHANNELS = 50,
    parameter CLK_FREQ = 20480000,
    parameter OUT_FREQ = 40000
)(
    input clk,
    input nReset,

    output logic [NUM_CHANNELS-1:0] t,
    output logic [2:0] LEDpwm,

    input mosi,
    input sck,
    input ncs,

    input syncin,
    input top,
    input left
);

    // =========================================================
    // CONSTANTS
    // =========================================================
    localparam TOTAL = 200;
    localparam PHASE_BITS = $clog2(CLK_FREQ / OUT_FREQ);

    // =========================================================
    // FAST SPI SHIFT REGISTER (STREAM INPUT)
    // =========================================================
    logic [7:0] shift;
    logic [2:0] bitCnt;
    logic byteValid;

    always @(posedge sck or posedge ncs) begin
        if(ncs) begin
            bitCnt <= 0;
            byteValid <= 0;
        end else begin
            shift <= {shift[6:0], mosi};
            bitCnt <= bitCnt + 1;

            if(bitCnt == 7) begin
                byteValid <= 1;
                bitCnt <= 0;
            end else begin
                byteValid <= 0;
            end
        end
    end

    // =========================================================
    // DUAL FRAME BUFFER (HIGH SPEED DOUBLE BUFFER)
    // =========================================================
    logic [15:0] bufferA [0:TOTAL-1];
    logic [15:0] bufferB [0:TOTAL-1];

    logic writeBank; // 0 = A, 1 = B
    logic [$clog2(TOTAL)-1:0] wptr;

    // write into active buffer
    always @(posedge clk) begin
        if(!nReset) begin
            wptr <= 0;
            writeBank <= 0;
        end else begin

            if(byteValid) begin

                if(writeBank == 0)
                    bufferA[wptr] <= {bufferA[wptr][7:0], shift};
                else
                    bufferB[wptr] <= {bufferB[wptr][7:0], shift};

                if(wptr == TOTAL-1) begin
                    wptr <= 0;
                    writeBank <= ~writeBank;
                end else begin
                    wptr <= wptr + 1;
                end
            end

        end
    end

    // =========================================================
    // READ BUFFER SELECTION (LOCKED FRAME)
    // =========================================================
    logic [15:0] bufferOut [0:TOTAL-1];

    always @(*) begin
        if(writeBank == 0)
            bufferOut = bufferB; // read stable old frame
        else
            bufferOut = bufferA;
    end

    // =========================================================
    // QUADRANT SELECTION (TOP / LEFT)
    // =========================================================
    function automatic [1:0] board_id;
        input top;
        input left;
        begin
            case ({top, left})
                2'b11: board_id = 2'd0;
                2'b10: board_id = 2'd1;
                2'b01: board_id = 2'd2;
                2'b00: board_id = 2'd3;
            endcase
        end
    endfunction

    function automatic [7:0] base_offset;
        input [1:0] id;
        begin
            base_offset = id * 50;
        end
    endfunction

    // =========================================================
    // OUTPUT PIPELINE (FAST PARALLEL UPDATE)
    // =========================================================
    logic [PHASE_BITS-1:0] phase [NUM_CHANNELS];
    logic en [NUM_CHANNELS];

    logic [7:0] base;

    always @(*) begin
        base = base_offset(board_id(top, left));
    end

    integer i;

    always @(posedge clk) begin
        if(!nReset) begin
            for(i=0;i<NUM_CHANNELS;i=i+1) begin
                phase[i] <= 0;
                en[i] <= 0;
            end
        end else begin
            for(i=0;i<NUM_CHANNELS;i=i+1) begin

                // safe indexing into flattened board segment
                phase[i] <= bufferOut[base + i][15:0];
                en[i]    <= bufferOut[base + i][0];

            end
        end
    end

    // =========================================================
    // PWM DRIVER
    // =========================================================
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
        .cycleStart(),

        .syncin(syncin)
    );

    // =========================================================
    // LED DEBUG
    // =========================================================
    assign LEDpwm = 3'b001;

endmodule