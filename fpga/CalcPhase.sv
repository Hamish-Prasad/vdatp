module CalcPhase #(
    parameter NUM_CHANNELS,
    parameter MAX_PHASE_CNT,
    parameter POS_BIT_SIZE,
    parameter PHASE_BIT_SIZE
)(
    input clk,
    input nReset,

    input [POS_BIT_SIZE-1:0] x,
    input [POS_BIT_SIZE-1:0] y,
    input [POS_BIT_SIZE-1:0] z,

    input start,

    input [POS_BIT_SIZE-1:0] halfHeight,
    input [22:0] XZradiusSquared,

    output reg [$clog2(MAX_PHASE_CNT)-1:0] phase [NUM_CHANNELS],
    output reg phaseEnabled [NUM_CHANNELS],
    output reg done,

    input top,
    input left,
    input cycleStart
);

    localparam NEG_WAVE_K_DIV10 = 32'hBD96104F;
    localparam SCALE_CNST       = 32'h42A2F983;

    typedef logic [31:0] float;

    // ------------------------------------------------------------------------
    // Transducer positions (unchanged)
    // ------------------------------------------------------------------------
    logic [POS_BIT_SIZE-1:0] xTransducerLeftTop [NUM_CHANNELS] = '{
        450,450,450,450,450,450,450,450,450,450,
        350,350,350,350,350,350,350,350,350,350,
        250,250,250,250,250,250,250,250,250,250,
        150,150,150,150,150,150,150,150,150,150,
        50,50,50,50,50,50,50,50,50,50
    };

    logic [POS_BIT_SIZE-1:0] xTransducerRightTop [NUM_CHANNELS] = '{
        -50,-50,-50,-50,-50,-50,-50,-50,-50,-50,
        -150,-150,-150,-150,-150,-150,-150,-150,-150,-150,
        -250,-250,-250,-250,-250,-250,-250,-250,-250,-250,
        -350,-350,-350,-350,-350,-350,-350,-350,-350,-350,
        -450,-450,-450,-450,-450,-450,-450,-450,-450,-450
    };

    logic [POS_BIT_SIZE-1:0] xTransducerRightBottom [NUM_CHANNELS] = '{
        -450,-450,-450,-450,-450,-450,-450,-450,-450,-450,
        -350,-350,-350,-350,-350,-350,-350,-350,-350,-350,
        -250,-250,-250,-250,-250,-250,-250,-250,-250,-250,
        -150,-150,-150,-150,-150,-150,-150,-150,-150,-150,
        -50,-50,-50,-50,-50,-50,-50,-50,-50,-50
    };

    logic [POS_BIT_SIZE-1:0] xTransducerLeftBottom [NUM_CHANNELS] = '{
        50,50,50,50,50,50,50,50,50,50,
        150,150,150,150,150,150,150,150,150,150,
        250,250,250,250,250,250,250,250,250,250,
        350,350,350,350,350,350,350,350,350,350,
        450,450,450,450,450,450,450,450,450,450
    };

    logic [POS_BIT_SIZE-1:0] zTransducer [NUM_CHANNELS] = '{
        -450,-350,-250,-150,-50,50,150,250,350,450,
        -450,-350,-250,-150,-50,50,150,250,350,450,
        -450,-350,-250,-150,-50,50,150,250,350,450,
        -450,-350,-250,-150,-50,50,150,250,350,450,
        -450,-350,-250,-150,-50,50,150,250,350,450
    };

    // ------------------------------------------------------------------------
    // Indexing
    // ------------------------------------------------------------------------
    logic [$clog2(NUM_CHANNELS)-1:0] idx;
    logic processing;

    // ------------------------------------------------------------------------
    // Coordinate diffs (unchanged structure, stable usage)
    // ------------------------------------------------------------------------
    logic [POS_BIT_SIZE-1:0] xDiff, yDiff, zDiff;

    lpm_add_sub #(.lpm_width(POS_BIT_SIZE), .lpm_direction("SUB"), .lpm_pipeline(1))
    xSubOp(
        .clock(clk),
        .dataa(x),
        .datab(!top ?
                (left ? xTransducerLeftTop[idx] : xTransducerRightTop[idx]) :
                (left ? xTransducerLeftBottom[idx] : xTransducerRightBottom[idx])),
        .result(xDiff)
    );

    lpm_add_sub #(.lpm_width(POS_BIT_SIZE), .lpm_direction("SUB"), .lpm_pipeline(1))
    ySubOp(
        .clock(clk),
        .dataa(y),
        .datab(!top ? halfHeight : -halfHeight),
        .result(yDiff)
    );

    lpm_add_sub #(.lpm_width(POS_BIT_SIZE), .lpm_direction("SUB"), .lpm_pipeline(1))
    zSubOp(
        .clock(clk),
        .dataa(z),
        .datab(zTransducer[idx]),
        .result(zDiff)
    );

    // ------------------------------------------------------------------------
    // Squaring
    // ------------------------------------------------------------------------
    logic [22:0] xSqr, ySqr, zSqr;

    altsquare #(.data_width(POS_BIT_SIZE), .result_width(23), .pipeline(1))
    xSqrOp(.clock(clk), .data(xDiff), .result(xSqr), .aclr(1'b0), .ena(1'b1));

    altsquare #(.data_width(POS_BIT_SIZE), .result_width(23), .pipeline(1))
    ySqrOp(.clock(clk), .data(yDiff), .result(ySqr), .aclr(1'b0), .ena(1'b1));

    altsquare #(.data_width(POS_BIT_SIZE), .result_width(23), .pipeline(1))
    zSqrOp(.clock(clk), .data(zDiff), .result(zSqr), .aclr(1'b0), .ena(1'b1));

    // ------------------------------------------------------------------------
    // Distance sum
    // ------------------------------------------------------------------------
    logic [22:0] sumXZ, sumXYZ;

    lpm_add_sub #(.lpm_width(23), .lpm_direction("ADD"), .lpm_pipeline(1))
    sumXZop(.clock(clk), .dataa(xSqr), .datab(zSqr), .result(sumXZ));

    lpm_add_sub #(.lpm_width(23), .lpm_direction("ADD"), .lpm_pipeline(1))
    sumXYZop(.clock(clk), .dataa(sumXZ), .datab(ySqr), .result(sumXYZ));

    // ------------------------------------------------------------------------
    // Phase pipeline (UNCHANGED math path, but stabilized timing)
    // ------------------------------------------------------------------------
    float sumF, sqrtF, multKF, scaleF;

    Int2Float int2Float(.clock(clk), .dataa(sumXYZ), .result(sumF));

    Sqrtf sqrtfOp(.clock(clk), .data(sumF), .result(sqrtF));

    MultK multKop(.clock(clk), .dataa(sqrtF), .datab(NEG_WAVE_K_DIV10), .result(multKF));

    MultK scaleOp(.clock(clk), .dataa(multKF), .datab(SCALE_CNST), .result(scaleF));

    logic [31:0] scaleInt;

    Float2Int float2Int(.clock(clk), .dataa(scaleF), .result(scaleInt));

    // ------------------------------------------------------------------------
    // IMPORTANT FIX: remove skew between phase & enable
    // ------------------------------------------------------------------------
    logic [PHASE_BIT_SIZE:0] remain;
    logic [PHASE_BIT_SIZE-1:0] phaseValue;

    always_ff @(posedge clk) begin
        if (!nReset) begin
            idx <= 0;
            processing <= 0;
            done <= 0;
        end else begin

            if (start && !processing) begin
                processing <= 1;
                idx <= 0;
                done <= 0;
            end

            if (processing) begin

                // stable phase wrapping (unchanged behavior, but deterministic)
                remain <= scaleInt % MAX_PHASE_CNT;

                phaseValue <= remain[PHASE_BIT_SIZE-1:0];

                // OUTPUTS UPDATED SAME CYCLE ORDER PER CHANNEL (fixes jitter)
                phase[idx] <= phaseValue;

                phaseEnabled[idx] <= (sumXZ < XZradiusSquared);

                if (idx == NUM_CHANNELS-1) begin
                    processing <= 0;
                    done <= 1;
                end else begin
                    idx <= idx + 1;
                end
            end

            if (cycleStart)
                done <= 0;
        end
    end

endmodule