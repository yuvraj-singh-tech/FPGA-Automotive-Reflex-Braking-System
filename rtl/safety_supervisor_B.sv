`timescale 1ns/1ps

module safety_supervisor_B #(
    // ------------------------------------------------------------
    // Same external policy knobs as A
    // ------------------------------------------------------------
    parameter int unsigned EMERG_HARD_CONFIRM_MS    = 20,   // unused in B (kept for parity)
    parameter int unsigned EMERG_MIN_HOLD_MS        = 150,
    parameter int unsigned EMERG_RELEASE_CONFIRM_MS = 60,

    // Invalid intent policy
    parameter int unsigned INVALID_HOLD_MS          = 50,
    parameter int unsigned INVALID_FAULT_MS         = 300,

    // Policy knobs
    parameter bit          PANIC_ALWAYS_EMERG       = 1'b1,
    parameter bit          INTENT_FAULT_IS_FATAL    = 1'b1,

    // ------------------------------------------------------------
    // B-specific risk integrator tuning
    // ------------------------------------------------------------
    parameter int unsigned SCORE_MAX                = 255,
    parameter int unsigned SCORE_HARD_UP            = 8,
    parameter int unsigned SCORE_MED_UP             = 4,
    parameter int unsigned SCORE_LIGHT_UP           = 1,
    parameter int unsigned SCORE_NONE_DOWN          = 3,
    parameter int unsigned SCORE_LIGHT_DOWN         = 2,
    parameter int unsigned SCORE_MED_DOWN           = 1,
    parameter int unsigned SCORE_INVALID_DOWN       = 8,

    parameter int unsigned EMERG_SCORE_ENTER        = 40,
    parameter int unsigned EMERG_SCORE_EXIT         = 12
) (
    input  logic       clk,
    input  logic       rst,
    input  logic       tick_1khz,

    input  logic       drv_valid,
    input  logic [1:0] drv_level,
    input  logic       panic_brake,
    input  logic       release_ok,
    input  logic       intent_fault,

    output logic       allow_brake,
    output logic       emergency_req,
    output logic       fault_req,
    output logic [2:0] supv_state
);

    // ------------------------------------------------------------
    // State encoding
    // ------------------------------------------------------------
    localparam logic [2:0] S_OK      = 3'd0;
    localparam logic [2:0] S_EMERG   = 3'd1;
    localparam logic [2:0] S_RELEASE = 3'd2;
    localparam logic [2:0] S_FAULT   = 3'd3;

    // ------------------------------------------------------------
    // Constants
    // ------------------------------------------------------------
    localparam logic [15:0] EMERG_MIN_HOLD_MS_16        = EMERG_MIN_HOLD_MS[15:0];
    localparam logic [15:0] EMERG_RELEASE_CONFIRM_MS_16 = EMERG_RELEASE_CONFIRM_MS[15:0];
    localparam logic [15:0] INVALID_HOLD_MS_16          = INVALID_HOLD_MS[15:0];
    localparam logic [15:0] INVALID_FAULT_MS_16         = INVALID_FAULT_MS[15:0];

    localparam logic [7:0] SCORE_MAX_8         = SCORE_MAX[7:0];
    localparam logic [7:0] EMERG_SCORE_ENTER_8 = EMERG_SCORE_ENTER[7:0];
    localparam logic [7:0] EMERG_SCORE_EXIT_8  = EMERG_SCORE_EXIT[7:0];

    // ------------------------------------------------------------
    // Registers
    // ------------------------------------------------------------
    logic [15:0] emerg_hold_cnt;
    logic [15:0] release_cnt;
    logic [15:0] invalid_cnt;
    logic [15:0] invalid_hold;
    logic [7:0]  risk_score;

    // ------------------------------------------------------------
    // Combinational helpers
    // ------------------------------------------------------------
    logic [15:0] invalid_cnt_next;
    logic        fatal_now;
    logic [7:0]  score_next;

    // ------------------------------------------------------------
    // Driver-first invariant
    // ------------------------------------------------------------
    always_comb begin
        allow_brake = 1'b1;
    end

    // ------------------------------------------------------------
    // Saturation helpers (Vivado-safe)
    // ------------------------------------------------------------
    function automatic logic [7:0] sat_add8_max(
        input logic [7:0] a,
        input int unsigned b,
        input logic [7:0] maxv
    );
        logic [15:0] sum;
        begin
            sum = {8'd0, a} + b[15:0];
            if (sum[15:8] != 0 || sum[7:0] > maxv)
                sat_add8_max = maxv;
            else
                sat_add8_max = sum[7:0];
        end
    endfunction

    function automatic logic [7:0] sat_sub8_zero(
        input logic [7:0] a,
        input int unsigned b
    );
        logic [15:0] diff;
        begin
            if ({8'd0,a} <= b[15:0])
                sat_sub8_zero = 8'd0;
            else begin
                diff = {8'd0,a} - b[15:0];
                sat_sub8_zero = diff[7:0];
            end
        end
    endfunction

    // ------------------------------------------------------------
    // Combinational next-state logic
    // ------------------------------------------------------------
    always_comb begin
        // invalid counter
        if (!drv_valid)
            invalid_cnt_next = (invalid_cnt == 16'hFFFF) ? invalid_cnt : invalid_cnt + 16'd1;
        else
            invalid_cnt_next = 16'd0;

        // fatal_now (same-tick)
        fatal_now =
            fault_req ||
            (INTENT_FAULT_IS_FATAL && intent_fault) ||
            (!drv_valid && (invalid_cnt_next >= INVALID_FAULT_MS_16));

        // risk integrator
        score_next = risk_score;

        if (!drv_valid) begin
            score_next = sat_sub8_zero(score_next, SCORE_INVALID_DOWN);
        end else begin
            case (drv_level)
                2'd3: score_next = sat_add8_max(score_next, SCORE_HARD_UP,  SCORE_MAX_8);
                2'd2: score_next = sat_add8_max(score_next, SCORE_MED_UP,   SCORE_MAX_8);
                2'd1: score_next = sat_add8_max(score_next, SCORE_LIGHT_UP, SCORE_MAX_8);
                default: score_next = sat_sub8_zero(score_next, SCORE_NONE_DOWN);
            endcase

            if (drv_level == 2'd1)
                score_next = sat_sub8_zero(score_next, SCORE_LIGHT_DOWN);

            if (drv_level == 2'd2)
                score_next = sat_sub8_zero(score_next, SCORE_MED_DOWN);
        end
    end

    // ------------------------------------------------------------
    // Main sequential logic
    // ------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            supv_state     <= S_OK;
            emergency_req  <= 1'b0;
            fault_req      <= 1'b0;
            emerg_hold_cnt <= 16'd0;
            release_cnt    <= 16'd0;
            invalid_cnt    <= 16'd0;
            invalid_hold   <= 16'd0;
            risk_score     <= 8'd0;

        end else if (tick_1khz) begin

            // Sticky fault
            if (INTENT_FAULT_IS_FATAL && intent_fault)
                fault_req <= 1'b1;
            if (!drv_valid && (invalid_cnt_next >= INVALID_FAULT_MS_16))
                fault_req <= 1'b1;

            // Invalid tracking
            if (!drv_valid) begin
                invalid_cnt <= invalid_cnt_next;
                if (invalid_hold < INVALID_HOLD_MS_16)
                    invalid_hold <= invalid_hold + 16'd1;
            end else begin
                invalid_cnt  <= 16'd0;
                invalid_hold <= 16'd0;
            end

            // Default commit score
            risk_score <= score_next;

            if (fatal_now) begin
                supv_state     <= S_FAULT;
                emergency_req  <= 1'b0;
                emerg_hold_cnt <= 16'd0;
                release_cnt    <= 16'd0;
                risk_score     <= 8'd0;

            end else begin
                case (supv_state)

                    S_OK: begin
                        emergency_req  <= 1'b0;
                        release_cnt    <= 16'd0;
                        emerg_hold_cnt <= 16'd0;

                        if (PANIC_ALWAYS_EMERG && panic_brake) begin
                            emergency_req  <= 1'b1;
                            supv_state     <= S_EMERG;
                            emerg_hold_cnt <= EMERG_MIN_HOLD_MS_16;

                        // =====================================================
                        // FIX (ARBSB): Only allow integrator-based entry on HARD
                        // =====================================================
                        end else if (drv_valid && (drv_level == 2'd3) &&
                                     (score_next >= EMERG_SCORE_ENTER_8)) begin
                            emergency_req  <= 1'b1;
                            supv_state     <= S_EMERG;
                            emerg_hold_cnt <= EMERG_MIN_HOLD_MS_16;
                        end
                    end

                    S_EMERG: begin
                        emergency_req <= 1'b1;

                        if (emerg_hold_cnt != 0)
                            emerg_hold_cnt <= emerg_hold_cnt - 16'd1;

                        if (PANIC_ALWAYS_EMERG && panic_brake)
                            emerg_hold_cnt <= EMERG_MIN_HOLD_MS_16;

                        if (emerg_hold_cnt == 0) begin
                            if (drv_valid && (drv_level <= 2'd1) &&
                                (score_next <= EMERG_SCORE_EXIT_8)) begin
                                if (release_cnt != 16'hFFFF)
                                    release_cnt <= release_cnt + 16'd1;
                            end else begin
                                release_cnt <= 16'd0;
                            end

                            if (release_ok && release_cnt < 16'd10)
                                release_cnt <= 16'd10;

                            if (release_cnt >= EMERG_RELEASE_CONFIRM_MS_16) begin
                                emergency_req <= 1'b0;
                                supv_state    <= S_RELEASE;
                                release_cnt   <= 16'd0;
                            end
                        end
                    end

                    S_RELEASE: begin
                        emergency_req <= 1'b0;

                        if (PANIC_ALWAYS_EMERG && panic_brake) begin
                            emergency_req  <= 1'b1;
                            supv_state     <= S_EMERG;
                            emerg_hold_cnt <= EMERG_MIN_HOLD_MS_16;
                        end else begin
                            supv_state <= S_OK;
                        end
                    end

                    default: begin
                        supv_state    <= S_OK;
                        emergency_req <= 1'b0;
                    end
                endcase
            end
        end
    end

endmodule
