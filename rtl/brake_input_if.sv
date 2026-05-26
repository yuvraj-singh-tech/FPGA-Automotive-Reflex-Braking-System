`timescale 1ns/1ps

// =============================================================================
// Module: brake_input_if
// -----------------------------------------------------------------------------
// Converts raw driver brake input into clean, validated braking intent signals.
//
// The interface supports two input paths:
//   1. Analog brake magnitude through a 12-bit ADC
//   2. Digital brake switch fallback
//
// Main functions:
//   - Synchronizes the raw digital brake input
//   - Holds recent ADC validity for short dropouts
//   - Rejects implausible ADC samples using range and jump checks
//   - Detects stuck ADC behavior during braking intent
//   - Debounces brake press/release events
//   - Classifies brake intensity into NONE / LIGHT / MEDIUM / HARD
//   - Uses hysteresis to avoid level chatter near thresholds
//
// Design note:
//   The ADC path is preferred when healthy. If the ADC path is unavailable or
//   faulty, the synchronized digital brake input is used as a safe fallback.
// =============================================================================

module brake_input_if #(
    parameter int unsigned DEBOUNCE_MS_ON   = 30,
    parameter int unsigned DEBOUNCE_MS_OFF  = 50,
    parameter int unsigned VALID_HOLD_MS    = 200,

    parameter int unsigned ADC_MAX          = 4095,
    parameter int unsigned ADC_MIN_OK       = 0,
    parameter int unsigned ADC_MAX_OK       = 4095,

    parameter int unsigned ADC_JUMP_MAX     = 450,
    parameter int unsigned ADC_STUCK_EPS    = 3,
    parameter int unsigned STUCK_TIMEOUT_MS = 1200,

    parameter int unsigned THR_LIGHT_ENTER  = 400,
    parameter int unsigned THR_MED_ENTER    = 1200,
    parameter int unsigned THR_HARD_ENTER   = 2400,

    parameter int unsigned THR_LIGHT_EXIT   = 300,
    parameter int unsigned THR_MED_EXIT     = 1000,
    parameter int unsigned THR_HARD_EXIT    = 2100
) (
    input  logic        clk,
    input  logic        rst,
    input  logic        tick_1khz,

    input  logic        brake_raw,
    input  logic [11:0] brake_adc,
    input  logic        brake_adc_valid,

    output logic        brake_valid,
    output logic        brake_active,
    output logic [1:0]  brake_level,
    output logic        brake_hard,

    output logic        adc_used,
    output logic        fault_stuck,
    output logic        fault_spike,
    output logic        fault_range
);

    // -------------------------------------------------------------------------
    // Local 12-bit parameter views
    // -------------------------------------------------------------------------
    // These casts keep threshold comparisons width-clean and avoid accidental
    // signed/int promotion issues during synthesis or simulation.
    // -------------------------------------------------------------------------
    localparam logic [11:0] THR_LIGHT_ENTER_12 = THR_LIGHT_ENTER[11:0];
    localparam logic [11:0] THR_MED_ENTER_12   = THR_MED_ENTER[11:0];
    localparam logic [11:0] THR_HARD_ENTER_12  = THR_HARD_ENTER[11:0];

    localparam logic [11:0] THR_LIGHT_EXIT_12  = THR_LIGHT_EXIT[11:0];
    localparam logic [11:0] THR_MED_EXIT_12    = THR_MED_EXIT[11:0];
    localparam logic [11:0] THR_HARD_EXIT_12   = THR_HARD_EXIT[11:0];

    localparam logic [11:0] ADC_MIN_OK_12      = ADC_MIN_OK[11:0];
    localparam logic [11:0] ADC_MAX_OK_12      = ADC_MAX_OK[11:0];
    localparam logic [11:0] ADC_JUMP_MAX_12    = ADC_JUMP_MAX[11:0];
    localparam logic [11:0] ADC_STUCK_EPS_12   = ADC_STUCK_EPS[11:0];

    // -------------------------------------------------------------------------
    // Absolute difference helper for 12-bit ADC samples
    // -------------------------------------------------------------------------
    function automatic [11:0] abs_diff12(input [11:0] a, input [11:0] b);
        if (a >= b) abs_diff12 = a - b;
        else        abs_diff12 = b - a;
    endfunction

    // -------------------------------------------------------------------------
    // Digital brake input synchronizer
    // -------------------------------------------------------------------------
    // The external brake switch can be asynchronous to the FPGA clock. A two-flop
    // synchronizer reduces metastability risk before the signal is used by logic.
    // -------------------------------------------------------------------------
    logic brake_raw_ff1, brake_raw_ff2;
    logic brake_raw_sync;

    always_ff @(posedge clk) begin
        if (rst) begin
            brake_raw_ff1 <= 1'b0;
            brake_raw_ff2 <= 1'b0;
        end else begin
            brake_raw_ff1 <= brake_raw;
            brake_raw_ff2 <= brake_raw_ff1;
        end
    end

    assign brake_raw_sync = brake_raw_ff2;

    // -------------------------------------------------------------------------
    // ADC validity hold
    // -------------------------------------------------------------------------
    // A recently valid ADC sample remains usable for a short hold window. This
    // prevents brief valid-signal dropouts from immediately forcing fallback mode.
    // -------------------------------------------------------------------------
    logic        adc_valid_hold;
    logic [15:0] adc_valid_hold_cnt;

    always_ff @(posedge clk) begin
        if (rst) begin
            adc_valid_hold     <= 1'b0;
            adc_valid_hold_cnt <= 16'd0;
        end else if (tick_1khz) begin
            if (brake_adc_valid) begin
                adc_valid_hold     <= 1'b1;
                adc_valid_hold_cnt <= 16'd0;
            end else if (adc_valid_hold) begin
                if (adc_valid_hold_cnt >= VALID_HOLD_MS[15:0]) begin
                    adc_valid_hold <= 1'b0;
                end else begin
                    adc_valid_hold_cnt <= adc_valid_hold_cnt + 16'd1;
                end
            end
        end
    end

    logic adc_candidate;
    assign adc_candidate = brake_adc_valid || adc_valid_hold;

    // -------------------------------------------------------------------------
    // Provisional brake magnitude
    // -------------------------------------------------------------------------
    // Used before final ADC health is known. When ADC data is available, the last
    // accepted ADC value is used. Otherwise, the digital brake input maps to HARD.
    // -------------------------------------------------------------------------
    logic [11:0] adc_last_good;
    logic [11:0] eff_mag_prov;
    logic        pressed_raw_prov;

    assign eff_mag_prov =
        adc_candidate ? adc_last_good :
        (brake_raw_sync ? THR_HARD_ENTER_12 : 12'd0);

    assign pressed_raw_prov = (eff_mag_prov >= THR_LIGHT_ENTER_12);

    // -------------------------------------------------------------------------
    // ADC plausibility checks and accepted-sample tracking
    // -------------------------------------------------------------------------
    // Range fault  : ADC value outside configured valid limits
    // Spike fault  : ADC jump exceeds allowed step after baseline sample exists
    // Stuck fault  : ADC remains nearly unchanged during braking intent too long
    //
    // Fault outputs are sticky until reset.
    // -------------------------------------------------------------------------
    logic [15:0] stuck_cnt;
    logic [1:0]  adc_sample_cnt;

    logic range_now, spike_now, accept_now;

    assign range_now =
        brake_adc_valid &&
        ((brake_adc < ADC_MIN_OK_12) || (brake_adc > ADC_MAX_OK_12));

    assign spike_now =
        brake_adc_valid &&
        (adc_sample_cnt >= 2'd1) &&
        (abs_diff12(brake_adc, adc_last_good) > ADC_JUMP_MAX_12);

    assign accept_now =
        brake_adc_valid &&
        ((adc_sample_cnt < 2'd1) || !(range_now || spike_now));

    logic [15:0] stuck_cnt_next;

    always_comb begin
        stuck_cnt_next = stuck_cnt;

        if (brake_adc_valid) begin
            if (accept_now && (adc_sample_cnt >= 2'd1) && pressed_raw_prov) begin
                if (abs_diff12(brake_adc, adc_last_good) <= ADC_STUCK_EPS_12) begin
                    stuck_cnt_next = (stuck_cnt == 16'hFFFF) ? stuck_cnt
                                                             : (stuck_cnt + 16'd1);
                end else begin
                    stuck_cnt_next = 16'd0;
                end
            end else begin
                stuck_cnt_next = 16'd0;
            end
        end else begin
            if (!adc_candidate || !pressed_raw_prov)
                stuck_cnt_next = 16'd0;
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            fault_range    <= 1'b0;
            fault_spike    <= 1'b0;
            fault_stuck    <= 1'b0;
            stuck_cnt      <= 16'd0;
            adc_last_good  <= 12'd0;
            adc_sample_cnt <= 2'd0;
        end else if (tick_1khz) begin
            if (range_now) fault_range <= 1'b1;
            if (spike_now) fault_spike <= 1'b1;

            stuck_cnt <= stuck_cnt_next;

            if (brake_adc_valid) begin
                if (accept_now) begin
                    adc_last_good <= brake_adc;

                    if (adc_sample_cnt != 2'd2)
                        adc_sample_cnt <= adc_sample_cnt + 2'd1;
                end

                if (adc_candidate && pressed_raw_prov) begin
                    if (stuck_cnt_next >= STUCK_TIMEOUT_MS[15:0])
                        fault_stuck <= 1'b1;
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // ADC health summary
    // -------------------------------------------------------------------------
    // During active braking intent, stuck detection is treated as part of ADC
    // health. Outside braking intent, range and spike faults are still monitored.
    // -------------------------------------------------------------------------
    logic adc_healthy_final;

    assign adc_healthy_final =
        pressed_raw_prov ? ~(fault_range | fault_spike | fault_stuck)
                         : ~(fault_range | fault_spike);

    // -------------------------------------------------------------------------
    // Final source selection
    // -------------------------------------------------------------------------
    // ADC is preferred when available and healthy. Otherwise, the synchronized
    // digital brake input provides a deterministic fallback path.
    // -------------------------------------------------------------------------
    logic [11:0] eff_mag;
    logic        use_adc;

    assign use_adc  = adc_candidate && adc_healthy_final;
    assign adc_used = use_adc;

    assign eff_mag  = use_adc ? adc_last_good :
                      (brake_raw_sync ? THR_HARD_ENTER_12 : 12'd0);

    assign brake_valid = use_adc ? adc_healthy_final : 1'b1;

    // -------------------------------------------------------------------------
    // Debounced brake activity detection
    // -------------------------------------------------------------------------
    // Separate press and release windows prevent chatter around the activation
    // threshold and make brake_active stable for downstream safety logic.
    // -------------------------------------------------------------------------
    logic pressed_raw;
    logic [15:0] press_cnt, release_cnt;

    assign pressed_raw = brake_valid && (eff_mag >= THR_LIGHT_ENTER_12);

    always_ff @(posedge clk) begin
        if (rst) begin
            brake_active <= 1'b0;
            press_cnt    <= 16'd0;
            release_cnt  <= 16'd0;
        end else if (tick_1khz) begin
            if (!brake_active) begin
                if (pressed_raw) begin
                    if (press_cnt < DEBOUNCE_MS_ON[15:0])
                        press_cnt <= press_cnt + 16'd1;
                end else begin
                    press_cnt <= 16'd0;
                end

                if (pressed_raw && ((press_cnt + 16'd1) >= DEBOUNCE_MS_ON[15:0]))
                    brake_active <= 1'b1;

                release_cnt <= 16'd0;
            end else begin
                if (!pressed_raw) begin
                    if (release_cnt < DEBOUNCE_MS_OFF[15:0])
                        release_cnt <= release_cnt + 16'd1;
                end else begin
                    release_cnt <= 16'd0;
                end

                if (!pressed_raw && ((release_cnt + 16'd1) >= DEBOUNCE_MS_OFF[15:0]))
                    brake_active <= 1'b0;

                press_cnt <= 16'd0;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Brake level classification with hysteresis
    // -------------------------------------------------------------------------
    // Encoded brake_level:
    //   2'd0 = none
    //   2'd1 = light
    //   2'd2 = medium
    //   2'd3 = hard
    //
    // Enter and exit thresholds are separated to avoid rapid switching near
    // boundary values. Digital fallback maps an active brake directly to HARD.
    // -------------------------------------------------------------------------
    logic [1:0] level_next;

    always_comb begin
        if (!use_adc && brake_active) begin
            level_next = 2'd3;
        end
        else if (!brake_valid || !brake_active) begin
            level_next = 2'd0;
        end
        else begin
            case (brake_level)
                2'd0: level_next = (eff_mag >= THR_HARD_ENTER_12)  ? 2'd3 :
                                   (eff_mag >= THR_MED_ENTER_12 )  ? 2'd2 :
                                   (eff_mag >= THR_LIGHT_ENTER_12) ? 2'd1 : 2'd0;

                2'd1: level_next = (eff_mag >= THR_HARD_ENTER_12)  ? 2'd3 :
                                   (eff_mag >= THR_MED_ENTER_12 )  ? 2'd2 :
                                   (eff_mag <  THR_LIGHT_EXIT_12)  ? 2'd0 : 2'd1;

                2'd2: level_next = (eff_mag >= THR_HARD_ENTER_12)  ? 2'd3 :
                                   (eff_mag <  THR_MED_EXIT_12  )  ?
                                   ((eff_mag >= THR_LIGHT_ENTER_12) ? 2'd1 : 2'd0) : 2'd2;

                default:
                      level_next = (eff_mag < THR_HARD_EXIT_12) ?
                                   ((eff_mag >= THR_MED_ENTER_12)   ? 2'd2 :
                                    (eff_mag >= THR_LIGHT_ENTER_12) ? 2'd1 : 2'd0) : 2'd3;
            endcase
        end
    end

    always_ff @(posedge clk) begin
        if (rst) brake_level <= 2'd0;
        else if (tick_1khz) brake_level <= level_next;
    end

    assign brake_hard = (brake_level == 2'd3);

endmodule
