`timescale 1ns/1ps

// =============================================================================
// Module      : driver_intent_if
// Project     : FPGA Automotive Reflex Braking System (ARBS)
// Author      : Yuvraj Singh
// -----------------------------------------------------------------------------
// Description :
//   Converts validated brake-input signals into stable driver intent for the
//   ARBS safety pipeline.
//
//   The module confirms driver brake levels, rejects short glitches, holds valid
//   intent across brief input dropouts, detects panic braking, and flags
//   implausible intent behavior such as excessive accepted toggles.
//
// Key Functions:
//   - Confirms LIGHT, MEDIUM, HARD, and RELEASE intent with separate timing windows
//   - Holds last stable driver intent during short validity interruptions
//   - Applies minimum dwell time after each accepted intent transition
//   - Generates a release_ok pulse after confirmed brake release
//   - Detects confirmed panic braking from stable HARD input
//   - Monitors excessive accepted intent toggling as a sticky intent fault
//   - Optionally maps upstream ADC faults into intent_fault
//
// Design Notes:
//   This block does not generate brake commands directly. It produces stable
//   driver-intent signals for supervisors, arbitration, and downstream safety
//   decision logic.
// =============================================================================

module driver_intent_if #(
    // ------------------------------------------------------------
    // Confirmation windows (1 kHz ticks -> ms)
    // Practical tuned defaults (ECU-like):
    //  - LIGHT confirms slower (most chattery region)
    //  - HARD confirms fast (panic press)
    //  - Release confirms longer (spring/foot lift jitter)
    // ------------------------------------------------------------
    parameter int unsigned CONF_LIGHT_MS    = 30,
    parameter int unsigned CONF_MED_MS      = 20,
    parameter int unsigned CONF_HARD_MS     = 10,
    parameter int unsigned CONF_RELEASE_MS  = 40,

    // Hold last stable intent across short validity glitches
    parameter int unsigned VALID_HOLD_MS    = 100,

    // Minimum dwell time once a stable intent is accepted
    parameter int unsigned DWELL_LIGHT_MS   = 25,
    parameter int unsigned DWELL_MED_MS     = 25,
    parameter int unsigned DWELL_HARD_MS    = 50,
    parameter int unsigned DWELL_NONE_MS    = 20,

    // Panic braking detect + hold
    parameter int unsigned PANIC_CONFIRM_MS = 12,
    parameter int unsigned PANIC_HOLD_MS    = 150,

    // Implausible toggling detector (accepted intent changes)
    parameter int unsigned TOGGLE_WIN_MS    = 500,
    parameter int unsigned MAX_TOGGLES      = 8,

    //  if brake_input_if reports sticky ADC faults, may want to mark intent_fault too
    parameter bit          FAULTS_CAUSE_INTENT_FAULT = 1'b0
) (
    input  logic       clk,
    input  logic       rst,
    input  logic       tick_1khz,

    // From brake_input_if
    input  logic       brake_valid,
    input  logic       brake_active,
    input  logic [1:0] brake_level,   // 0 NONE, 1 LIGHT, 2 MED, 3 HARD
    input  logic       brake_hard,
    input  logic       adc_used,
    input  logic       fault_stuck,
    input  logic       fault_spike,
    input  logic       fault_range,

    // Outputs to supervisors/arbiter
    output logic       drv_valid,
    output logic [1:0] drv_level,        // stable driver intent (0/1/2/3)
    output logic       panic_brake,       // confirmed panic press (HARD held)
    output logic       release_ok,        // stable release confirmed
    output logic       intent_fault       // implausible intent behavior (toggle spam / optional faults)
);

    // -------------------------------------------------------------------------
    // Tool-safe explicit width constants (avoid int-vs-logic surprises)
    // NOTE: DO NOT use logic'(X) here (1-bit cast). Use explicit slicing.
    // -------------------------------------------------------------------------
    localparam logic [15:0] CONF_LIGHT_MS_16    = CONF_LIGHT_MS[15:0];
    localparam logic [15:0] CONF_MED_MS_16      = CONF_MED_MS[15:0];
    localparam logic [15:0] CONF_HARD_MS_16     = CONF_HARD_MS[15:0];
    localparam logic [15:0] CONF_RELEASE_MS_16  = CONF_RELEASE_MS[15:0];

    localparam logic [15:0] VALID_HOLD_MS_16    = VALID_HOLD_MS[15:0];

    localparam logic [15:0] DWELL_LIGHT_MS_16   = DWELL_LIGHT_MS[15:0];
    localparam logic [15:0] DWELL_MED_MS_16     = DWELL_MED_MS[15:0];
    localparam logic [15:0] DWELL_HARD_MS_16    = DWELL_HARD_MS[15:0];
    localparam logic [15:0] DWELL_NONE_MS_16    = DWELL_NONE_MS[15:0];

    localparam logic [15:0] PANIC_CONFIRM_MS_16 = PANIC_CONFIRM_MS[15:0];
    localparam logic [15:0] PANIC_HOLD_MS_16    = PANIC_HOLD_MS[15:0];

    localparam logic [15:0] TOGGLE_WIN_MS_16    = TOGGLE_WIN_MS[15:0];
    localparam logic [7:0]  MAX_TOGGLES_8       = MAX_TOGGLES[7:0];

    // ------------------------------------------------------------
    // Derived "candidate intent" this tick
    // IMPORTANT:
    //  - If upstream is invalid but we're still in validity-hold, do NOT
    //    interpret that as driver release. Hold last stable drv_level.
    // ------------------------------------------------------------
    logic [1:0] cand_level;
    logic       cand_valid;

    always_comb begin
        cand_valid = brake_valid;

        if (!brake_valid) begin
            // During validity-hold, keep candidate at last stable level
            cand_level = drv_level;
        end else begin
            if (!brake_active) cand_level = 2'd0;
            else               cand_level = brake_level;
        end
    end

    // ------------------------------------------------------------
    // Validity hold (grace period on brief invalid)
    // ------------------------------------------------------------
    logic        hold_valid;
    logic [15:0] hold_cnt;

    always_ff @(posedge clk) begin
        if (rst) begin
            hold_valid <= 1'b0;
            hold_cnt   <= 16'd0;
        end else if (tick_1khz) begin
            if (cand_valid) begin
                hold_valid <= 1'b1;
                hold_cnt   <= 16'd0;
            end else if (hold_valid) begin
                if (hold_cnt >= VALID_HOLD_MS_16) begin
                    hold_valid <= 1'b0;
                end else begin
                    hold_cnt <= hold_cnt + 16'd1;
                end
            end
        end
    end

    logic eff_valid;
    assign eff_valid = cand_valid || hold_valid;

    // ------------------------------------------------------------
    // Confirmation counters (accept new stable intent only after it persists)
    // ------------------------------------------------------------
    logic [1:0]  cand_level_q;
    logic [15:0] conf_cnt;

    function automatic logic [15:0] req_conf_ms(input logic [1:0] lvl);
        case (lvl)
            2'd1: req_conf_ms = CONF_LIGHT_MS_16;
            2'd2: req_conf_ms = CONF_MED_MS_16;
            2'd3: req_conf_ms = CONF_HARD_MS_16;
            default: req_conf_ms = CONF_RELEASE_MS_16;
        endcase
    endfunction

    // ------------------------------------------------------------
    // Minimum dwell after accepting a stable level (anti-chatter)
    // ------------------------------------------------------------
    logic [15:0] dwell_cnt;
    logic [15:0] dwell_req; // harmless (can keep for debug/future)

    function automatic logic [15:0] req_dwell_ms(input logic [1:0] lvl);
        case (lvl)
            2'd1: req_dwell_ms = DWELL_LIGHT_MS_16;
            2'd2: req_dwell_ms = DWELL_MED_MS_16;
            2'd3: req_dwell_ms = DWELL_HARD_MS_16;
            default: req_dwell_ms = DWELL_NONE_MS_16;
        endcase
    endfunction

    // ------------------------------------------------------------
    // Toggle spam detector over a fixed window
    // Counts accepted stable transitions (drv_level changes).
    // ------------------------------------------------------------
    logic [15:0] toggle_win_cnt;
    logic [7:0]  toggle_cnt;

    // ------------------------------------------------------------
    // Panic detection: HARD confirmed for PANIC_CONFIRM_MS, then held PANIC_HOLD_MS
    // ------------------------------------------------------------
    logic [15:0] panic_cnt;
    logic [15:0] panic_hold_cnt;

    // Optional: map upstream sticky ADC faults into intent_fault (disabled by default)
    logic upstream_fault_any;
    assign upstream_fault_any = (fault_stuck | fault_spike | fault_range);

    // ------------------------------------------------------------
    // Main state: drv_level / drv_valid
    // ------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            drv_level      <= 2'd0;
            drv_valid      <= 1'b0;

            cand_level_q   <= 2'd0;
            conf_cnt       <= 16'd0;

            dwell_cnt      <= 16'd0;
            dwell_req      <= 16'd0;

            toggle_win_cnt <= 16'd0;
            toggle_cnt     <= 8'd0;

            intent_fault   <= 1'b0;

            release_ok     <= 1'b0;

            panic_brake    <= 1'b0;
            panic_cnt      <= 16'd0;
            panic_hold_cnt <= 16'd0;
        end else if (tick_1khz) begin
            // pulse by default
            release_ok <= 1'b0;

            // -----------------------------------------------------------------
            // drv_valid + safe policy on persistent invalid
            // -----------------------------------------------------------------
            if (!eff_valid) begin
                drv_valid <= 1'b0;
                drv_level <= 2'd0;

                // Reset confirm/dwell (safe)
                cand_level_q <= 2'd0;
                conf_cnt     <= 16'd0;
                dwell_cnt    <= 16'd0;
                dwell_req    <= req_dwell_ms(2'd0);

                // Clear panic (conservative)
                panic_brake    <= 1'b0;
                panic_cnt      <= 16'd0;
                panic_hold_cnt <= 16'd0;

                // NOTE: intent_fault is sticky by design (do not clear)
            end else begin
                drv_valid <= 1'b1;

                // ----------------------------
                // Dwell tick-down
                // ----------------------------
                if (dwell_cnt != 16'd0)
                    dwell_cnt <= dwell_cnt - 16'd1;

                // ----------------------------
                // Confirmation tracking
                // ----------------------------
                if (cand_level != cand_level_q) begin
                    cand_level_q <= cand_level;
                    conf_cnt     <= 16'd0;
                end else begin
                    if (conf_cnt != 16'hFFFF)
                        conf_cnt <= conf_cnt + 16'd1;
                end

                // ----------------------------
                // Accept new stable intent
                // ----------------------------
                if ((dwell_cnt == 16'd0) &&
                    (cand_level != drv_level) &&
                    ((conf_cnt + 16'd1) >= req_conf_ms(cand_level))) begin

                    // Toggle bookkeeping (accepted transition)
                    if (toggle_win_cnt == 16'd0) begin
                        toggle_win_cnt <= TOGGLE_WIN_MS_16;
                        toggle_cnt     <= 8'd1;
                    end else begin
                        if (toggle_cnt != 8'hFF)
                            toggle_cnt <= toggle_cnt + 8'd1;
                    end

                    // Accept level
                    drv_level <= cand_level;

                    // Start dwell lock
                    dwell_req <= req_dwell_ms(cand_level);
                    dwell_cnt <= req_dwell_ms(cand_level);

                    // Release pulse
                    if (cand_level == 2'd0)
                        release_ok <= 1'b1;
                end
            end

            // -----------------------------------------------------------------
            // Toggle window countdown + fault
            // -----------------------------------------------------------------
            if (toggle_win_cnt != 16'd0) begin
                toggle_win_cnt <= toggle_win_cnt - 16'd1;

                if (toggle_win_cnt == 16'd1) begin
                    if (toggle_cnt > MAX_TOGGLES_8)
                        intent_fault <= 1'b1;
                    toggle_cnt <= 8'd0;
                end
            end

            // -----------------------------------------------------------------
            // Optional upstream ADC faults -> intent_fault
            // -----------------------------------------------------------------
            if (FAULTS_CAUSE_INTENT_FAULT) begin
                if (upstream_fault_any && adc_used)
                    intent_fault <= 1'b1;
            end

            // -----------------------------------------------------------------
            // Panic braking detection (stable intent HARD)
            // Conservative: require stable HARD + upstream HARD indicator.
            // -----------------------------------------------------------------
            if (drv_valid && (drv_level == 2'd3) && brake_hard) begin
                if (!panic_brake) begin
                    if (panic_cnt < PANIC_CONFIRM_MS_16)
                        panic_cnt <= panic_cnt + 16'd1;

                    if ((panic_cnt + 16'd1) >= PANIC_CONFIRM_MS_16) begin
                        panic_brake    <= 1'b1;
                        panic_hold_cnt <= PANIC_HOLD_MS_16;
                    end
                end else begin
                    // refresh hold
                    panic_hold_cnt <= PANIC_HOLD_MS_16;
                end
            end else begin
                // not meeting panic conditions
                panic_cnt <= 16'd0;

                if (panic_brake) begin
                    if (panic_hold_cnt == 16'd0) begin
                        panic_brake <= 1'b0;
                    end else begin
                        panic_hold_cnt <= panic_hold_cnt - 16'd1;
                    end
                end
            end

        end
    end

endmodule
