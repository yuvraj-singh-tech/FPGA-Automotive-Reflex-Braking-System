`timescale 1ns/1ps

// =============================================================================
// Module      : safety_voter
// Project     : FPGA Automotive Reflex Braking System (ARBS)
// Author      : Yuvraj Singh
// -----------------------------------------------------------------------------
// Description :
//   Votes the outputs of two independent safety supervisor channels.
//
//   The module compares supervisor channel A and channel B decisions, combines
//   their emergency and fault requests, and detects persistent disagreement
//   between the channels as a voter mismatch fault.
//
// Key Functions:
//   - Combines allow_brake using conservative AND voting
//   - Combines emergency requests using OR voting
//   - Combines supervisor faults and voter mismatch into final fault request
//   - Detects persistent disagreement between supervisor channels
//   - Supports sticky or self-clearing mismatch fault behavior
//   - Optionally suppresses emergency request when a final fault is active
//   - Provides mismatch counter and live mismatch status for debug visibility
//
// Design Notes:
//   This block improves decision robustness by comparing two supervisor channels
//   before sending final safety requests to the braking profile and arbiter.
// =============================================================================

module safety_voter #(
    // ------------------------------------------------------------
    // Automotive-style mismatch policy @ 1 kHz tick
    // ------------------------------------------------------------
    parameter int unsigned MISMATCH_CONFIRM_MS     = 20,   // consecutive mismatch required to declare voter fault
    parameter bit          STICKY_MISMATCH         = 1'b1, // latch mismatch_fault until reset
    parameter bit          FAULT_SUPPRESSES_EMERG  = 1'b0  // if 1: when fault_final=1, emergency_final is forced 0 (fail-silent ARBS)
) (
    input  logic clk,
    input  logic rst,
    input  logic tick_1khz,

    // Channel A (from safety_supervisor_A)
    input  logic allow_brake_A,
    input  logic emergency_req_A,
    input  logic fault_req_A,

    // Channel B (from safety_supervisor_B)
    input  logic allow_brake_B,
    input  logic emergency_req_B,
    input  logic fault_req_B,

    // Voted outputs (to downstream arbiter / profile)
    output logic allow_brake_final,
    output logic emergency_req_final,
    output logic fault_req_final,

    // Diagnostics
    output logic mismatch_fault,
    output logic [7:0] mismatch_cnt,
    output logic mismatch_now
);

    // ------------------------------------------------------------
    // Derived: channel disagreement (split-brain)
    // Compare only safety intent outputs (emerg/fault).
    // ------------------------------------------------------------
    always_comb begin
        mismatch_now =
            (emergency_req_A != emergency_req_B) ||
            (fault_req_A     != fault_req_B);
    end

    // ------------------------------------------------------------
    // Confirm threshold (clamped)
    // - clamp to [1..255] so 0 never becomes "always true"
    // ------------------------------------------------------------
    logic [7:0] confirm_ms_8;
    always_comb begin
        if (MISMATCH_CONFIRM_MS < 1)
            confirm_ms_8 = 8'd1;
        else if (MISMATCH_CONFIRM_MS > 255)
            confirm_ms_8 = 8'd255;
        else
            confirm_ms_8 = MISMATCH_CONFIRM_MS[7:0];
    end

    // ------------------------------------------------------------
    // Voted outputs (fail-safe philosophy)
    // - allow_brake: AND is conservative (if any channel ever gates),
    //   but in your supervisors it's always 1 (driver-first invariant).
    // - emergency: OR (either channel can request emergency)
    // - fault: OR + mismatch_fault
    // Optional policy: FAULT_SUPPRESSES_EMERG (fail-silent ARBS under fault)
    // ------------------------------------------------------------
    logic emerg_or;
    logic fault_or;
    logic allow_and;

    always_comb begin
        allow_and = allow_brake_A & allow_brake_B;
        emerg_or  = emergency_req_A | emergency_req_B;
        fault_or  = fault_req_A     | fault_req_B     | mismatch_fault;

        allow_brake_final   = allow_and;
        fault_req_final     = fault_or;

        if (FAULT_SUPPRESSES_EMERG && fault_or)
            emergency_req_final = 1'b0;
        else
            emergency_req_final = emerg_or;
    end

    // ------------------------------------------------------------
    // Mismatch persistence counter (noise-aware)
    // - Counts consecutive mismatch ticks
    // - Declares mismatch_fault after confirm_ms_8 consecutive mismatches
    // - If STICKY_MISMATCH=1: mismatch_fault latches until reset
    // - Once sticky mismatch_fault is set, counter freezes (clean diagnostics)
    // ------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            mismatch_cnt   <= 8'd0;
            mismatch_fault <= 1'b0;

        end else if (tick_1khz) begin

            // Sticky mode: once fault is set, freeze counter and hold fault
            if (STICKY_MISMATCH && mismatch_fault) begin
                mismatch_cnt   <= mismatch_cnt;
                mismatch_fault <= 1'b1;

            end else begin
                if (mismatch_now) begin
                    // count consecutive mismatch
                    if (mismatch_cnt != 8'hFF)
                        mismatch_cnt <= mismatch_cnt + 8'd1;

                    // declare fault once confirmed (use +1 to include this tick)
                    if ((mismatch_cnt + 8'd1) >= confirm_ms_8)
                        mismatch_fault <= 1'b1;

                end else begin
                    // agreement: clear counter and (if non-sticky) clear fault
                    mismatch_cnt <= 8'd0;
                    if (!STICKY_MISMATCH)
                        mismatch_fault <= 1'b0;
                end
            end
        end
    end

endmodule
