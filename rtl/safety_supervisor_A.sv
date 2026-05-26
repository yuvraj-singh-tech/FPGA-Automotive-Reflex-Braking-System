`timescale 1ns/1ps

module safety_supervisor_A #(
    // ------------------------------------------------------------
    // Emergency decision tuning @ 1 kHz tick (ms)
    // ------------------------------------------------------------
    parameter int unsigned EMERG_HARD_CONFIRM_MS    = 20,   // HARD must persist this long (if no panic_brake)
    parameter int unsigned EMERG_MIN_HOLD_MS        = 150,  // once emergency activates, hold at least this long
    parameter int unsigned EMERG_RELEASE_CONFIRM_MS = 60,   // require stable "released/low" this long to clear

    // Invalid intent policy
    parameter int unsigned INVALID_HOLD_MS          = 50,   // tolerate brief drv_valid drops
    parameter int unsigned INVALID_FAULT_MS         = 300,  // persistent invalid => fault_req sticky

    // Policy knobs
    parameter bit          PANIC_ALWAYS_EMERG       = 1'b1, // panic_brake => emergency immediately
    parameter bit          INTENT_FAULT_IS_FATAL    = 1'b1  // intent_fault => fault_req sticky
) (
    input  logic       clk,
    input  logic       rst,
    input  logic       tick_1khz,

    // From driver_intent_if (aligned exactly)
    input  logic       drv_valid,
    input  logic [1:0] drv_level,     // 0 NONE, 1 LIGHT, 2 MED, 3 HARD
    input  logic       panic_brake,   // confirmed + held
    input  logic       release_ok,    // 1-cycle pulse on confirmed release to NONE
    input  logic       intent_fault,  // treat as fatal (optional)

    // Outputs to voter/arbiter
    output logic       allow_brake,
    output logic       emergency_req,
    output logic       fault_req,

    // Optional debug
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
    // Explicit width constants
    // ------------------------------------------------------------
    localparam logic [15:0] EMERG_HARD_CONFIRM_MS_16    = EMERG_HARD_CONFIRM_MS[15:0];
    localparam logic [15:0] EMERG_MIN_HOLD_MS_16        = EMERG_MIN_HOLD_MS[15:0];
    localparam logic [15:0] EMERG_RELEASE_CONFIRM_MS_16 = EMERG_RELEASE_CONFIRM_MS[15:0];

    localparam logic [15:0] INVALID_HOLD_MS_16          = INVALID_HOLD_MS[15:0];
    localparam logic [15:0] INVALID_FAULT_MS_16         = INVALID_FAULT_MS[15:0];

    // ------------------------------------------------------------
    // Counters (ms @ tick_1khz)
    // ------------------------------------------------------------
    logic [15:0] hard_cnt;
    logic [15:0] emerg_hold_cnt;
    logic [15:0] release_cnt;

    logic [15:0] invalid_cnt;   // consecutive invalid ticks
    logic [15:0] invalid_hold;  // brief-invalid tracker (paranoia gating)

    // ------------------------------------------------------------
    // Simple derived conditions (declare outside procedural blocks)
    // ------------------------------------------------------------
    logic hard_now;
    always_comb begin
        hard_now = (drv_valid && (drv_level == 2'd3));
    end

    // ------------------------------------------------------------
    // allow_brake policy (driver-first)
    // In FAULT, we keep allow_brake=1 so driver braking path is allowed;
    // downstream arbiter will block ARBS intervention.
    // ------------------------------------------------------------
    always_comb begin
        allow_brake = 1'b1;
    end

    // ------------------------------------------------------------
    // Main supervisor logic
    // ------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            supv_state     <= S_OK;
            emergency_req  <= 1'b0;
            fault_req      <= 1'b0;

            hard_cnt       <= 16'd0;
            emerg_hold_cnt <= 16'd0;
            release_cnt    <= 16'd0;

            invalid_cnt    <= 16'd0;
            invalid_hold   <= 16'd0;

        end else if (tick_1khz) begin

            // --------------------------------------------------------
            // Compute "fatal now" using current + this-tick evidence
            // so FAULT behavior takes effect IMMEDIATELY (no 1-tick lag)
            // --------------------------------------------------------
            logic [15:0] invalid_cnt_next;
            logic        fatal_now;

            // next invalid counter (consecutive invalid)
            if (!drv_valid) begin
                if (invalid_cnt != 16'hFFFF) invalid_cnt_next = invalid_cnt + 16'd1;
                else                          invalid_cnt_next = invalid_cnt;
            end else begin
                invalid_cnt_next = 16'd0;
            end

            fatal_now =
                fault_req || // already latched
                (INTENT_FAULT_IS_FATAL && intent_fault) ||
                (!drv_valid && (invalid_cnt_next >= INVALID_FAULT_MS_16));

            // --------------------------------------------------------
            // Update sticky fault latch
            // --------------------------------------------------------
            if (INTENT_FAULT_IS_FATAL && intent_fault)
                fault_req <= 1'b1;

            if (!drv_valid && (invalid_cnt_next >= INVALID_FAULT_MS_16))
                fault_req <= 1'b1;

            // --------------------------------------------------------
            // Invalid tracking (hold vs persistent)
            // --------------------------------------------------------
            if (!drv_valid) begin
                invalid_cnt <= invalid_cnt_next;

                if (invalid_hold < INVALID_HOLD_MS_16)
                    invalid_hold <= invalid_hold + 16'd1;
            end else begin
                invalid_cnt  <= 16'd0;
                invalid_hold <= 16'd0;
            end

            // --------------------------------------------------------
            // If fatal now: force FAULT behavior immediately
            // --------------------------------------------------------
            if (fatal_now) begin
                supv_state    <= S_FAULT;
                emergency_req <= 1'b0;

                hard_cnt       <= 16'd0;
                emerg_hold_cnt <= 16'd0;
                release_cnt    <= 16'd0;

            end else begin
                // ----------------------------------------------------
                // Track sustained HARD (only when valid)
                // ----------------------------------------------------
                if (hard_now) begin
                    if (hard_cnt != 16'hFFFF)
                        hard_cnt <= hard_cnt + 16'd1;
                end else begin
                    hard_cnt <= 16'd0;
                end

                // ----------------------------------------------------
                // Supervisor FSM
                // ----------------------------------------------------
                case (supv_state)

                    S_OK: begin
                        emergency_req  <= 1'b0;
                        release_cnt    <= 16'd0;
                        emerg_hold_cnt <= 16'd0;

                        // Emergency entry conditions:
                        // 1) panic_brake (already confirmed+held upstream)
                        // 2) sustained HARD confirm
                        if (PANIC_ALWAYS_EMERG && panic_brake) begin
                            emergency_req  <= 1'b1;
                            supv_state     <= S_EMERG;
                            emerg_hold_cnt <= EMERG_MIN_HOLD_MS_16;
                        end else if (hard_now && ((hard_cnt + 16'd1) >= EMERG_HARD_CONFIRM_MS_16)) begin
                            // +1 so confirm aligns to the tick we just observed HARD
                            emergency_req  <= 1'b1;
                            supv_state     <= S_EMERG;
                            emerg_hold_cnt <= EMERG_MIN_HOLD_MS_16;
                        end
                    end

                    S_EMERG: begin
                        emergency_req <= 1'b1;

                        // Minimum hold timer
                        if (emerg_hold_cnt != 16'd0)
                            emerg_hold_cnt <= emerg_hold_cnt - 16'd1;

                        // Evaluate release only after min-hold elapsed
                        if (emerg_hold_cnt == 16'd0) begin
                            // Require stable low intent for EMERG_RELEASE_CONFIRM_MS
                            // We treat NONE or LIGHT as "released enough" for clearing emergency
                            if (drv_valid && (drv_level <= 2'd1)) begin
                                if (release_cnt != 16'hFFFF)
                                    release_cnt <= release_cnt + 16'd1;
                            end else begin
                                release_cnt <= 16'd0;
                            end

                            // release_ok pulse gives a small boost but still requires persistence
                            if (release_ok) begin
                                if (release_cnt < 16'd10)
                                    release_cnt <= 16'd10;
                            end

                            if (release_cnt >= EMERG_RELEASE_CONFIRM_MS_16) begin
                                emergency_req <= 1'b0;
                                supv_state    <= S_RELEASE;

                                // clean counters for exit
                                release_cnt   <= 16'd0;
                                hard_cnt      <= 16'd0;
                            end
                        end
                    end

                    S_RELEASE: begin
                        // brief cooldown-ish state to avoid re-trigger jitter
                        emergency_req <= 1'b0;

                        // If panic returns, re-enter immediately
                        if (PANIC_ALWAYS_EMERG && panic_brake) begin
                            emergency_req  <= 1'b1;
                            supv_state     <= S_EMERG;
                            emerg_hold_cnt <= EMERG_MIN_HOLD_MS_16;
                        end else begin
                            // Return to OK when not HARD (or when valid is briefly low)
                            if (drv_valid) begin
                                if (drv_level != 2'd3)
                                    supv_state <= S_OK;
                                else
                                    supv_state <= S_OK; // require fresh confirm for re-entry
                            end
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