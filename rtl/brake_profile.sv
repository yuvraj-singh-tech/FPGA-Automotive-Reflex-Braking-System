`timescale 1ns/1ps

// ============================================================================
// brake_profile.sv  (ARBS-B)
// ----------------------------------------------------------------------------
// Aligned inputs from safety_voter:
//   allow_brake_final, emergency_req_final, fault_req_final
//
// What it does:
//   - Deterministic @ tick_1khz
//   - Noise-aware entry/exit confirmation
//   - Minimum emergency hold
//   - Two-stage ramp-up (bite + controlled ramp)
//   - Controlled ramp-down
//   - Fault => immediate fail-silent (brake_cmd=0)
//
// Command format:
//   - brake_cmd is 0..CMD_MAX (default: 0..1000 permille)
// ----------------------------------------------------------------------------
// Coding policy: synthesizable SystemVerilog only (logic/always_ff/always_comb)
// ============================================================================

module brake_profile #(
    // ------------------------------------------------------------
    // Command scaling
    // ------------------------------------------------------------
    parameter int unsigned CMD_MAX           = 1000,  // 0..1000 permille
    parameter int unsigned EMERG_CMD         = 1000,  // target during emergency

    // Two-stage ramp knee (bite threshold)
    parameter int unsigned BITE_THRESH_CMD   = 350,   // ~35% command

    // ------------------------------------------------------------
    // Noise-aware timing @ 1 kHz tick (ms)
    // ------------------------------------------------------------
    parameter int unsigned EMERG_CONFIRM_MS  = 10,    // entry confirmation
    parameter int unsigned CLEAR_CONFIRM_MS  = 150,   // exit confirmation
    parameter int unsigned MIN_HOLD_MS       = 300,   // minimum time in EMERG before allowing release

    // ------------------------------------------------------------
    // Rate limits (delta command per 1 ms tick)
    // ------------------------------------------------------------
    parameter int unsigned RAMP_UP_FAST      = 80,    // to bite threshold
    parameter int unsigned RAMP_UP_SLOW      = 30,    // after bite threshold
    parameter int unsigned RAMP_DN           = 20     // release ramp-down
) (
    input  logic clk,
    input  logic rst,
    input  logic tick_1khz,

    // From safety_voter
    input  logic allow_brake_final,
    input  logic emergency_req_final,
    input  logic fault_req_final,

    // Output command (to safety_arbiter / actuator_if)
    output logic [15:0] brake_cmd,
    output logic        brake_active,

    // Debug (optional but recommended for ILA/waveforms)
    output logic [1:0]  profile_state,
    output logic [15:0] emerg_cnt_dbg,
    output logic [15:0] clear_cnt_dbg,
    output logic [15:0] hold_cnt_dbg
);

    // ------------------------------------------------------------
    // Helpers
    // ------------------------------------------------------------
    function automatic [15:0] u16_min(input [15:0] a, input [15:0] b);
        begin
            u16_min = (a < b) ? a : b;
        end
    endfunction

    function automatic [15:0] u16_max(input [15:0] a, input [15:0] b);
        begin
            u16_max = (a > b) ? a : b;
        end
    endfunction

    function automatic [15:0] sat_add_u16(input [15:0] base, input [15:0] inc, input [15:0] maxv);
        logic [16:0] sum;
        begin
            sum = {1'b0, base} + {1'b0, inc};
            if (sum[16] || (sum[15:0] > maxv))
                sat_add_u16 = maxv;
            else
                sat_add_u16 = sum[15:0];
        end
    endfunction

    function automatic [15:0] sat_sub_u16(input [15:0] base, input [15:0] dec);
        begin
            if (base <= dec) sat_sub_u16 = 16'd0;
            else             sat_sub_u16 = base - dec;
        end
    endfunction

    // Clamp and normalize parameters into u16 domain
    logic [15:0] cmd_max_u16;
    logic [15:0] emerg_cmd_u16;
    logic [15:0] bite_thresh_u16;

    logic [15:0] emerg_confirm_u16;
    logic [15:0] clear_confirm_u16;
    logic [15:0] min_hold_u16;

    logic [15:0] ramp_up_fast_u16;
    logic [15:0] ramp_up_slow_u16;
    logic [15:0] ramp_dn_u16;

    always_comb begin
        cmd_max_u16       = (CMD_MAX  > 65535) ? 16'hFFFF : CMD_MAX[15:0];
        emerg_cmd_u16     = (EMERG_CMD > CMD_MAX) ? cmd_max_u16 : EMERG_CMD[15:0];
        bite_thresh_u16   = (BITE_THRESH_CMD > EMERG_CMD) ? emerg_cmd_u16 : BITE_THRESH_CMD[15:0];

        // clamp confirm/hold to at least 1 ms so "0" never becomes always-true
        emerg_confirm_u16 = (EMERG_CONFIRM_MS < 1) ? 16'd1 :
                            (EMERG_CONFIRM_MS > 65535) ? 16'hFFFF : EMERG_CONFIRM_MS[15:0];

        clear_confirm_u16 = (CLEAR_CONFIRM_MS < 1) ? 16'd1 :
                            (CLEAR_CONFIRM_MS > 65535) ? 16'hFFFF : CLEAR_CONFIRM_MS[15:0];

        min_hold_u16      = (MIN_HOLD_MS < 1) ? 16'd1 :
                            (MIN_HOLD_MS > 65535) ? 16'hFFFF : MIN_HOLD_MS[15:0];

        ramp_up_fast_u16  = (RAMP_UP_FAST < 1) ? 16'd1 :
                            (RAMP_UP_FAST > 65535) ? 16'hFFFF : RAMP_UP_FAST[15:0];

        ramp_up_slow_u16  = (RAMP_UP_SLOW < 1) ? 16'd1 :
                            (RAMP_UP_SLOW > 65535) ? 16'hFFFF : RAMP_UP_SLOW[15:0];

        ramp_dn_u16       = (RAMP_DN < 1) ? 16'd1 :
                            (RAMP_DN > 65535) ? 16'hFFFF : RAMP_DN[15:0];
    end

    // ------------------------------------------------------------
    // State machine
    // ------------------------------------------------------------
    typedef enum logic [1:0] {
        ST_IDLE    = 2'd0,
        ST_EMERG   = 2'd1,
        ST_RELEASE = 2'd2,
        ST_FAULT   = 2'd3
    } st_t;

    st_t st;

    // Counters (saturating)
    logic [15:0] emerg_cnt;
    logic [15:0] clear_cnt;
    logic [15:0] hold_cnt;

    // Derived gating
    logic req_eff;

    always_comb begin
        // Effective emergency request is only meaningful when allowed and no fault
        req_eff = emergency_req_final && allow_brake_final && !fault_req_final;
    end

    // Outputs
    always_comb begin
        brake_active   = (brake_cmd != 16'd0);
        profile_state  = st;
        emerg_cnt_dbg  = emerg_cnt;
        clear_cnt_dbg  = clear_cnt;
        hold_cnt_dbg   = hold_cnt;
    end

    // ------------------------------------------------------------
    // Main deterministic ticked logic
    // ------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            st        <= ST_IDLE;
            brake_cmd <= 16'd0;

            emerg_cnt <= 16'd0;
            clear_cnt <= 16'd0;
            hold_cnt  <= 16'd0;

        end else if (tick_1khz) begin

            // --------------------------------------------------------
            // Fault: immediate fail-silent
            // --------------------------------------------------------
            if (fault_req_final) begin
                st        <= ST_FAULT;
                brake_cmd <= 16'd0;

                emerg_cnt <= 16'd0;
                clear_cnt <= 16'd0;
                hold_cnt  <= 16'd0;

            end else begin

                // --------------------------------------------------------
                // Gate closed: comfort-safe ramp down to 0, reset logic
                // --------------------------------------------------------
                if (!allow_brake_final) begin
                    st        <= ST_IDLE;
                    emerg_cnt <= 16'd0;
                    clear_cnt <= 16'd0;
                    hold_cnt  <= 16'd0;

                    brake_cmd <= sat_sub_u16(brake_cmd, ramp_dn_u16);

                end else begin

                    unique case (st)

                        // ====================================================
                        ST_IDLE: begin
                            brake_cmd <= sat_sub_u16(brake_cmd, ramp_dn_u16);

                            hold_cnt  <= 16'd0;
                            clear_cnt <= 16'd0;

                            if (req_eff) begin
                                // count consecutive request
                                if (emerg_cnt != 16'hFFFF)
                                    emerg_cnt <= emerg_cnt + 16'd1;

                                // enter emergency after confirm (include this tick)
                                if ((emerg_cnt + 16'd1) >= emerg_confirm_u16) begin
                                    st       <= ST_EMERG;
                                    hold_cnt <= 16'd0;
                                end
                            end else begin
                                emerg_cnt <= 16'd0;
                            end
                        end

                        // ====================================================
                        ST_EMERG: begin
                            emerg_cnt <= 16'd0;

                            // min-hold timer
                            if (hold_cnt != 16'hFFFF)
                                hold_cnt <= hold_cnt + 16'd1;

                            // two-stage ramp up
                            if (brake_cmd < emerg_cmd_u16) begin
                                if (brake_cmd < bite_thresh_u16)
                                    brake_cmd <= sat_add_u16(brake_cmd, ramp_up_fast_u16, emerg_cmd_u16);
                                else
                                    brake_cmd <= sat_add_u16(brake_cmd, ramp_up_slow_u16, emerg_cmd_u16);
                            end else begin
                                brake_cmd <= emerg_cmd_u16;
                            end

                            // clear confirmation logic (noise-aware)
                            if (!req_eff) begin
                                if (clear_cnt != 16'hFFFF)
                                    clear_cnt <= clear_cnt + 16'd1;
                            end else begin
                                clear_cnt <= 16'd0;
                            end

                            // allow leaving EMERG only when:
                            //  - hold met
                            //  - stable clear met
                            if ((hold_cnt >= min_hold_u16) &&
                                (!req_eff) &&
                                ((clear_cnt + 16'd1) >= clear_confirm_u16)) begin
                                st        <= ST_RELEASE;
                                clear_cnt <= 16'd0;
                                hold_cnt  <= 16'd0;
                            end
                        end

                        // ====================================================
                        ST_RELEASE: begin
                            // If emergency re-asserts, snap back to EMERG immediately
                            if (req_eff) begin
                                st        <= ST_EMERG;
                                hold_cnt  <= 16'd0;
                                clear_cnt <= 16'd0;
                            end

                            brake_cmd <= sat_sub_u16(brake_cmd, ramp_dn_u16);

                            // once at 0, go idle (clean)
                            if (brake_cmd <= ramp_dn_u16) begin
                                st        <= ST_IDLE;
                                emerg_cnt <= 16'd0;
                                clear_cnt <= 16'd0;
                                hold_cnt  <= 16'd0;
                            end
                        end

                        // ====================================================
                        ST_FAULT: begin
                            // Fault cleared (we wouldn't be here if still fault_req_final),
                            // keep safe and return to IDLE.
                            brake_cmd <= 16'd0;
                            st        <= ST_IDLE;

                            emerg_cnt <= 16'd0;
                            clear_cnt <= 16'd0;
                            hold_cnt  <= 16'd0;
                        end

                        default: begin
                            st        <= ST_IDLE;
                            brake_cmd <= 16'd0;

                            emerg_cnt <= 16'd0;
                            clear_cnt <= 16'd0;
                            hold_cnt  <= 16'd0;
                        end
                    endcase

                end
            end
        end
    end

endmodule