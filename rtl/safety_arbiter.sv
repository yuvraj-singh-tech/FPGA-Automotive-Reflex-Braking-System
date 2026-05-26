`timescale 1ns/1ps

// NOTE: Keep synthesizable RTL free of `default_nettype directives.

// =============================================================================
// Module      : safety_arbiter
// Project     : FPGA Automotive Reflex Braking System (ARBS)
// Author      : Yuvraj Singh
// -----------------------------------------------------------------------------
// Description :
//   Selects the final braking command before the actuator output interface.
//
//   The module arbitrates between the driver brake command and the ARBS emergency
//   brake command, while enforcing global safety gating, watchdog protection,
//   command clamping, and fail-silent behavior during faults.
//
// Key Functions:
//   - Gives ARBS emergency braking priority when active
//   - Allows normal driver braking when no emergency override is active
//   - Forces safe zero command during faults or disabled braking conditions
//   - Detects missing 1 kHz timing ticks using a clock-domain watchdog
//   - Applies a short release hold to avoid emergency-command deglitching
//   - Prevents stale driver command leakage when driver_active is low
//   - Exposes arbitration source and watchdog status for debug visibility
//
// Design Notes:
//   This block is the final authority selector before command shaping and PWM
//   generation. It does not generate brake profiles; it selects the safest
//   available command from upstream decision logic.
// =============================================================================

module safety_arbiter #(
    // ------------------------------------------------------------
    // Command scaling (must match your system convention)
    // ------------------------------------------------------------
    parameter int unsigned CMD_MAX              = 1000,      // 0..1000 permille

    // ------------------------------------------------------------
    // Optional small deglitch hold on ARBS release
    // ------------------------------------------------------------
    parameter int unsigned ARBS_RELEASE_HOLD_MS = 5,         // recommended small
    parameter int unsigned ARBS_HOLD_MAX_MS     = 50,        // clamp safety max

    // ------------------------------------------------------------
    // Watchdog: detect missing tick_1khz pulses using clk cycles
    // Default assumes clk = 100 MHz:
    //   WD_TIMEOUT_MS = 250 ms -> 25,000,000 cycles
    // ------------------------------------------------------------
    parameter int unsigned CLK_HZ               = 100_000_000,
    parameter int unsigned WD_TIMEOUT_MS        = 250
) (
    input  logic clk,
    input  logic rst,

    // Timing
    input  logic tick_1khz,

    // ----------------------------
    // Driver command path
    // ----------------------------
    input  logic [15:0] driver_cmd,      // 0..CMD_MAX preferred
    input  logic        driver_active,   // 1 when driver is applying brake

    // ----------------------------
    // ARBS emergency path (from brake_profile)
    // ----------------------------
    input  logic [15:0] arbs_cmd,        // brake_profile.brake_cmd
    input  logic        arbs_active,     // brake_profile.brake_active

    // ----------------------------
    // Fault / gating
    // ----------------------------
    input  logic        fault_any,       // OR of system faults
    input  logic        allow_brake,     // global gate (allow_brake_final)

    // ----------------------------
    // Final output to actuator_if / PWM
    // ----------------------------
    output logic [15:0] brake_cmd_final,

    // Debug / visibility (ILA-friendly)
    output logic [1:0]  arb_src_sel,     // 0=SAFE, 1=DRIVER, 2=ARBS
    output logic        fault_wd         // watchdog fault
);

    // ------------------------------------------------------------
    // Helpers
    // ------------------------------------------------------------
    function automatic [15:0] u16_min(input [15:0] a, input [15:0] b);
        begin
            u16_min = (a < b) ? a : b;
        end
    endfunction

    // Clamp CMD_MAX into u16 domain
    logic [15:0] cmd_max_u16;
    always_comb begin
        cmd_max_u16 = (CMD_MAX > 65535) ? 16'hFFFF : CMD_MAX[15:0];
    end

    // ------------------------------------------------------------
    // Watchdog constant math (guarded)
    // ------------------------------------------------------------
    // cycles_per_ms = CLK_HZ/1000 ; if CLK_HZ < 1000, clamp to 1 for safety
    localparam int unsigned CYCLES_PER_MS =
        (CLK_HZ < 1000) ? 1 : (CLK_HZ / 1000);

    // If WD_TIMEOUT_MS is 0, clamp to 1 ms (prevents underflow)
    localparam int unsigned WD_TIMEOUT_MS_EFF =
        (WD_TIMEOUT_MS < 1) ? 1 : WD_TIMEOUT_MS;

    // Use 64-bit math for safety, then clamp into 32-bit range
    localparam longint unsigned WD_TIMEOUT_CYCLES_64 =
        longint'(CYCLES_PER_MS) * longint'(WD_TIMEOUT_MS_EFF);

    localparam longint unsigned WD_TIMEOUT_CYCLES_64_EFF =
        (WD_TIMEOUT_CYCLES_64 < 1) ? 1 : WD_TIMEOUT_CYCLES_64;

    localparam int unsigned WD_TIMEOUT_CYCLES =
        (WD_TIMEOUT_CYCLES_64_EFF > 32'hFFFF_FFFF) ? 32'hFFFF_FFFF
                                                   : int'(WD_TIMEOUT_CYCLES_64_EFF);

    // ------------------------------------------------------------
    // Tick watchdog in clk domain (works even if tick_1khz stalls)
    // ------------------------------------------------------------
    logic [31:0] wd_cycle_cnt;

    always_ff @(posedge clk) begin
        if (rst) begin
            wd_cycle_cnt <= 32'd0;
            fault_wd     <= 1'b0;
        end else begin
            if (tick_1khz) begin
                wd_cycle_cnt <= 32'd0;
                fault_wd     <= 1'b0;
            end else begin
                if (!fault_wd) begin
                    if (wd_cycle_cnt >= (WD_TIMEOUT_CYCLES - 1)) begin
                        fault_wd     <= 1'b1;
                        wd_cycle_cnt <= wd_cycle_cnt; // hold
                    end else begin
                        wd_cycle_cnt <= wd_cycle_cnt + 32'd1;
                    end
                end
            end
        end
    end

    // ------------------------------------------------------------
    // Optional ARBS release hold (tiny deglitch)
    // ------------------------------------------------------------
    logic [15:0] arbs_hold_cnt;
    logic        arbs_active_held;

    // Clamp hold ms: at least 1, at most ARBS_HOLD_MAX_MS
    logic [15:0] arbs_hold_ms_u16;
    logic [15:0] arbs_hold_max_u16;

    always_comb begin
        arbs_hold_max_u16 = (ARBS_HOLD_MAX_MS < 1) ? 16'd1 :
                            (ARBS_HOLD_MAX_MS > 65535) ? 16'hFFFF : ARBS_HOLD_MAX_MS[15:0];

        // clamp requested hold into [1 .. arbs_hold_max_u16]
        if (ARBS_RELEASE_HOLD_MS < 1)
            arbs_hold_ms_u16 = 16'd1;
        else if (ARBS_RELEASE_HOLD_MS > ARBS_HOLD_MAX_MS)
            arbs_hold_ms_u16 = arbs_hold_max_u16;
        else
            arbs_hold_ms_u16 = (ARBS_RELEASE_HOLD_MS > 65535) ? 16'hFFFF : ARBS_RELEASE_HOLD_MS[15:0];
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            arbs_hold_cnt    <= 16'd0;
            arbs_active_held <= 1'b0;
        end else if (tick_1khz) begin
            if (arbs_active) begin
                arbs_active_held <= 1'b1;
                arbs_hold_cnt    <= 16'd0;
            end else begin
                if (arbs_active_held) begin
                    if ((arbs_hold_cnt + 16'd1) >= arbs_hold_ms_u16) begin
                        arbs_active_held <= 1'b0;
                        arbs_hold_cnt    <= 16'd0;
                    end else begin
                        arbs_hold_cnt <= arbs_hold_cnt + 16'd1;
                    end
                end else begin
                    arbs_hold_cnt <= 16'd0;
                end
            end
        end
    end

    // ------------------------------------------------------------
    // Command hygiene (driver_active enforcement + sim-safe X guards)
    // ------------------------------------------------------------
    logic [15:0] driver_cmd_eff;
    logic [15:0] arbs_cmd_eff;

    always_comb begin
        // Driver hygiene: if not active, treat as 0 (prevents stale nonzero)
        driver_cmd_eff = (driver_active) ? driver_cmd : 16'd0;

        // Sim-safe guard: if cmd contains X, force to 0
        // (In real hardware this won't happen; helps TB/waveforms.)
        if (^driver_cmd_eff === 1'bX) driver_cmd_eff = 16'd0;
        if (^arbs_cmd       === 1'bX) arbs_cmd_eff   = 16'd0;
        else                          arbs_cmd_eff   = arbs_cmd;
    end

    // ------------------------------------------------------------
    // Final authority decision
    // ------------------------------------------------------------
    logic sys_fault;
    logic use_arbs;

    always_comb begin
        // Fail-silent policy:
        // - any fault OR watchdog OR allow_brake low => SAFE output
        sys_fault = fault_any || fault_wd || (!allow_brake);

        // ARBS owns command when emergency is active (or held briefly)
        use_arbs  = arbs_active || arbs_active_held;

        // Defaults
        brake_cmd_final = 16'd0;
        arb_src_sel     = 2'd0; // SAFE

        if (sys_fault) begin
            brake_cmd_final = 16'd0;
            arb_src_sel     = 2'd0;
        end else if (use_arbs) begin
            brake_cmd_final = u16_min(arbs_cmd_eff,   cmd_max_u16);
            arb_src_sel     = 2'd2;
        end else begin
            brake_cmd_final = u16_min(driver_cmd_eff, cmd_max_u16);
            arb_src_sel     = 2'd1;
        end
    end

endmodule
