`timescale 1ns/1ps
// NOTE (ARBSB RTL policy): no `default_nettype in synthesizable RTL.

// ============================================================================
// top_arbsb.sv  (ARBS-B)  - Paranoid, deterministic integration (Automotive style)
// ----------------------------------------------------------------------------
// MUST-FIXES applied in this top:
//   1) safety_arbiter watchdog clock alignment: CLK_HZ set to 33_333_333
//   2) Explicit param alignment for CMD_MAX + hold values (no hidden defaults)
//
// NOTE: You still MUST fix your brake_profile file's last line:
//   replace "endmodul" -> "endmodule"  (compile blocker)
// ============================================================================

module top_arbsb #(
    // Global scaling (consistent across modules)
    parameter int unsigned CMD_MAX = 1000,

    // Driver command mapping (permille) from drv_level
    parameter int unsigned DRIVER_CMD_LIGHT = 300,
    parameter int unsigned DRIVER_CMD_MED   = 650,
    parameter int unsigned DRIVER_CMD_HARD  = 1000
) (
    // Clock + raw reset
    input  logic        clk_33m,          // 33.333 MHz
    input  logic        rst_raw,           // async, active-high (button/POR)

    // Brake inputs
    input  logic        brake_raw,         // digital brake switch
    input  logic [11:0] brake_adc,         // ADC magnitude (0..4095)
    input  logic        brake_adc_valid,   // ADC validity flag

    // PWM output
    output logic        pwm_out,

    // Debug taps (optional)
    output logic        rst_sync_dbg,
    output logic        tick_1khz_dbg,
    output logic        tick_50hz_dbg,

    output logic [1:0]  drv_level_dbg,
    output logic        drv_valid_dbg,
    output logic        panic_brake_dbg,
    output logic        intent_fault_dbg,

    output logic        emergency_req_final_dbg,
    output logic        fault_req_final_dbg,
    output logic        mismatch_fault_dbg,

    output logic [1:0]  arb_src_sel_dbg,
    output logic        fault_wd_dbg,

    output logic        out_enable_dbg,
    output logic        out_fault_dbg,
    output logic [15:0] cmd_out_dbg,
    output logic        armed_dbg
);

    // =========================================================================
    // 0) reset_sync (async assert, sync deassert)
    // =========================================================================
    logic rst_sync;

    reset_sync u_reset_sync (
        .clk      (clk_33m),
        .rst_raw  (rst_raw),
        .rst_sync (rst_sync)
    );

    assign rst_sync_dbg = rst_sync;

    // =========================================================================
    // 1) tick_gen (1kHz + 50Hz)
    // =========================================================================
    logic tick_1khz, tick_50hz;

    tick_gen u_tick_gen (
        .clk       (clk_33m),
        .rst       (rst_sync),
        .tick_1khz (tick_1khz),
        .tick_50hz (tick_50hz)
    );

    assign tick_1khz_dbg = tick_1khz;
    assign tick_50hz_dbg = tick_50hz;

    // =========================================================================
    // 2) brake_input_if (conditioning)
    // =========================================================================
    logic        brake_valid;
    logic        brake_active;
    logic [1:0]  brake_level;
    logic        brake_hard;
    logic        adc_used;
    logic        fault_stuck, fault_spike, fault_range;

    brake_input_if #(
        .ADC_MAX (4095)  // keep explicit for clarity
    ) u_brake_input_if (
        .clk             (clk_33m),
        .rst             (rst_sync),
        .tick_1khz        (tick_1khz),

        .brake_raw       (brake_raw),
        .brake_adc       (brake_adc),
        .brake_adc_valid (brake_adc_valid),

        .brake_valid     (brake_valid),
        .brake_active    (brake_active),
        .brake_level     (brake_level),
        .brake_hard      (brake_hard),

        .adc_used        (adc_used),
        .fault_stuck     (fault_stuck),
        .fault_spike     (fault_spike),
        .fault_range     (fault_range)
    );

    // =========================================================================
    // 3) driver_intent_if (stable intent)
    // =========================================================================
    logic        drv_valid;
    logic [1:0]  drv_level;
    logic        panic_brake;
    logic        release_ok;
    logic        intent_fault;

    driver_intent_if u_driver_intent_if (
        .clk          (clk_33m),
        .rst          (rst_sync),
        .tick_1khz     (tick_1khz),

        .brake_valid  (brake_valid),
        .brake_active (brake_active),
        .brake_level  (brake_level),
        .brake_hard   (brake_hard),
        .adc_used     (adc_used),
        .fault_stuck  (fault_stuck),
        .fault_spike  (fault_spike),
        .fault_range  (fault_range),

        .drv_valid    (drv_valid),
        .drv_level    (drv_level),
        .panic_brake  (panic_brake),
        .release_ok   (release_ok),
        .intent_fault (intent_fault)
    );

    assign drv_valid_dbg    = drv_valid;
    assign drv_level_dbg    = drv_level;
    assign panic_brake_dbg  = panic_brake;
    assign intent_fault_dbg = intent_fault;

    // =========================================================================
    // 4) safety supervisors A/B
    // =========================================================================
    logic allow_brake_A, emergency_req_A, fault_req_A;
    logic allow_brake_B, emergency_req_B, fault_req_B;
    logic [2:0] supv_state_A, supv_state_B;

    safety_supervisor_A u_supv_A (
        .clk          (clk_33m),
        .rst          (rst_sync),
        .tick_1khz     (tick_1khz),

        .drv_valid    (drv_valid),
        .drv_level    (drv_level),
        .panic_brake  (panic_brake),
        .release_ok   (release_ok),
        .intent_fault (intent_fault),

        .allow_brake  (allow_brake_A),
        .emergency_req(emergency_req_A),
        .fault_req    (fault_req_A),
        .supv_state   (supv_state_A)
    );

    safety_supervisor_B u_supv_B (
        .clk          (clk_33m),
        .rst          (rst_sync),
        .tick_1khz     (tick_1khz),

        .drv_valid    (drv_valid),
        .drv_level    (drv_level),
        .panic_brake  (panic_brake),
        .release_ok   (release_ok),
        .intent_fault (intent_fault),

        .allow_brake  (allow_brake_B),
        .emergency_req(emergency_req_B),
        .fault_req    (fault_req_B),
        .supv_state   (supv_state_B)
    );

    // =========================================================================
    // 5) safety_voter (A/B vote + mismatch)
    // =========================================================================
    logic allow_brake_final;
    logic emergency_req_final;
    logic fault_req_final;

    logic mismatch_fault;
    logic [7:0] mismatch_cnt;
    logic mismatch_now;

    safety_voter u_safety_voter (
        .clk                (clk_33m),
        .rst                (rst_sync),
        .tick_1khz            (tick_1khz),

        .allow_brake_A      (allow_brake_A),
        .emergency_req_A    (emergency_req_A),
        .fault_req_A        (fault_req_A),

        .allow_brake_B      (allow_brake_B),
        .emergency_req_B    (emergency_req_B),
        .fault_req_B        (fault_req_B),

        .allow_brake_final  (allow_brake_final),
        .emergency_req_final(emergency_req_final),
        .fault_req_final    (fault_req_final),

        .mismatch_fault     (mismatch_fault),
        .mismatch_cnt       (mismatch_cnt),
        .mismatch_now       (mismatch_now)
    );

    assign emergency_req_final_dbg = emergency_req_final;
    assign fault_req_final_dbg     = fault_req_final;
    assign mismatch_fault_dbg      = mismatch_fault;

    // =========================================================================
    // 6) brake_profile (emergency shaping)
    // =========================================================================
    logic [15:0] arbs_cmd;
    logic        arbs_active;
    logic [1:0]  profile_state;
    logic [15:0] emerg_cnt_dbg, clear_cnt_dbg, hold_cnt_dbg;

    brake_profile #(
        .CMD_MAX (CMD_MAX),
        .EMERG_CMD(CMD_MAX)
    ) u_brake_profile (
        .clk                 (clk_33m),
        .rst                 (rst_sync),
        .tick_1khz            (tick_1khz),

        .allow_brake_final   (allow_brake_final),
        .emergency_req_final (emergency_req_final),
        .fault_req_final     (fault_req_final),

        .brake_cmd           (arbs_cmd),
        .brake_active        (arbs_active),

        .profile_state       (profile_state),
        .emerg_cnt_dbg       (emerg_cnt_dbg),
        .clear_cnt_dbg       (clear_cnt_dbg),
        .hold_cnt_dbg        (hold_cnt_dbg)
    );

    // =========================================================================
    // 7) Driver command mapping for arbiter (self-contained top)
    // =========================================================================
    logic [15:0] driver_cmd;
    logic        driver_active;

    function automatic [15:0] clamp_u16_max(input [15:0] x, input [15:0] maxv);
        if (x > maxv) clamp_u16_max = maxv;
        else          clamp_u16_max = x;
    endfunction

    logic [15:0] cmd_max_u16;

    always_comb begin
        cmd_max_u16 = (CMD_MAX > 65535) ? 16'hFFFF : CMD_MAX[15:0];

        // Clean layering: drv_level already stable; require drv_valid.
        driver_active = (drv_valid && (drv_level != 2'd0));

        driver_cmd = 16'd0;
        if (driver_active) begin
            unique case (drv_level)
                2'd1: driver_cmd = DRIVER_CMD_LIGHT[15:0];
                2'd2: driver_cmd = DRIVER_CMD_MED[15:0];
                default: driver_cmd = DRIVER_CMD_HARD[15:0]; // 2'd3
            endcase
            driver_cmd = clamp_u16_max(driver_cmd, cmd_max_u16);
        end
    end

    // =========================================================================
    // 8) safety_arbiter (authority + watchdog)  **CLK_HZ FIX APPLIED**
    // =========================================================================
    logic [15:0] brake_cmd_final;
    logic [1:0]  arb_src_sel;
    logic        fault_wd;

    safety_arbiter #(
        .CMD_MAX              (CMD_MAX),
        .ARBS_RELEASE_HOLD_MS (5),
        .ARBS_HOLD_MAX_MS     (50),
        .CLK_HZ               (33_333_333),  // IMPORTANT: match clk_33m
        .WD_TIMEOUT_MS        (250)
    ) u_safety_arbiter (
        .clk            (clk_33m),
        .rst            (rst_sync),
        .tick_1khz        (tick_1khz),

        .driver_cmd     (driver_cmd),
        .driver_active  (driver_active),

        .arbs_cmd       (arbs_cmd),
        .arbs_active    (arbs_active),

        .fault_any      (fault_req_final),      // already includes mismatch_fault
        .allow_brake    (allow_brake_final),

        .brake_cmd_final(brake_cmd_final),
        .arb_src_sel    (arb_src_sel),
        .fault_wd       (fault_wd)
    );

    assign arb_src_sel_dbg = arb_src_sel;
    assign fault_wd_dbg    = fault_wd;

    // =========================================================================
    // 9) brake_output_if (slew + enable + alive + crc + hard cut)
    // =========================================================================
    logic [15:0] cmd_out;
    logic        out_enable;
    logic        out_fault;
    logic        stale_cmd;
    logic        saturated;
    logic [7:0]  alive_cnt;
    logic [7:0]  crc8;
    logic [15:0] target_cmd_dbg;
    logic        enable_req_dbg;
    logic        x_guard_trip_dbg;

    brake_output_if #(
        .CMD_MAX(CMD_MAX)
    ) u_brake_output_if (
        .clk            (clk_33m),
        .rst            (rst_sync),
        .tick_1khz        (tick_1khz),

        .brake_cmd_final(brake_cmd_final),
        .arb_src_sel    (arb_src_sel),
        .fault_wd       (fault_wd),

        .cmd_out        (cmd_out),
        .out_enable     (out_enable),
        .out_fault      (out_fault),
        .stale_cmd      (stale_cmd),
        .saturated      (saturated),

        .alive_cnt      (alive_cnt),
        .crc8           (crc8),

        .target_cmd_dbg (target_cmd_dbg),
        .enable_req_dbg (enable_req_dbg),
        .x_guard_trip_dbg(x_guard_trip_dbg)
    );

    assign out_enable_dbg = out_enable;
    assign out_fault_dbg  = out_fault;
    assign cmd_out_dbg    = cmd_out;

    // =========================================================================
    // 10) pwm_actuator (50Hz servo demo)
    // =========================================================================
    logic armed;
    logic [15:0] pulse_us_latched_dbg;
    logic [31:0] pulse_cycles_latched_dbg;

    pwm_actuator #(
        .CLK_HZ  (33_333_333),
        .CMD_MAX (CMD_MAX)
    ) u_pwm_actuator (
        .clk                     (clk_33m),
        .rst                     (rst_sync),

        .tick_1khz                (tick_1khz),
        .tick_50hz                (tick_50hz),

        .cmd_out                 (cmd_out),
        .out_enable              (out_enable),
        .out_fault               (out_fault),
        .stale_cmd               (stale_cmd),

        .pwm_out                 (pwm_out),
        .armed                   (armed),
        .pulse_us_latched_dbg    (pulse_us_latched_dbg),
        .pulse_cycles_latched_dbg(pulse_cycles_latched_dbg)
    );

    assign armed_dbg = armed;

endmodule
