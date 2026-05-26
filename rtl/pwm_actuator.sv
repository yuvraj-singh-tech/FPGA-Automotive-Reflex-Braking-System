`timescale 1ns/1ps
// NOTE (ARBSB RTL policy): no `default_nettype in synthesizable RTL.

// ============================================================================
// pwm_actuator.sv (ARBS-B) - TIMING-OPTIMIZED (internal pipelining)
// ============================================================================

module pwm_actuator #(
    parameter int unsigned CLK_HZ        = 33_333_333,
    parameter int unsigned CMD_MAX       = 1000,
    parameter int unsigned PULSE_MIN_US  = 1100,
    parameter int unsigned PULSE_MAX_US  = 1900,
    parameter int unsigned SAFE_US       = 1100,
    parameter int unsigned ARM_MS        = 500
) (
    input  logic        clk,
    input  logic        rst,

    input  logic        tick_1khz,
    input  logic        tick_50hz,

    input  logic [15:0] cmd_out,
    input  logic        out_enable,
    input  logic        out_fault,
    input  logic        stale_cmd,

    output logic        pwm_out,
    output logic        armed,
    output logic [15:0] pulse_us_latched_dbg,
    output logic [31:0] pulse_cycles_latched_dbg
);

    // ============================================================
    // Clamp helper
    // ============================================================
    function automatic [15:0] clamp_0_to_max_u16(input [15:0] x, input [15:0] maxv);
        if (x > maxv) clamp_0_to_max_u16 = maxv;
        else          clamp_0_to_max_u16 = x;
    endfunction

    // ============================================================
    // us -> cycles helper (unchanged)
    // ============================================================
    function automatic [31:0] us_to_cycles(input int unsigned us);
        longint unsigned num;
        num = (longint'(us) * CLK_HZ) + 500_000;
        us_to_cycles = num / 1_000_000;
    endfunction

    // ============================================================
    // Arming logic (UNCHANGED)
    // ============================================================
    localparam int unsigned ARM_DEN   = (ARM_MS < 1) ? 1 : ARM_MS;
    localparam int unsigned ARM_CNT_W = (ARM_DEN <= 1) ? 1 : $clog2(ARM_DEN + 1);

    logic [ARM_CNT_W-1:0] arm_cnt_ms;
    logic healthy_now;

    always_comb
        healthy_now = out_enable && !out_fault && !stale_cmd;

    always_ff @(posedge clk) begin
        if (rst) begin
            armed      <= 1'b0;
            arm_cnt_ms <= '0;
        end else if (!healthy_now) begin
            armed      <= 1'b0;
            arm_cnt_ms <= '0;
        end else if (tick_1khz && !armed) begin
            arm_cnt_ms <= arm_cnt_ms + 1'b1;
            if (arm_cnt_ms + 1'b1 >= ARM_DEN)
                armed <= 1'b1;
        end
    end

    // ============================================================
    // STAGE 1 PIPELINE: cmd ? pulse_us
    // ============================================================
    localparam int unsigned CMD_SPAN_US = PULSE_MAX_US - PULSE_MIN_US;

    logic        safe_mode_s1;
    logic [15:0] pulse_us_s1;

    always_ff @(posedge clk) begin
        if (rst) begin
            pulse_us_s1 <= SAFE_US[15:0];
            safe_mode_s1 <= 1'b1;
        end else begin
            safe_mode_s1 <= (!out_enable) || out_fault || stale_cmd || (!armed);

            if ((!out_enable) || out_fault || stale_cmd || (!armed)) begin
                pulse_us_s1 <= SAFE_US[15:0];
            end else begin
                pulse_us_s1 <= PULSE_MIN_US[15:0] +
                               ((clamp_0_to_max_u16(cmd_out, CMD_MAX[15:0]) *
                                 CMD_SPAN_US + (CMD_MAX/2)) / CMD_MAX);
            end

            if (pulse_us_s1 < PULSE_MIN_US) pulse_us_s1 <= PULSE_MIN_US;
            if (pulse_us_s1 > PULSE_MAX_US) pulse_us_s1 <= PULSE_MAX_US;
        end
    end

    // ============================================================
    // STAGE 2 PIPELINE: pulse_us ? cycles (tick_50hz)
    // ============================================================
    logic [31:0] frame_cnt;
    logic [31:0] pulse_cycles_latched;
    logic [15:0] pulse_us_latched;

    always_ff @(posedge clk) begin
        if (rst) begin
            frame_cnt            <= 32'd0;
            pulse_us_latched     <= SAFE_US[15:0];
            pulse_cycles_latched <= us_to_cycles(SAFE_US);
        end else if (tick_50hz) begin
            frame_cnt            <= 32'd0;
            pulse_us_latched     <= pulse_us_s1;
            pulse_cycles_latched <= us_to_cycles(pulse_us_s1);
        end else begin
            frame_cnt <= frame_cnt + 1'b1;
        end
    end

    always_ff @(posedge clk) begin
        if (rst)
            pwm_out <= 1'b0;
        else
            pwm_out <= (frame_cnt < pulse_cycles_latched);
    end

    // Debug taps (UNCHANGED)
    assign pulse_us_latched_dbg     = pulse_us_latched;
    assign pulse_cycles_latched_dbg = pulse_cycles_latched;

endmodule
