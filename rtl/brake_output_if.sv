`timescale 1ns/1ps
// NOTE (ARBSB RTL policy): no `default_nettype in synthesizable RTL.

module brake_output_if #(
    // ------------------------------------------------------------
    // Command scaling (system convention)
    // ------------------------------------------------------------
    parameter int unsigned CMD_MAX           = 1000,

    // ------------------------------------------------------------
    // Output hygiene: slew limiting (units per 1ms tick)
    // Practical: up slower than down (comfort + safety)
    // ------------------------------------------------------------
    parameter int unsigned SLEW_UP_PER_MS    = 200,
    parameter int unsigned SLEW_DN_PER_MS    = 400,

    // ------------------------------------------------------------
    // Safety policies
    // ------------------------------------------------------------
    parameter bit          RAMPDOWN_ON_SAFE  = 1'b1,  // SAFE -> ramp down to 0 on tick
    parameter bit          HARD_CUT_ON_WD    = 1'b1,  // WD -> immediate 0 (same clk edge)
    parameter bit          HARD_CUT_ON_RST   = 1'b1,  // rst -> force 0

    // ------------------------------------------------------------
    // Actuation-frame integrity (OEM-flavor)
    // ------------------------------------------------------------
    parameter bit          ENABLE_ALIVE_CRC  = 1'b1,  // generate alive_cnt + crc8
    parameter bit          CRC_INCLUDES_CMD  = 1'b1   // include cmd_out (post-shape) in CRC (default)
) (
    input  logic        clk,
    input  logic        rst,
    input  logic        tick_1khz,

    // From safety_arbiter (precise alignment)
    input  logic [15:0] brake_cmd_final,
    input  logic [1:0]  arb_src_sel,        // 0=SAFE, 1=DRIVER, 2=ARBS
    input  logic        fault_wd,

    // Outputs to pwm_actuator (or real actuator interface)
    output logic [15:0] cmd_out,
    output logic        out_enable,
    output logic        out_fault,          // sticky (WD / X-guard)
    output logic        stale_cmd,          // alias: WD fault indicates stale timing
    output logic        saturated,          // clamp indication

    // Integrity markers (optional, but we output always)
    output logic [7:0]  alive_cnt,
    output logic [7:0]  crc8,

    // Debug taps (ILA-friendly)
    output logic [15:0] target_cmd_dbg,     // post clamp, pre shape
    output logic        enable_req_dbg,
    output logic        x_guard_trip_dbg
);

    // ============================================================
    // Helpers
    // ============================================================
    logic [15:0] cmd_max_u16;

    function automatic [15:0] clamp_0_to_max(input [15:0] x, input [15:0] maxv);
        if (x > maxv) clamp_0_to_max = maxv;
        else          clamp_0_to_max = x;
    endfunction

    // Guard slew params: if 0, treat as 1 (prevents freeze)
    logic [15:0] slew_up_eff;
    logic [15:0] slew_dn_eff;

    function automatic [15:0] apply_slew(
        input [15:0] prev,
        input [15:0] target,
        input [15:0] up_step,
        input [15:0] dn_step
    );
        logic [15:0] nextv;
        begin
            if (target >= prev) begin
                if ((target - prev) > up_step) nextv = prev + up_step;
                else                            nextv = target;
            end else begin
                if ((prev - target) > dn_step) nextv = prev - dn_step;
                else                            nextv = target;
            end
            apply_slew = nextv;
        end
    endfunction

    // CRC8 (poly 0x07, init 0x00) over a packed bitstream, MSB-first
    function automatic [7:0] crc8_poly07_bits(
        input [63:0] data,
        input int unsigned nbits
    );
        int unsigned i;
        logic [7:0] c;
        logic din;
        begin
            c = 8'h00;
            for (i = 0; i < nbits; i = i + 1) begin
                din = data[nbits-1-i];               // MSB-first over nbits
                if (c[7] ^ din) c = {c[6:0], 1'b0} ^ 8'h07;
                else            c = {c[6:0], 1'b0};
            end
            crc8_poly07_bits = c;
        end
    endfunction

    // ============================================================
    // Comb logic: clamp, gating, X-guard
    // ============================================================
    logic [15:0] target_cmd_clamped;
    logic        enable_req;
    logic        x_guard_trip;

    always_comb begin
        cmd_max_u16 = (CMD_MAX > 65535) ? 16'hFFFF : CMD_MAX[15:0];

        slew_up_eff = (SLEW_UP_PER_MS < 1) ? 16'd1 : SLEW_UP_PER_MS[15:0];
        slew_dn_eff = (SLEW_DN_PER_MS < 1) ? 16'd1 : SLEW_DN_PER_MS[15:0];

        // Enable request: not SAFE + watchdog healthy
        enable_req   = (arb_src_sel != 2'd0) && (!fault_wd);

        // Sim-safe guard: if command contains X -> treat as 0 (and flag)
        x_guard_trip = (^brake_cmd_final === 1'bX);

        if (x_guard_trip) target_cmd_clamped = 16'd0;
        else              target_cmd_clamped = clamp_0_to_max(brake_cmd_final, cmd_max_u16);

        saturated = (!x_guard_trip) && (brake_cmd_final > cmd_max_u16);

        // debug taps
        target_cmd_dbg    = target_cmd_clamped;
        enable_req_dbg    = enable_req;
        x_guard_trip_dbg  = x_guard_trip;
    end

    // ============================================================
    // Fault flags
    // ============================================================
    always_ff @(posedge clk) begin
        if (rst) begin
            out_fault <= 1'b0;
        end else begin
            if (fault_wd || x_guard_trip) out_fault <= 1'b1; // sticky
        end
    end

    assign stale_cmd = fault_wd;

    // ============================================================
    // Output shaping & gating
    // ============================================================
    always_ff @(posedge clk) begin
        if (rst && HARD_CUT_ON_RST) begin
            cmd_out    <= 16'd0;
            out_enable <= 1'b0;
        end else if (HARD_CUT_ON_WD && fault_wd) begin
            // Immediate safety cut: do not wait for tick_1khz
            cmd_out    <= 16'd0;
            out_enable <= 1'b0;
        end else if (tick_1khz) begin
            out_enable <= enable_req;

            if (enable_req) begin
                cmd_out <= apply_slew(cmd_out, target_cmd_clamped, slew_up_eff, slew_dn_eff);
            end else begin
                if (RAMPDOWN_ON_SAFE) cmd_out <= apply_slew(cmd_out, 16'd0, slew_up_eff, slew_dn_eff);
                else                  cmd_out <= 16'd0;
            end
        end
    end

    // ============================================================
    // Alive counter + CRC8 "actuation frame" (frame-consistent)
    // Fix: CRC uses alive_next (the value that will be latched this tick)
    // ============================================================
    logic [15:0] cmd_for_crc;
    logic [63:0] crc_payload;
    logic [7:0]  alive_next;

    always_comb begin
        cmd_for_crc = (CRC_INCLUDES_CMD) ? cmd_out : target_cmd_clamped;

        alive_next = alive_cnt + 8'd1;

        // Pack MSB-first into lower bits then CRC over nbits=28
        //   cmd_for_crc[15:0], arb_src_sel[1:0], out_enable, out_fault, alive_next[7:0]
        crc_payload = 64'd0;
        crc_payload[27:12] = cmd_for_crc;
        crc_payload[11:10] = arb_src_sel;
        crc_payload[9]     = out_enable;
        crc_payload[8]     = out_fault;
        crc_payload[7:0]   = alive_next;
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            alive_cnt <= 8'd0;
            crc8      <= 8'd0;
        end else if (HARD_CUT_ON_WD && fault_wd) begin
            // keep integrity markers safe on a hard cut
            alive_cnt <= alive_cnt; // hold
            crc8      <= 8'd0;
        end else if (tick_1khz) begin
            if (ENABLE_ALIVE_CRC) begin
                alive_cnt <= alive_next;
                crc8      <= crc8_poly07_bits(crc_payload, 28);
            end else begin
                alive_cnt <= 8'd0;
                crc8      <= 8'd0;
            end
        end
    end

endmodule
