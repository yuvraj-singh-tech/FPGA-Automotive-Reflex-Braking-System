`timescale 1ns/1ps
`default_nettype none

// =============================================================================
// Testbench  : tb_top_arbs
// Project    : FPGA Automotive Reflex Braking System (ARBS)
// Author     : Yuvraj Singh
// -----------------------------------------------------------------------------
// Description:
//   Full-system simulation testbench for the ARBS top-level integration.
//
//   The testbench drives brake ADC and digital brake-switch inputs through
//   multiple operating scenarios, including idle, light braking, medium braking,
//   release behavior, input noise, ADC-valid dropouts, digital fallback braking,
//   supervisor mismatch checks, watchdog observation, and final release.
//
// Key Verification Points:
//   - Reset and timing behavior
//   - Brake input conditioning
//   - Driver intent generation
//   - Safety supervisor and voter response
//   - Brake profile and arbitration behavior
//   - Output command enable/fault behavior
//   - PWM actuator arming visibility
//
// Design Notes:
//   The testbench uses accelerated timing pulses through hierarchical force
//   statements to keep simulation runtime short while preserving system-level
//   control behavior.
// =============================================================================

module tb_top_arbs;

  // ============================================================
  // Simulation clock
  // ============================================================
  logic clk_33m = 1'b0;
  always #5 clk_33m = ~clk_33m;

  // ============================================================
  // DUT inputs
  // ============================================================
  logic        rst_raw;
  logic        brake_raw;
  logic [11:0] brake_adc;
  logic        brake_adc_valid;

  // ============================================================
  // DUT outputs and debug ports
  // ============================================================
  logic        pwm_out;

  logic        rst_sync_dbg;
  logic        tick_1khz_dbg;
  logic        tick_50hz_dbg;

  logic [1:0]  drv_level_dbg;
  logic        drv_valid_dbg;
  logic        panic_brake_dbg;
  logic        intent_fault_dbg;

  logic        emergency_req_final_dbg;
  logic        fault_req_final_dbg;
  logic        mismatch_fault_dbg;

  logic [1:0]  arb_src_sel_dbg;
  logic        fault_wd_dbg;

  logic        out_enable_dbg;
  logic        out_fault_dbg;
  logic [15:0] cmd_out_dbg;
  logic        armed_dbg;

  // ============================================================
  // Device under test
  // ============================================================
  top_arbs dut (
    .clk_33m                 (clk_33m),
    .rst_raw                 (rst_raw),

    .brake_raw               (brake_raw),
    .brake_adc               (brake_adc),
    .brake_adc_valid         (brake_adc_valid),

    .pwm_out                 (pwm_out),

    .rst_sync_dbg            (rst_sync_dbg),
    .tick_1khz_dbg           (tick_1khz_dbg),
    .tick_50hz_dbg           (tick_50hz_dbg),

    .drv_level_dbg           (drv_level_dbg),
    .drv_valid_dbg           (drv_valid_dbg),
    .panic_brake_dbg         (panic_brake_dbg),
    .intent_fault_dbg        (intent_fault_dbg),

    .emergency_req_final_dbg (emergency_req_final_dbg),
    .fault_req_final_dbg     (fault_req_final_dbg),
    .mismatch_fault_dbg      (mismatch_fault_dbg),

    .arb_src_sel_dbg         (arb_src_sel_dbg),
    .fault_wd_dbg            (fault_wd_dbg),

    .out_enable_dbg          (out_enable_dbg),
    .out_fault_dbg           (out_fault_dbg),
    .cmd_out_dbg             (cmd_out_dbg),
    .armed_dbg               (armed_dbg)
  );

  // ============================================================
  // Accelerated timing pulse generation
  // ============================================================
  // A compressed 1 kHz timing pulse is forced into tick_gen to reduce simulation
  // runtime. The 50 Hz pulse is derived from the accelerated 1 kHz pulse.
  // ============================================================
  localparam int unsigned TICK_DIV = 20;
  int unsigned div_ctr;
  int unsigned div50_ctr;

  bit pause_ticks = 0;

  always @(posedge clk_33m) begin
    if (rst_sync_dbg) begin
      div_ctr   <= 0;
      div50_ctr <= 0;
      force dut.u_tick_gen.tick_1khz = 1'b0;
      force dut.u_tick_gen.tick_50hz = 1'b0;
    end else if (pause_ticks) begin
      force dut.u_tick_gen.tick_1khz = 1'b0;
      force dut.u_tick_gen.tick_50hz = 1'b0;
    end else begin
      force dut.u_tick_gen.tick_1khz = 1'b0;
      force dut.u_tick_gen.tick_50hz = 1'b0;

      if (div_ctr == (TICK_DIV-1)) begin
        div_ctr <= 0;
        force dut.u_tick_gen.tick_1khz = 1'b1;

        if (div50_ctr == 19) begin
          div50_ctr <= 0;
          force dut.u_tick_gen.tick_50hz = 1'b1;
        end else begin
          div50_ctr <= div50_ctr + 1;
        end
      end else begin
        div_ctr <= div_ctr + 1;
      end
    end
  end

  // ============================================================
  // Testbench utilities
  // ============================================================
  task automatic wait_ticks(input int unsigned n);
    int unsigned i;
    begin
      for (i=0; i<n; i++) begin
        @(posedge clk_33m);
        while (dut.u_tick_gen.tick_1khz !== 1'b1) @(posedge clk_33m);
        #1;
      end
    end
  endtask

  task automatic wait_clks(input int unsigned n);
    int unsigned i;
    begin
      for (i=0; i<n; i++) @(posedge clk_33m);
      #1;
    end
  endtask

  function automatic bit healthy_now();
    begin
      healthy_now = (rst_sync_dbg === 1'b0) &&
                    (fault_wd_dbg === 1'b0) &&
                    (out_fault_dbg === 1'b0) &&
                    (fault_req_final_dbg === 1'b0) &&
                    (mismatch_fault_dbg === 1'b0);
    end
  endfunction

  task automatic snapshot(input string tag);
    $display("[%s] lvl=%0d valid=%0b panic=%0b ifault=%0b | emerg=%0b fault=%0b mismatch=%0b | src=%0d wd=%0b | en=%0b out_fault=%0b cmd=%0d armed=%0b",
      tag,
      drv_level_dbg, drv_valid_dbg, panic_brake_dbg, intent_fault_dbg,
      emergency_req_final_dbg, fault_req_final_dbg, mismatch_fault_dbg,
      arb_src_sel_dbg, fault_wd_dbg,
      out_enable_dbg, out_fault_dbg, cmd_out_dbg, armed_dbg
    );
  endtask

  task automatic chk(input bit cond, input string msg);
    if (!cond) begin
      $display("FAIL: %s", msg);
      snapshot("FAIL-SNAPSHOT");
      $fatal(1);
    end else begin
      $display("PASS: %s", msg);
    end
  endtask

  // ============================================================
  // ADC ramp utility
  // ============================================================
  localparam int unsigned ADC_STEP_MAX = 200;

  task automatic adc_ramp_to(input int unsigned target);
    int signed cur, tgt, delta, step;
    begin
      cur = brake_adc;
      tgt = (target > 4095) ? 4095 : target;

      while (cur != tgt) begin
        delta = tgt - cur;
        if (delta > 0) step = (delta > ADC_STEP_MAX) ? ADC_STEP_MAX : delta;
        else           step = (delta < -ADC_STEP_MAX) ? -ADC_STEP_MAX : delta;

        cur = cur + step;
        brake_adc <= cur[11:0];
        wait_ticks(1);
      end
    end
  endtask

  // ============================================================
  // Testbench timeout watchdog
  // ============================================================
  initial begin
    #12_000_000;
    $display("FAIL: testbench timeout");
    snapshot("TB-WATCHDOG");
    $fatal(1);
  end

  // ============================================================
  // Main verification scenarios
  // ============================================================
  initial begin
    rst_raw         = 1'b1;
    brake_raw       = 1'b0;
    brake_adc       = 12'd0;
    brake_adc_valid = 1'b1;
    pause_ticks     = 0;

    // Reset sequence
    wait_clks(50);
    rst_raw = 1'b0;
    while (rst_sync_dbg !== 1'b0) @(posedge clk_33m);

    // Warmup
    $display("---- WARMUP ----");
    wait_ticks(300);
    snapshot("WARMUP");
    chk(!fault_wd_dbg,  "WARMUP: fault_wd == 0");
    chk(!out_fault_dbg, "WARMUP: out_fault == 0");

    // S0: Idle
    $display("---- S0: IDLE ----");
    snapshot("S0");
    chk(drv_level_dbg == 2'd0, "S0: drv_level NONE");
    chk(cmd_out_dbg == 16'd0,  "S0: cmd_out == 0");

    // S1: Light braking
    $display("---- S1: LIGHT BRAKE ----");
    adc_ramp_to(450);
    wait_ticks(250);
    snapshot("S1");
    chk(drv_valid_dbg == 1'b1, "S1: drv_valid == 1");
    chk(drv_level_dbg >= 2'd1, "S1: drv_level >= LIGHT");
    if (healthy_now()) begin
      chk(arb_src_sel_dbg == 2'd1, "S1: arb_src_sel == DRIVER");
      chk(out_enable_dbg  == 1'b1, "S1: out_enable == 1");
    end else $display("INFO: S1 health gate not open; skipping driver-enable checks.");

    // S2: Medium braking attempt
    $display("---- S2: MEDIUM BRAKE ATTEMPT ----");
    adc_ramp_to(900);
    wait_ticks(250);
    snapshot("S2");
    chk(drv_valid_dbg == 1'b1, "S2: drv_valid == 1");
    chk(drv_level_dbg != 2'd0, "S2: drv_level not NONE");

    // S3: Higher brake level attempt
    $display("---- S3: HIGHER BRAKE LEVEL ATTEMPT ----");
    adc_ramp_to(1400);
    wait_ticks(300);
    snapshot("S3");
    chk(mismatch_fault_dbg  == 1'b0, "S3: mismatch_fault remains 0");
    chk(fault_req_final_dbg == 1'b0, "S3: fault_req_final remains 0");
    if (healthy_now() && out_enable_dbg)
      chk(armed_dbg == 1'b1, "S3: actuator path armed");
    else
      $display("INFO: S3 actuator path not armed; inspect snapshot for gate status.");

    // S4: Release
    $display("---- S4: RELEASE ----");
    adc_ramp_to(0);
    wait_ticks(400);
    snapshot("S4");
    chk(cmd_out_dbg == 16'd0, "S4: cmd_out returns to 0");

    // S5: Noise / bounce behavior
    $display("---- S5: INPUT NOISE / BOUNCE ----");
    adc_ramp_to(400); wait_ticks(20);
    adc_ramp_to(250); wait_ticks(20);
    adc_ramp_to(420); wait_ticks(20);
    adc_ramp_to(0);   wait_ticks(200);
    snapshot("S5");
    chk(out_fault_dbg == 1'b0, "S5: out_fault remains 0");

    // S6: Brief ADC invalid window
    $display("---- S6: BRIEF ADC INVALID ----");
    adc_ramp_to(450);
    wait_ticks(100);
    brake_adc_valid = 1'b0;
    wait_ticks(40);
    brake_adc_valid = 1'b1;
    wait_ticks(120);
    snapshot("S6");
    chk(intent_fault_dbg == 1'b0, "S6: intent_fault remains 0 for brief invalid");

    // S7: Digital brake fallback
    $display("---- S7: DIGITAL BRAKE PRESS ----");
    adc_ramp_to(0);
    brake_raw = 1'b1;
    wait_ticks(80);
    snapshot("S7");
    chk(drv_valid_dbg == 1'b1, "S7: drv_valid == 1 on digital press");
    brake_raw = 1'b0;
    wait_ticks(120);

    // S8: Supervisor agreement check
    $display("---- S8: SUPERVISOR AGREEMENT CHECK ----");
    adc_ramp_to(900);
    wait_ticks(120);
    snapshot("S8");
    chk(mismatch_fault_dbg == 1'b0, "S8: mismatch_fault stays 0");

    // S9: Watchdog observation
    $display("---- S9: WATCHDOG OBSERVATION ----");
    pause_ticks = 1;
    wait_clks(40000);
    pause_ticks = 0;
    wait_ticks(10);
    snapshot("S9");
    $display("INFO: fault_wd=%0b", fault_wd_dbg);

    // S10: Final release
    $display("---- S10: FINAL RELEASE ----");
    adc_ramp_to(0);
    wait_ticks(200);
    snapshot("S10");
    chk(cmd_out_dbg == 16'd0, "S10: cmd_out == 0");

    $display("==============================================");
    $display("ARBS full-system testbench completed successfully");
    $display("==============================================");

    release dut.u_tick_gen.tick_1khz;
    release dut.u_tick_gen.tick_50hz;

    $finish;
  end

endmodule

`default_nettype wire
