`timescale 1ns/1ps

// =============================================================================
// Module      : tick_gen
// Project     : FPGA Automotive Reflex Braking System (ARBS)
// Author      : Yuvraj Singh
// -----------------------------------------------------------------------------
// Description :
//   Generates fixed-rate timing pulses for the ARBS control pipeline.
//
//   The module derives a 1 kHz system tick from the FPGA clock and then derives
//   a 50 Hz actuator-frame tick from the 1 kHz tick.
//
// Key Functions:
//   - Generates a 1 ms tick as a single-clock pulse
//   - Generates a 20 ms / 50 Hz tick as a single-clock pulse
//   - Keeps all timing synchronous to the system clock
//   - Provides timing references for control, debounce, watchdog, and PWM logic
//
// Design Notes:
//   The default constants are selected for a 33.333 MHz system clock.
// =============================================================================

module tick_gen (
    input  logic clk,       // 33.333 MHz system clock
    input  logic rst,       // synchronous, active-high (from reset_sync)
    output logic tick_1khz,  // 1 ms tick, 1-cycle pulse
    output logic tick_50hz   // 20 ms tick, 1-cycle pulse
);

    // 1 kHz tick: ~1 ms
    localparam int CYCLES_1KHZ = 33333;  // 33.333 MHz / 1 kHz

    // 50 Hz derived from 1 kHz tick -> 20 ms = 20 * 1 ms
    localparam int DIV_50HZ_FROM_1KHZ = 20;

    logic [19:0] ctr_1khz;   // enough for 33333
    logic [4:0]  ctr_50hz;   // counts 0..19

    always_ff @(posedge clk) begin
        if (rst) begin
            ctr_1khz  <= 20'd0;
            ctr_50hz  <= 5'd0;
            tick_1khz <= 1'b0;
            tick_50hz <= 1'b0;
        end else begin
            // 1 kHz tick
            if (ctr_1khz == CYCLES_1KHZ - 1) begin
                ctr_1khz  <= 20'd0;
                tick_1khz <= 1'b1;
            end else begin
                ctr_1khz  <= ctr_1khz + 20'd1;
                tick_1khz <= 1'b0;
            end

            // 50 Hz tick derived from the 1 kHz timing pulse
            if (tick_1khz) begin
                if (ctr_50hz == DIV_50HZ_FROM_1KHZ - 1) begin
                    ctr_50hz  <= 5'd0;
                    tick_50hz <= 1'b1;
                end else begin
                    ctr_50hz  <= ctr_50hz + 5'd1;
                    tick_50hz <= 1'b0;
                end
            end else begin
                tick_50hz <= 1'b0; // keep high for exactly 1 clk
            end
        end
    end

endmodule
