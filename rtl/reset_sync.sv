`timescale 1ns/1ps

// =============================================================================
// Module      : reset_sync
// Project     : FPGA Automotive Reflex Braking System (ARBS)
// Author      : Yuvraj Singh
// -----------------------------------------------------------------------------
// Description :
//   Generates a clean active-high reset for the ARBS synchronous logic.
//
//   The module accepts an asynchronous raw reset input and produces a reset that
//   asserts immediately but deasserts synchronously after two clock cycles.
//
// Key Functions:
//   - Supports asynchronous reset assertion
//   - Provides two-flop synchronized reset release
//   - Initializes the synchronizer in reset after FPGA configuration
//   - Uses ASYNC_REG attributes for FPGA implementation guidance
//
// Design Notes:
//   This reset is intended to be the common internal reset source for downstream
//   ARBS modules.
// =============================================================================

module reset_sync (
    input  logic clk,       // 33.333 MHz system clock
    input  logic rst_raw,     // async, active-high (button / POR)
    output logic rst_sync     // clean, active-high, sync deassert
);

    // 2-FF synchronizer with async assert, sync deassert.
    // Init to 2'b11 so design starts in reset after config.
    (* ASYNC_REG = "TRUE" *) logic [1:0] sync_ff = 2'b11;

    always_ff @(posedge clk or posedge rst_raw) begin
        if (rst_raw) begin
            // Asynchronous assertion
            sync_ff <= 2'b11;
        end else begin
            // Synchronous deassertion over 2 cycles
            sync_ff <= {1'b0, sync_ff[1]};
        end
    end

    always_comb begin
        rst_sync = sync_ff[0];
    end

endmodule
