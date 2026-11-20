/*
 * Copyright (c) 2026 Ruchi Prasad
 * SPDX-License-Identifier: Apache-2.0
 *
 * Top level of the programmable protocol emulator.
 * Week 1 state: pins are safe (all outputs low, bidirectionals as inputs).
 * Blocks are built and unit-tested separately, then wired in here.
 */

`default_nettype none

module tt_um_ruchii_prasad_protoemu (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so you can ignore it
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

  assign uo_out  = 8'h00;
  assign uio_out = 8'h00;
  assign uio_oe  = 8'h00;   // all bidirectional pins are inputs until a program drives them

  wire _unused = &{ena, clk, rst_n, ui_in, uio_in, 1'b0};

endmodule
