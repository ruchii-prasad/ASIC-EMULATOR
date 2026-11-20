/*
 * Copyright (c) 2026 Ruchi Prasad
 * SPDX-License-Identifier: Apache-2.0
 *
 * clk_div: programmable tick generator.
 * When en=1, `tick` pulses high for exactly one clk cycle every (div + 1) cycles.
 *   div = 0  -> tick every cycle
 *   div = 433 -> 50 MHz / 434 = 115207 Hz (115200 baud, 0.006% error)
 * The design never creates a new clock: logic stays on `clk` and uses `tick`
 * as an enable. This keeps the ASIC to a single clock domain (easier timing).
 */

`default_nettype none

module clk_div #(
    parameter W = 16
) (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         en,
    input  wire [W-1:0] div,
    output reg          tick
);

  reg [W-1:0] count;

  always @(posedge clk) begin
    if (!rst_n || !en) begin
      count <= {W{1'b0}};
      tick  <= 1'b0;
    end else if (count >= div) begin   // >= so a div lowered mid-count can't overshoot
      count <= {W{1'b0}};
      tick  <= 1'b1;
    end else begin
      count <= count + 1'b1;
      tick  <= 1'b0;
    end
  end

endmodule
