/*
 * Copyright (c) 2026 Ruchi Prasad
 * SPDX-License-Identifier: Apache-2.0
 *
 * fifo: small synchronous FIFO (first in, first out).
 * `dout` always shows the oldest entry, so the reader can use it in the same
 * cycle it asserts `pop` ("show-ahead" / first-word-fall-through).
 * Pushing when full or popping when empty is ignored.
 */

`default_nettype none

module fifo #(
    parameter W     = 8,
    parameter DEPTH = 4,          // must be a power of two
    parameter AW    = 2           // log2(DEPTH)
) (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         push,
    input  wire [W-1:0] din,
    input  wire         pop,
    output wire [W-1:0] dout,
    output wire         empty,
    output wire         full,
    output reg  [AW:0]  count
);

  reg [W-1:0]  mem [0:DEPTH-1];
  reg [AW-1:0] rd_ptr, wr_ptr;

  assign empty = (count == 0);
  assign full  = (count == DEPTH);
  assign dout  = mem[rd_ptr];

  wire do_push = push && !full;
  wire do_pop  = pop && !empty;

  always @(posedge clk) begin
    if (!rst_n) begin
      rd_ptr <= {AW{1'b0}};
      wr_ptr <= {AW{1'b0}};
      count  <= {(AW+1){1'b0}};
    end else begin
      if (do_push) begin
        mem[wr_ptr] <= din;
        wr_ptr      <= wr_ptr + 1'b1;
      end
      if (do_pop) rd_ptr <= rd_ptr + 1'b1;
      count <= count + {{AW{1'b0}}, do_push} - {{AW{1'b0}}, do_pop};
    end
  end

endmodule
