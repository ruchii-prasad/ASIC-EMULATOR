/*
 * Test wrapper for one SM: instruction memory + TX FIFO + clock divider + sm.
 * Only used by the unit test; the real top level comes later.
 */
`default_nettype none
`timescale 1ns / 1ps

module sm_wrap (
    input  wire        clk,
    input  wire        rst_n,
    // instruction memory write port
    input  wire        imem_we,
    input  wire [4:0]  imem_addr,
    input  wire [15:0] imem_wdata,
    // TX FIFO write port
    input  wire        tx_push,
    input  wire [7:0]  tx_wdata,
    output wire        tx_full,
    // control
    input  wire        en,
    input  wire        restart,
    input  wire [15:0] clkdiv,
    input  wire        exec_valid,
    input  wire [15:0] exec_instr,
    // config
    input  wire [4:0]  wrap_bottom,
    input  wire [4:0]  wrap_top,
    input  wire        side_opt,
    input  wire [1:0]  side_count,
    input  wire        out_shift_right,
    input  wire [3:0]  pull_thresh,
    input  wire [2:0]  out_base,
    input  wire [3:0]  out_count,
    input  wire [2:0]  set_base,
    input  wire [2:0]  set_count,
    input  wire [2:0]  side_base,
    input  wire [2:0]  jmp_pin,
    // pins
    input  wire [7:0]  pins_in,
    output wire [7:0]  pin_val,
    output wire [7:0]  pin_dir,
    output wire [4:0]  pc,
    output wire        stalled
);

  initial begin
    $dumpfile("sm_wrap.fst");
    $dumpvars(0, sm_wrap);
  end

  reg [15:0] imem [0:31];
  always @(posedge clk) if (imem_we) imem[imem_addr] <= imem_wdata;

  wire tick;
  clk_div #(.W(16)) u_div (.clk(clk), .rst_n(rst_n), .en(en), .div(clkdiv), .tick(tick));

  wire       tx_empty, tx_pop;
  wire [7:0] tx_data;
  fifo #(.W(8), .DEPTH(4), .AW(2)) u_txf (
      .clk(clk), .rst_n(rst_n), .push(tx_push), .din(tx_wdata), .pop(tx_pop),
      .dout(tx_data), .empty(tx_empty), .full(tx_full), .count());

  sm u_sm (
      .clk(clk), .rst_n(rst_n), .en(en), .tick(tick), .restart(restart),
      .pc_o(pc), .imem_data(imem[pc]),
      .exec_valid(exec_valid), .exec_instr(exec_instr),
      .wrap_bottom(wrap_bottom), .wrap_top(wrap_top),
      .side_opt(side_opt), .side_count(side_count),
      .out_shift_right(out_shift_right), .pull_thresh(pull_thresh),
      .out_base(out_base), .out_count(out_count),
      .set_base(set_base), .set_count(set_count),
      .side_base(side_base), .jmp_pin(jmp_pin),
      .pins_in(pins_in), .pin_val(pin_val), .pin_dir(pin_dir),
      .tx_empty(tx_empty), .tx_data(tx_data), .tx_pop(tx_pop),
      .stalled(stalled));

endmodule
