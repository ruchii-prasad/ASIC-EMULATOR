/*
 * Copyright (c) 2026 Ruchi Prasad
 * SPDX-License-Identifier: Apache-2.0
 *
 * sm: one protocol-engine state machine (docs/isa.md).
 *
 * Implements the full v0.2 ISA: JMP, WAIT, IN, OUT, PUSH, PULL, MOV, IRQ, SET,
 * side-set, delay, wrap, autopull/autopush, and forced execution (EXEC).
 *
 * Timing: the SM does one step per `tick` (from clk_div). A step either
 *   - burns one delay cycle, or
 *   - executes the instruction at PC, or stalls on it (e.g. PULL with an
 *     empty FIFO, WAIT on a pin that has not changed yet).
 * Side-set is applied whenever an instruction executes, including while stalled.
 */

`default_nettype none

module sm (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        en,              // SM enabled
    input  wire        tick,            // one-cycle step enable from clk_div
    input  wire        restart,         // PC <- wrap_bottom, clear delay and IRQ wait

    // instruction memory (shared; combinational read)
    output wire [4:0]  pc_o,
    input  wire [15:0] imem_data,

    // forced instruction from the host: runs on the next clk, ignores delay,
    // only changes PC if it is a jump. A stalling EXEC is dropped.
    input  wire        exec_valid,
    input  wire [15:0] exec_instr,

    // configuration
    input  wire [4:0]  wrap_bottom,
    input  wire [4:0]  wrap_top,
    input  wire        side_opt,
    input  wire [1:0]  side_count,      // 0..2
    input  wire        out_shift_right, // 1 = LSB first
    input  wire        in_shift_right,  // 1 = new bits enter at the MSB end
    input  wire        autopull,
    input  wire        autopush,
    input  wire [3:0]  pull_thresh,     // 1..8
    input  wire [3:0]  push_thresh,     // 1..8
    input  wire [2:0]  out_base,
    input  wire [3:0]  out_count,       // 0..8
    input  wire [2:0]  set_base,
    input  wire [2:0]  set_count,       // 0..5
    input  wire [2:0]  in_base,
    input  wire [2:0]  side_base,
    input  wire [2:0]  jmp_pin,

    // pins
    input  wire [7:0]  pins_in,
    output reg  [7:0]  pin_val,         // value this SM drives
    output reg  [7:0]  pin_dir,         // 1 = output

    // TX FIFO (host -> SM)
    input  wire        tx_empty,
    input  wire [7:0]  tx_data,
    output reg         tx_pop,

    // RX FIFO (SM -> host)
    input  wire        rx_full,
    output reg         rx_push,
    output reg  [7:0]  rx_data,

    // IRQ flags (shared between SMs; the flag register lives outside)
    input  wire [7:0]  irq_flags,
    output reg  [7:0]  irq_set,
    output reg  [7:0]  irq_clr,

    output reg         stalled
);

  // ---------------- opcodes ----------------
  localparam OP_JMP  = 3'b000, OP_WAIT = 3'b001, OP_IN  = 3'b010, OP_OUT = 3'b011,
             OP_PUSH = 3'b100, OP_MOV  = 3'b101, OP_IRQ = 3'b110, OP_SET = 3'b111;

  // ---------------- architectural state ----------------
  reg [4:0] pc;
  reg [7:0] x, y, osr, isr;
  reg [3:0] osr_count;     // bits shifted out of OSR since the last load (8 = empty)
  reg [3:0] isr_count;     // bits shifted into ISR since the last clear
  reg [4:0] delay_cnt;
  reg       irq_waiting;   // IRQ WAIT has set its flag and is waiting for it to clear

  assign pc_o = pc;

  // ---------------- helpers ----------------
  // rotate an 8-bit value left / right by b (pin windows wrap modulo 8)
  function [7:0] rotl;
    input [7:0] v;
    input [2:0] b;
    reg  [15:0] t;
    begin
      t    = {v, v} << b;
      rotl = t[15:8];
    end
  endfunction

  function [7:0] rotr;
    input [7:0] v;
    input [2:0] b;
    reg  [15:0] t;
    begin
      t    = {v, v} >> b;
      rotr = t[7:0];
    end
  endfunction

  // mask with the low `n` bits set (n = 0..8)
  function [7:0] low_mask;
    input [3:0] n;
    reg   [8:0] t;
    begin
      t        = (9'd1 << n) - 9'd1;
      low_mask = t[7:0];
    end
  endfunction

  // write the low `n` bits of `data` into `pins` starting at pin `base`
  function [7:0] write_pins;
    input [7:0] pins;
    input [2:0] base;
    input [3:0] n;
    input [7:0] data;
    reg   [7:0] m;
    begin
      m          = rotl(low_mask(n), base);
      write_pins = (pins & ~m) | (rotl(data, base) & m);
    end
  endfunction

  function [7:0] bitrev;
    input [7:0] v;
    integer i;
    begin
      for (i = 0; i < 8; i = i + 1) bitrev[i] = v[7-i];
    end
  endfunction

  function [3:0] sat_add8;   // min(a + b, 8)
    input [3:0] a, b;
    reg   [4:0] s;
    begin
      s        = a + b;
      sat_add8 = (s > 5'd8) ? 4'd8 : s[3:0];
    end
  endfunction

  // ---------------- decode ----------------
  wire [15:0] instr  = exec_valid ? exec_instr : imem_data;
  wire [2:0]  opcode = instr[15:13];
  wire [4:0]  ssd    = instr[12:8];    // side-set / delay field
  wire [7:0]  ops    = instr[7:0];

  wire run = exec_valid || (en && tick && delay_cnt == 5'd0);

  // split the side-set / delay field according to the config
  reg       side_en;
  reg [1:0] side_val;
  reg [4:0] delay_val;
  always @(*) begin
    side_en   = 1'b0;
    side_val  = 2'b00;
    delay_val = ssd;
    case ({side_opt, side_count})
      3'b0_01: begin side_en = 1'b1;    side_val = {1'b0, ssd[4]}; delay_val = {1'b0, ssd[3:0]}; end
      3'b0_10: begin side_en = 1'b1;    side_val = ssd[4:3];       delay_val = {2'b0, ssd[2:0]}; end
      3'b1_01: begin side_en = ssd[4];  side_val = {1'b0, ssd[3]}; delay_val = {2'b0, ssd[2:0]}; end
      3'b1_10: begin side_en = ssd[4];  side_val = ssd[3:2];       delay_val = {3'b0, ssd[1:0]}; end
      default: ;  // side_count = 0: whole field is delay
    endcase
  end

  // bit count for IN/OUT: 0 encodes 8
  wire [3:0] nbits    = (ops[2:0] == 3'd0) ? 4'd8 : {1'b0, ops[2:0]};
  wire [7:0] pins_rel = rotr(pins_in, in_base);   // pin in_base appears at bit 0

  // ---------------- OUT shifter (with autopull refill) ----------------
  // With autopull, an OUT that finds OSR at/over the threshold first refills
  // OSR from the TX FIFO, then shifts from the fresh value in the same step.
  wire       out_refill = autopull && (osr_count >= pull_thresh);
  wire [7:0] osr_src    = out_refill ? tx_data : osr;
  wire [3:0] osr_cnt_src= out_refill ? 4'd0    : osr_count;

  reg [7:0] out_data, osr_shifted;
  always @(*) begin
    if (out_shift_right) begin
      out_data    = osr_src & low_mask(nbits);
      osr_shifted = osr_src >> nbits;
    end else begin
      out_data    = osr_src >> (4'd8 - nbits);
      osr_shifted = osr_src << nbits;
    end
  end

  wire [3:0] out_n = (nbits < out_count) ? nbits : out_count;  // pins actually written

  // ---------------- IN shifter ----------------
  reg [7:0] in_src, isr_shifted;
  always @(*) begin
    case (ops[7:5])
      3'd0:    in_src = pins_rel;
      3'd1:    in_src = x;
      3'd2:    in_src = y;
      3'd6:    in_src = isr;
      3'd7:    in_src = osr;
      default: in_src = 8'd0;   // NULL
    endcase
    if (in_shift_right)
      isr_shifted = (isr >> nbits) | ((in_src & low_mask(nbits)) << (4'd8 - nbits));
    else
      isr_shifted = (isr << nbits) | (in_src & low_mask(nbits));
  end
  wire [3:0] isr_cnt_after = sat_add8(isr_count, nbits);

  // ---------------- MOV source ----------------
  reg [7:0] mov_src, mov_val;
  always @(*) begin
    case (ops[2:0])
      3'd0:    mov_src = pins_rel;
      3'd1:    mov_src = x;
      3'd2:    mov_src = y;
      3'd5:    mov_src = tx_empty ? 8'hFF : 8'h00;   // STATUS
      3'd6:    mov_src = isr;
      3'd7:    mov_src = osr;
      default: mov_src = 8'd0;                       // NULL
    endcase
    case (ops[4:3])
      2'd1:    mov_val = ~mov_src;
      2'd2:    mov_val = bitrev(mov_src);
      default: mov_val = mov_src;
    endcase
  end

  // ---------------- execute (combinational next state) ----------------
  reg [7:0] n_x, n_y, n_osr, n_isr, n_pin_val, n_pin_dir;
  reg [3:0] n_osr_count, n_isr_count;
  reg       jump, stall, n_irq_waiting;
  reg [4:0] jump_target;
  reg       wait_ok;

  always @(*) begin
    n_x           = x;
    n_y           = y;
    n_osr         = osr;
    n_isr         = isr;
    n_osr_count   = osr_count;
    n_isr_count   = isr_count;
    n_pin_val     = pin_val;
    n_pin_dir     = pin_dir;
    n_irq_waiting = irq_waiting;
    jump          = 1'b0;
    jump_target   = ops[4:0];
    stall         = 1'b0;
    tx_pop        = 1'b0;
    rx_push       = 1'b0;
    rx_data       = isr;
    irq_set       = 8'd0;
    irq_clr       = 8'd0;
    wait_ok       = 1'b0;

    case (opcode)
      OP_JMP: begin
        case (ops[7:5])
          3'd0: jump = 1'b1;
          3'd1: jump = (x == 8'd0);
          3'd2: begin jump = (x != 8'd0); n_x = x - 8'd1; end
          3'd3: jump = (y == 8'd0);
          3'd4: begin jump = (y != 8'd0); n_y = y - 8'd1; end
          3'd5: jump = (x != y);
          3'd6: jump = pins_in[jmp_pin];
          3'd7: jump = (osr_count < pull_thresh);
        endcase
      end

      OP_WAIT: begin
        case (ops[6:5])
          2'd0:    wait_ok = (pins_in[ops[2:0]]  == ops[7]);   // absolute pin
          2'd1:    wait_ok = (pins_rel[ops[2:0]] == ops[7]);   // pin relative to in_base
          2'd2:    wait_ok = (irq_flags[ops[2:0]] == ops[7]);  // IRQ flag
          default: wait_ok = 1'b1;
        endcase
        if (!wait_ok)
          stall = 1'b1;
        else if (ops[6:5] == 2'd2 && ops[7])
          irq_clr[ops[2:0]] = 1'b1;                            // consume the IRQ
      end

      OP_IN: begin
        if (autopush && isr_cnt_after >= push_thresh) begin
          if (rx_full) begin
            stall = 1'b1;                    // nowhere to put the word yet
          end else begin
            rx_push     = 1'b1;
            rx_data     = isr_shifted;
            n_isr       = 8'd0;
            n_isr_count = 4'd0;
          end
        end else begin
          n_isr       = isr_shifted;
          n_isr_count = isr_cnt_after;
        end
      end

      OP_OUT: begin
        if (out_refill && tx_empty) begin
          stall = 1'b1;                      // autopull with nothing to pull
        end else begin
          tx_pop      = out_refill;
          n_osr       = osr_shifted;
          n_osr_count = sat_add8(osr_cnt_src, nbits);
          case (ops[7:5])
            3'd0: n_pin_val = write_pins(pin_val, out_base, out_n, out_data);
            3'd1: n_x = out_data;
            3'd2: n_y = out_data;
            3'd3: ;  // NULL: discard
            3'd4: n_pin_dir = write_pins(pin_dir, out_base, out_n, out_data);
            3'd5: begin jump = 1'b1; jump_target = out_data[4:0]; end
            3'd6: begin                      // ISR (8-bit: replace, count = n)
              n_isr       = out_data;
              n_isr_count = nbits;
            end
            default: ;
          endcase
        end
      end

      OP_PUSH: begin
        if (ops[7]) begin  // PULL
          if (ops[6] && osr_count < pull_thresh) begin
            // PULL IFEMPTY with OSR not yet empty enough: no-op
          end else if (!tx_empty) begin
            n_osr       = tx_data;
            n_osr_count = 4'd0;
            tx_pop      = 1'b1;
          end else if (ops[5]) begin
            stall = 1'b1;                    // blocking pull, FIFO empty
          end else begin
            n_osr       = x;                 // non-blocking pull from empty FIFO
            n_osr_count = 4'd0;
          end
        end else begin     // PUSH
          if (ops[6] && isr_count < push_thresh) begin
            // PUSH IFFULL with ISR not full enough: no-op
          end else if (rx_full && ops[5]) begin
            stall = 1'b1;                    // blocking push, FIFO full
          end else begin
            rx_push     = !rx_full;          // non-blocking push to a full FIFO drops the data
            n_isr       = 8'd0;
            n_isr_count = 4'd0;
          end
        end
      end

      OP_MOV: begin
        case (ops[7:5])
          3'd0: n_pin_val = write_pins(pin_val, out_base, out_count, mov_val);
          3'd1: n_x = mov_val;
          3'd2: n_y = mov_val;
          3'd3: n_pin_dir = write_pins(pin_dir, out_base, out_count, mov_val);
          3'd4: begin jump = 1'b1; jump_target = mov_val[4:0]; end
          3'd6: begin n_isr = mov_val; n_isr_count = 4'd0; end
          3'd7: begin n_osr = mov_val; n_osr_count = 4'd0; end
          default: ;
        endcase
      end

      OP_IRQ: begin
        if (ops[6]) begin
          irq_clr[ops[2:0]] = 1'b1;          // IRQ CLEAR
        end else if (!ops[5]) begin
          irq_set[ops[2:0]] = 1'b1;          // IRQ SET
        end else if (!irq_waiting) begin
          irq_set[ops[2:0]] = 1'b1;          // IRQ WAIT: raise the flag ...
          n_irq_waiting     = 1'b1;
          stall             = 1'b1;          // ... and wait for someone to clear it
        end else if (irq_flags[ops[2:0]]) begin
          stall = 1'b1;                      // still set: keep waiting
        end else begin
          n_irq_waiting = 1'b0;              // cleared: done
        end
      end

      OP_SET: begin
        case (ops[7:5])
          3'd0: n_pin_val = write_pins(pin_val, set_base, {1'b0, set_count}, {3'b0, ops[4:0]});
          3'd1: n_x = {3'b0, ops[4:0]};
          3'd2: n_y = {3'b0, ops[4:0]};
          3'd4: n_pin_dir = write_pins(pin_dir, set_base, {1'b0, set_count}, {3'b0, ops[4:0]});
          default: ;
        endcase
      end

      default: ;
    endcase

    // side-set wins over OUT/SET/MOV on the same pin
    if (side_en)
      n_pin_val = write_pins(n_pin_val, side_base, {2'b0, side_count}, {6'b0, side_val});

    // nothing leaves the SM unless this step really executes
    // (irq_set is deliberately not gated by stall: IRQ WAIT raises its flag
    //  in the same step that starts its wait)
    if (!run || restart || stall) begin
      tx_pop  = 1'b0;
      rx_push = 1'b0;
      irq_clr = 8'd0;
    end
    if (!run || restart) irq_set = 8'd0;
    if (exec_valid && stall) irq_set = 8'd0;   // a dropped EXEC has no side effects
  end

  // ---------------- state update ----------------
  always @(posedge clk) begin
    if (!rst_n) begin
      pc          <= 5'd0;
      x           <= 8'd0;
      y           <= 8'd0;
      osr         <= 8'd0;
      isr         <= 8'd0;
      osr_count   <= 4'd8;      // empty
      isr_count   <= 4'd0;
      delay_cnt   <= 5'd0;
      irq_waiting <= 1'b0;
      pin_val     <= 8'd0;
      pin_dir     <= 8'd0;      // all inputs after reset
      stalled     <= 1'b0;
    end else if (restart) begin
      pc          <= wrap_bottom;
      delay_cnt   <= 5'd0;
      irq_waiting <= 1'b0;
      stalled     <= 1'b0;
    end else if (run) begin
      pin_val <= n_pin_val;     // side-set applies even when stalled
      stalled <= stall && !exec_valid;
      if (!exec_valid) irq_waiting <= n_irq_waiting;
      if (!stall) begin
        x         <= n_x;
        y         <= n_y;
        osr       <= n_osr;
        isr       <= n_isr;
        osr_count <= n_osr_count;
        isr_count <= n_isr_count;
        pin_dir   <= n_pin_dir;
        if (exec_valid) begin
          if (jump) pc <= jump_target;
        end else begin
          delay_cnt <= delay_val;
          if (jump)                pc <= jump_target;
          else if (pc == wrap_top) pc <= wrap_bottom;
          else                     pc <= pc + 5'd1;
        end
      end
    end else if (en && tick && delay_cnt != 5'd0) begin
      delay_cnt <= delay_cnt - 5'd1;
    end
  end

endmodule
