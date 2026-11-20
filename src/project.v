/*
 * Copyright (c) 2026 Ruchi Prasad
 * SPDX-License-Identifier: Apache-2.0
 *
 * Top level of the programmable protocol emulator (docs/isa.md).
 *
 *   host (RP2350) --SPI on ui_in/uo_out--> host_spi --> command decoder
 *                                                          |
 *                     shared instruction memory (32 x 16) + per-SM config,
 *                     TX/RX FIFOs, enables, IRQ flags
 *                                                          |
 *                            N_SM state machines --> pin merge + open-drain --> uio[7:0]
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

  localparam N_SM  = 2;   // number of state machines (status byte supports up to 4)
  localparam N_CFG = 9;   // per-SM config registers 0..8 (register 9, ODMASK, is shared)

  integer i;

  // ======================= host link =======================
  wire       cs_start, byte_start, byte_valid, miso;
  wire [7:0] byte_data;
  reg        resp_load;
  reg  [7:0] resp;

  host_spi u_host (
      .clk(clk), .rst_n(rst_n),
      .sck_in(ui_in[0]), .csn_in(ui_in[1]), .mosi_in(ui_in[2]), .miso(miso),
      .cs_start(cs_start), .byte_start(byte_start),
      .byte_valid(byte_valid), .byte_data(byte_data),
      .resp_load(resp_load), .resp(resp));

  // ======================= shared state =======================
  reg [15:0] imem [0:31];                 // not reset: the host loads it after every reset
  reg [7:0]  cfg  [0:N_SM*N_CFG-1];
  reg [7:0]  odmask;
  reg [N_SM-1:0] sm_en;
  reg [7:0]  irq_flags;

  // per-SM wires
  wire [4:0]  sm_pc      [0:N_SM-1];
  wire [7:0]  sm_pin_val [0:N_SM-1];
  wire [7:0]  sm_pin_dir [0:N_SM-1];
  wire [7:0]  sm_irq_set [0:N_SM-1];
  wire [7:0]  sm_irq_clr [0:N_SM-1];
  wire [7:0]  rx_head    [0:N_SM-1];
  wire [N_SM-1:0] sm_stalled, tx_full, rx_empty;

  // pulses from the command decoder
  reg  [N_SM-1:0] tx_push, rx_pop, exec_valid, restart;
  reg  [7:0]      host_wdata;              // data for TX_PUSH
  reg  [15:0]     exec_instr;
  reg  [7:0]      host_irq_clr;

  // ======================= command decoder =======================
  reg        first;        // next byte is the command byte
  reg  [7:0] cmd;
  reg  [1:0] phase;
  reg  [4:0] ptr;          // IMEM address or config register number
  reg  [7:0] hi;
  reg        pend_pop;     // a peeked RX byte is being sent; pop it when its byte starts

  wire [3:0] c_hi = byte_data[7:4];        // used when byte_data is the command byte
  wire [1:0] c_sm = byte_data[1:0];
  wire [1:0] sm_sel = cmd[1:0];            // SM selected by the current command

  // status byte: {rx_nonempty[3:0], tx_full[3:0]}
  wire [3:0] rx_ne4   = {{(4-N_SM){1'b0}}, ~rx_empty};
  wire [3:0] tx_full4 = {{(4-N_SM){1'b0}}, tx_full};

  always @(posedge clk) begin
    if (!rst_n) begin
      first        <= 1'b1;
      cmd          <= 8'd0;
      phase        <= 2'd0;
      ptr          <= 5'd0;
      hi           <= 8'd0;
      pend_pop     <= 1'b0;
      odmask       <= 8'd0;
      sm_en        <= {N_SM{1'b0}};
      tx_push      <= {N_SM{1'b0}};
      rx_pop       <= {N_SM{1'b0}};
      exec_valid   <= {N_SM{1'b0}};
      restart      <= {N_SM{1'b0}};
      host_wdata   <= 8'd0;
      exec_instr   <= 16'd0;
      host_irq_clr <= 8'd0;
      resp_load    <= 1'b0;
      resp         <= 8'd0;
      for (i = 0; i < N_SM*N_CFG; i = i + 1) cfg[i] <= 8'd0;
    end else begin
      // one-cycle pulses default to 0
      tx_push      <= {N_SM{1'b0}};
      rx_pop       <= {N_SM{1'b0}};
      exec_valid   <= {N_SM{1'b0}};
      restart      <= {N_SM{1'b0}};
      host_irq_clr <= 8'd0;
      resp_load    <= 1'b0;

      if (cs_start) begin
        first    <= 1'b1;
        pend_pop <= 1'b0;
      end

      if (byte_start && pend_pop) begin       // host is now clocking out the peeked byte
        rx_pop[sm_sel] <= 1'b1;
        pend_pop       <= 1'b0;
      end

      if (byte_valid) begin
        if (first) begin
          // ---------- command byte ----------
          first <= 1'b0;
          cmd   <= byte_data;
          phase <= 2'd0;
          case (c_hi)
            4'h4: if (c_sm < N_SM) begin                       // RX_POP: send head
              resp      <= rx_empty[c_sm] ? 8'h00 : rx_head[c_sm];
              resp_load <= 1'b1;
              pend_pop  <= ~rx_empty[c_sm];
            end
            4'h5: begin                                        // STATUS
              resp      <= {rx_ne4, tx_full4};
              resp_load <= 1'b1;
            end
            4'h6: begin                                        // ENABLE
              sm_en   <= byte_data[N_SM-1:0];
              restart <= byte_data[N_SM-1:0] & ~sm_en;         // newly enabled SMs restart
            end
            default: ;
          endcase
        end else begin
          // ---------- payload bytes ----------
          case (cmd[7:4])
            4'h1: begin                                        // WRITE_IMEM
              if (phase == 2'd0) begin
                ptr   <= byte_data[4:0];
                phase <= 2'd1;
              end else if (phase == 2'd1) begin
                hi    <= byte_data;
                phase <= 2'd2;
              end else begin
                imem[ptr] <= {hi, byte_data};
                ptr       <= ptr + 5'd1;
                phase     <= 2'd1;
              end
            end
            4'h2: begin                                        // WRITE_CFG
              if (phase == 2'd0) begin
                ptr   <= byte_data[4:0];
                phase <= 2'd1;
              end else begin
                if (ptr < N_CFG && sm_sel < N_SM) cfg[sm_sel*N_CFG + ptr] <= byte_data;
                else if (ptr == N_CFG)            odmask <= byte_data;
                ptr <= ptr + 5'd1;
              end
            end
            4'h3: if (sm_sel < N_SM) begin                     // TX_PUSH
              tx_push[sm_sel] <= 1'b1;
              host_wdata      <= byte_data;
            end
            4'h4: if (sm_sel < N_SM) begin                     // RX_POP: send the next head
              // the previous byte's pop happened at its byte_start, long before now
              resp      <= rx_empty[sm_sel] ? 8'h00 : rx_head[sm_sel];
              resp_load <= 1'b1;
              pend_pop  <= ~rx_empty[sm_sel];
            end
            4'h7: host_irq_clr <= byte_data;                   // IRQ_CLEAR
            4'h8: begin                                        // EXEC
              if (phase == 2'd0) begin
                hi    <= byte_data;
                phase <= 2'd1;
              end else begin
                if (sm_sel < N_SM) exec_valid[sm_sel] <= 1'b1;
                exec_instr <= {hi, byte_data};
                phase      <= 2'd0;
              end
            end
            default: ;
          endcase
        end
      end
    end
  end

  // ======================= IRQ flags =======================
  reg [7:0] irq_set_all, irq_clr_all;
  always @(*) begin
    irq_set_all = 8'd0;
    irq_clr_all = host_irq_clr;
    for (i = 0; i < N_SM; i = i + 1) begin
      irq_set_all = irq_set_all | sm_irq_set[i];
      irq_clr_all = irq_clr_all | sm_irq_clr[i];
    end
  end
  always @(posedge clk) begin
    if (!rst_n) irq_flags <= 8'd0;
    else        irq_flags <= (irq_flags | irq_set_all) & ~irq_clr_all;
  end

  // ======================= state machines =======================
  // threshold field encoding: 0 = 8, 1 = 1, 2 = 4, 3 = 7
  function [3:0] thresh;
    input [1:0] code;
    begin
      case (code)
        2'd1:    thresh = 4'd1;
        2'd2:    thresh = 4'd4;
        2'd3:    thresh = 4'd7;
        default: thresh = 4'd8;
      endcase
    end
  endfunction

  genvar s;
  generate
    for (s = 0; s < N_SM; s = s + 1) begin : g_sm
      wire [7:0] r_div_lo = cfg[s*N_CFG + 0];
      wire [7:0] r_div_hi = cfg[s*N_CFG + 1];
      wire [7:0] r_exec_l = cfg[s*N_CFG + 2];
      wire [7:0] r_exec_h = cfg[s*N_CFG + 3];
      wire [7:0] r_shift  = cfg[s*N_CFG + 4];
      wire [7:0] r_pinout = cfg[s*N_CFG + 5];
      wire [7:0] r_pinset = cfg[s*N_CFG + 6];
      wire [7:0] r_pinin  = cfg[s*N_CFG + 7];
      wire [7:0] r_jmppin = cfg[s*N_CFG + 8];

      wire tick;
      clk_div #(.W(16)) u_div (
          .clk(clk), .rst_n(rst_n), .en(sm_en[s]), .div({r_div_hi, r_div_lo}), .tick(tick));

      wire       tx_empty, tx_pop, rx_full, rx_push;
      wire [7:0] tx_data, rx_data;

      fifo #(.W(8), .DEPTH(4), .AW(2)) u_txf (
          .clk(clk), .rst_n(rst_n), .push(tx_push[s]), .din(host_wdata), .pop(tx_pop),
          .dout(tx_data), .empty(tx_empty), .full(tx_full[s]), .count());

      fifo #(.W(8), .DEPTH(4), .AW(2)) u_rxf (
          .clk(clk), .rst_n(rst_n), .push(rx_push), .din(rx_data), .pop(rx_pop[s]),
          .dout(rx_head[s]), .empty(rx_empty[s]), .full(rx_full), .count());

      sm u_sm (
          .clk(clk), .rst_n(rst_n), .en(sm_en[s]), .tick(tick), .restart(restart[s]),
          .pc_o(sm_pc[s]), .imem_data(imem[sm_pc[s]]),
          .exec_valid(exec_valid[s]), .exec_instr(exec_instr),
          .wrap_bottom(r_exec_l[4:0]), .wrap_top(r_exec_h[4:0]),
          .side_opt(r_exec_h[5]), .side_count(r_exec_h[7:6]),
          .out_shift_right(r_shift[0]), .in_shift_right(r_shift[1]),
          .autopull(r_shift[2]), .autopush(r_shift[3]),
          .pull_thresh(thresh(r_shift[5:4])), .push_thresh(thresh(r_shift[7:6])),
          .out_base(r_pinout[2:0]), .out_count(r_pinout[6:3]),
          .set_base(r_pinset[2:0]), .set_count(r_pinset[5:3]),
          .in_base(r_pinin[2:0]), .side_base(r_pinin[5:3]), .jmp_pin(r_jmppin[2:0]),
          .pins_in(uio_in), .pin_val(sm_pin_val[s]), .pin_dir(sm_pin_dir[s]),
          .tx_empty(tx_empty), .tx_data(tx_data), .tx_pop(tx_pop),
          .rx_full(rx_full), .rx_push(rx_push), .rx_data(rx_data),
          .irq_flags(irq_flags), .irq_set(sm_irq_set[s]), .irq_clr(sm_irq_clr[s]),
          .stalled(sm_stalled[s]));
    end
  endgenerate

  // ======================= pin merge + open-drain =======================
  // A pin is an output if any SM sets its direction bit. If several SMs own the
  // same pin, the highest-numbered SM supplies the value.
  // Open-drain pins (ODMASK = 1) never drive high: value 0 pulls low, value 1 releases.
  reg [7:0] pin_val, pin_dir;
  always @(*) begin
    pin_val = 8'd0;
    pin_dir = 8'd0;
    for (i = 0; i < N_SM; i = i + 1) begin
      pin_val = (pin_val & ~sm_pin_dir[i]) | (sm_pin_val[i] & sm_pin_dir[i]);
      pin_dir = pin_dir | sm_pin_dir[i];
    end
  end

  assign uio_out = pin_val & ~odmask;
  assign uio_oe  = pin_dir & ~(odmask & pin_val);

  // ======================= host-visible outputs =======================
  assign uo_out = {sm_stalled[0], sm_pc[0], |irq_flags, miso};

  wire _unused = &{ena, ui_in[7:3], 1'b0};

endmodule
