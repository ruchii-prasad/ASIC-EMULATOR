/*
 * Copyright (c) 2026 Ruchi Prasad
 * SPDX-License-Identifier: Apache-2.0
 *
 * host_spi: SPI target (mode 0, MSB first) for the host link.
 *
 * SCK, CS_n and MOSI come from the host's clock domain, so they are passed
 * through 2-flop synchronizers and SCK edges are detected in the `clk` domain.
 * That limits the host SPI clock to about clk / 8 (6.25 MHz at 50 MHz).
 *
 * Byte-level interface to the command decoder:
 *   cs_start    pulse when CS_n goes low (a new transaction starts)
 *   byte_start  pulse on the first SCK rising edge of each byte
 *   byte_valid  pulse when 8 bits have arrived; byte_data holds them
 *   resp_load   the decoder supplies the byte to send back during the *next*
 *               byte. If nothing is loaded, MISO sends 0x00.
 */

`default_nettype none

module host_spi (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       sck_in,
    input  wire       csn_in,
    input  wire       mosi_in,
    output wire       miso,
    output reg        cs_start,
    output reg        byte_start,
    output reg        byte_valid,
    output reg  [7:0] byte_data,
    input  wire       resp_load,
    input  wire [7:0] resp
);

  // synchronizers: [0] and [1] synchronize, [2] remembers the previous value
  reg [2:0] sck_s, csn_s;
  reg [1:0] mosi_s;
  always @(posedge clk) begin
    if (!rst_n) begin
      sck_s  <= 3'b000;
      csn_s  <= 3'b111;
      mosi_s <= 2'b00;
    end else begin
      sck_s  <= {sck_s[1:0], sck_in};
      csn_s  <= {csn_s[1:0], csn_in};
      mosi_s <= {mosi_s[0], mosi_in};
    end
  end

  wire sck_rise = sck_s[1] & ~sck_s[2];
  wire sck_fall = ~sck_s[1] & sck_s[2];
  wire csn      = csn_s[1];
  wire cs_fall  = ~csn_s[1] & csn_s[2];
  wire mosi     = mosi_s[1];

  reg [2:0] bit_cnt;
  reg [6:0] sh_in;
  reg [7:0] sh_out, next_out;
  reg       have_next;

  assign miso = sh_out[7] & ~csn;

  always @(posedge clk) begin
    if (!rst_n) begin
      bit_cnt    <= 3'd0;
      sh_in      <= 7'd0;
      sh_out     <= 8'd0;
      next_out   <= 8'd0;
      have_next  <= 1'b0;
      cs_start   <= 1'b0;
      byte_start <= 1'b0;
      byte_valid <= 1'b0;
      byte_data  <= 8'd0;
    end else begin
      cs_start   <= 1'b0;
      byte_start <= 1'b0;
      byte_valid <= 1'b0;

      if (csn) begin
        bit_cnt   <= 3'd0;
        sh_out    <= 8'd0;
        have_next <= 1'b0;
      end else begin
        if (cs_fall) cs_start <= 1'b1;

        if (sck_rise) begin                    // sample MOSI
          if (bit_cnt == 3'd0) byte_start <= 1'b1;
          sh_in   <= {sh_in[5:0], mosi};
          bit_cnt <= bit_cnt + 3'd1;
          if (bit_cnt == 3'd7) begin
            byte_valid <= 1'b1;
            byte_data  <= {sh_in, mosi};
          end
        end

        if (sck_fall) begin                    // shift MISO
          if (bit_cnt == 3'd0) begin           // byte boundary: start the response byte
            sh_out    <= have_next ? next_out : 8'h00;
            have_next <= 1'b0;
          end else begin
            sh_out <= {sh_out[6:0], 1'b0};
          end
        end
      end

      if (resp_load) begin                     // after the clear above, so it wins
        next_out  <= resp;
        have_next <= 1'b1;
      end
    end
  end

endmodule
