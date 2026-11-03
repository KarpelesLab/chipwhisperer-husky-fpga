// -----------------------------------------------------------------------------
// Copyright (c) 2026 Karpelès Lab Inc.   SPDX-License-Identifier: MIT
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND. See the MIT
// License text for the full disclaimer.
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps
`default_nettype none
`include "includes.v"

/***********************************************************************
reg_sca_display — a live OLED framebuffer mirror for the ChipWhisperer
Husky, added to NewAE's cwhusky_top for the Ledger Nano X SE bench.

The SE (socketed on a CW308T-QFN20-SOCKET) is the SPI master of its
SSD1306/SSD1312 128x64 panel. Three of the four display wires reach the
Husky on the target IO pins:

    OLED_CLK = TIO1 = target_io1 -> oled_sclk
    OLED_DIN = TIO2 = target_io2 -> oled_mosi
    OLED_DC  = TIO4 = target_io4 -> oled_dc

OLED_CS (socket pin 9) is NOT routed to the Husky (pulled to VCC on the
socket board), so we have no chip-select. We recover the byte frame from
the idle gap between sclk bursts instead: the SE clocks exactly eight
edges per byte with a gap between bytes, and spi_display_rx frames a byte
on cs_n. We synthesize cs_n from sclk activity:

    cs_syn_n = 0  while sclk has toggled within the last GAP_THRESH clocks
    cs_syn_n = 1  (a "gap") once sclk has been idle that long

spi_display_rx + ssd1306_slave are reticle's own proven IP (the same
real-time framebuffer slave that made the FPGA card-bench's `g` dump
pixel-perfect). Everything runs on clk_usb (96 MHz) — the register-bus
clock — so there is no CDC between the framebuffer BRAM and the readout
mux, and 96 MHz oversamples the ~1 MHz OLED SPI by ~96x.

Framing race / why the SPI bus is delayed
-----------------------------------------
cs_syn_n is derived *reactively* from sclk, so it lags the first edge of
a burst by the synchronizer depth. If spi_display_rx saw that first edge
before frame_open (cs going active) propagated, it would drop the first
bit of every byte. We therefore pipeline-delay the sampled bus
(sclk/mosi/dc) by SPI_DELAY clocks while deriving cs_syn_n from the
undelayed sclk, so frame_open always leads the first sampled edge. The
delay (~60 ns) is negligible against the ~500 ns sclk half-period, and
GAP_THRESH >> SPI_DELAY keeps the trailing gap from closing early.

Register map (clk_usb domain, big-OR readback like every other block)
---------------------------------------------------------------------
  DISP_STATUS      read, muxed by reg_bytecnt:
                     0: {overrun,dc_error,framing_error,framed,
                         inverse,charge_pump,all_on,display_on}
                     1: {2'b0, start_line[5:0]}   (snooped 0x40-0x7f)
                     2..3: frame_count   (16-bit LE, panel frames drawn)
                     4..5: bit_error_count
                     6..7: overrun_count
                     8..9: data_byte_count
                    10..11: cmd_byte_count
                    12: col_ptr   13: page_ptr
                    14..15: dc_change_count
  DISP_PAGESEL     write page 0..7 (which 128-byte page DISP_DATA returns)
  DISP_DATA        read 128 bytes = GDDRAM[page_sel*128 + reg_bytecnt]
  DISP_GAP_THRESH  read/write 16-bit cs-synth gap threshold (clk cycles)
  DISP_CONTROL     write bit0 = soft reset (one-cycle clear of rx/slave
                   framing + counters; BRAM is overwritten by the SE's
                   own screen-clear draw, so no explicit wipe is needed)
*************************************************************************/

module reg_sca_display #(
   parameter pBYTECNT_SIZE = 7,
   // cs-synth race margin; must exceed (my 2-FF + rx 2-FF + a cycle).
   parameter SPI_DELAY     = 6
)(
   input  wire         reset,          // active-high, synchronous (NewAE style)
   input  wire         clk_usb,
   input  wire [7:0]   reg_address,
   input  wire [pBYTECNT_SIZE-1:0] reg_bytecnt,
   input  wire [7:0]   reg_datai,      // data to write
   output reg  [7:0]   reg_datao,      // data to read (0 unless addressed+reading)
   input  wire         reg_read,
   input  wire         reg_write,

   // OLED SPI, tapped from the target IO pins (async to clk_usb)
   input  wire         oled_sclk,      // TIO1 / OLED_CLK
   input  wire         oled_mosi,      // TIO2 / OLED_DIN
   input  wire         oled_dc         // TIO4 / OLED_DC
);

   // ------------------------------------------------------------------
   // config / write side
   // ------------------------------------------------------------------
   reg [15:0] gap_thresh;
   reg [2:0]  page_sel;
   reg        soft_rst;

   always @(posedge clk_usb) begin
      if (reset) begin
         gap_thresh <= 16'd96;     // ~1 us idle at 96 MHz ends a byte
         page_sel   <= 3'd0;
         soft_rst   <= 1'b0;
      end
      else begin
         soft_rst <= 1'b0;         // one-cycle pulse
         if (reg_write) begin
            case (reg_address)
               `DISP_GAP_THRESH: gap_thresh[reg_bytecnt*8 +: 8] <= reg_datai;
               `DISP_PAGESEL:    page_sel <= reg_datai[2:0];
               `DISP_CONTROL:    soft_rst <= reg_datai[0];
            endcase
         end
      end
   end

   wire sub_rst_n = ~(reset | soft_rst);

   // ------------------------------------------------------------------
   // synthesize cs_n from the sclk gap (undelayed sclk -> fast reaction)
   // ------------------------------------------------------------------
   reg [1:0] sclk_sync;
   always @(posedge clk_usb) sclk_sync <= {sclk_sync[0], oled_sclk};
   reg sclk_sync_q;
   always @(posedge clk_usb) sclk_sync_q <= sclk_sync[1];
   wire sclk_any_edge = sclk_sync[1] ^ sclk_sync_q;

   reg [15:0] gap_ctr;
   reg        cs_syn_n;
   always @(posedge clk_usb) begin
      if (reset) begin
         gap_ctr  <= 16'd0;
         cs_syn_n <= 1'b1;         // start in a gap (between bytes)
      end
      else if (sclk_any_edge) begin
         gap_ctr  <= 16'd0;
         cs_syn_n <= 1'b0;         // burst in progress
      end
      else if (gap_ctr >= gap_thresh) begin
         cs_syn_n <= 1'b1;         // long idle -> gap
      end
      else begin
         gap_ctr  <= gap_ctr + 16'd1;
      end
   end

   // ------------------------------------------------------------------
   // delay the sampled bus so frame_open leads the first sampled edge
   // ------------------------------------------------------------------
   reg [SPI_DELAY-1:0] sclk_dly, mosi_dly, dc_dly;
   always @(posedge clk_usb) begin
      sclk_dly <= {sclk_dly[SPI_DELAY-2:0], oled_sclk};
      mosi_dly <= {mosi_dly[SPI_DELAY-2:0], oled_mosi};
      dc_dly   <= {dc_dly  [SPI_DELAY-2:0], oled_dc};
   end
   wire sclk_d = sclk_dly[SPI_DELAY-1];
   wire mosi_d = mosi_dly[SPI_DELAY-1];
   wire dc_d   = dc_dly  [SPI_DELAY-1];

   // ------------------------------------------------------------------
   // reticle IP: wire -> bytes -> framebuffer
   // ------------------------------------------------------------------
   wire [7:0]  rx_byte;
   wire        rx_is_data, rx_valid;
   wire [15:0] cmd_bytes, data_bytes, bit_errors, dc_changes, overruns;
   wire        framed, framing_error, dc_error, overrun;

   spi_display_rx #(
      .SAMPLE_EDGE(0), .MSB_FIRST(1), .FRAME_MODE(0),
      .CS_ACTIVE_LOW(1), .CS_PULSE(0), .DC_SAMPLE(0),
      .DC_DATA_LEVEL(1), .PHASE_MARGIN(2), .SYNC_STAGES(2),
      .COUNT_WIDTH(16)
   ) u_rx (
      .clk(clk_usb), .rst_n(sub_rst_n),
      .sclk(sclk_d), .mosi(mosi_d), .dc(dc_d), .cs_n(cs_syn_n),
      .rx_byte(rx_byte), .rx_is_data(rx_is_data), .rx_valid(rx_valid),
      .cmd_byte_count(cmd_bytes), .data_byte_count(data_bytes),
      .frame_count(), .bit_error_count(bit_errors),
      .dc_change_count(dc_changes), .overrun_count(overruns),
      .bit_count(), .last_bit_count(),
      .framed(framed), .framing_error(framing_error),
      .dc_error(dc_error), .overrun(overrun)
   );

   wire [15:0] fb_rd_addr = {3'b0, page_sel, reg_bytecnt[6:0]}; // page*128+col
   wire [7:0]  fb_rd_data;
   wire [7:0]  col_ptr, page_ptr;
   wire        display_on, inverse, all_on, charge_pump;
   wire [15:0] panel_frames;

   ssd1306_slave #(
      .COLUMNS(128), .PAGES(8), .COUNT_WIDTH(16), .BLOCK_RAM(1)
   ) u_panel (
      .clk(clk_usb), .rst_n(sub_rst_n),
      .in_byte(rx_byte), .in_is_data(rx_is_data), .in_valid(rx_valid),
      .rd_addr(fb_rd_addr), .rd_data(fb_rd_data),
      .frame_done(),
      .display_on(display_on), .inverse(inverse), .all_on(all_on),
      .contrast(), .addr_mode(), .charge_pump(charge_pump),
      .seg_remap(), .com_reverse(),
      .col_ptr(col_ptr), .page_ptr(page_ptr), .pending_args(),
      .cmd_count(), .data_count(), .frame_count(panel_frames),
      .unknown_cmds()
   );

   // ------------------------------------------------------------------
   // snoop the display start line (0x40-0x7f) for BOLOS scroll handling
   // ------------------------------------------------------------------
   reg [5:0] start_line;
   always @(posedge clk_usb) begin
      if (reset | soft_rst)
         start_line <= 6'd0;
      else if (rx_valid && !rx_is_data && (rx_byte[7:6] == 2'b01))
         start_line <= rx_byte[5:0];
   end

   // ------------------------------------------------------------------
   // read side (combinational; 0 unless addressed and reading)
   // ------------------------------------------------------------------
   reg [7:0] status_byte;
   always @(*) begin
      case (reg_bytecnt)
          0: status_byte = {overrun, dc_error, framing_error, framed,
                            inverse, charge_pump, all_on, display_on};
          1: status_byte = {2'b0, start_line};
          2: status_byte = panel_frames[7:0];
          3: status_byte = panel_frames[15:8];
          4: status_byte = bit_errors[7:0];
          5: status_byte = bit_errors[15:8];
          6: status_byte = overruns[7:0];
          7: status_byte = overruns[15:8];
          8: status_byte = data_bytes[7:0];
          9: status_byte = data_bytes[15:8];
         10: status_byte = cmd_bytes[7:0];
         11: status_byte = cmd_bytes[15:8];
         12: status_byte = col_ptr;
         13: status_byte = page_ptr;
         14: status_byte = dc_changes[7:0];
         15: status_byte = dc_changes[15:8];
         default: status_byte = 8'd0;
      endcase
   end

   always @(*) begin
      if (reg_read) begin
         case (reg_address)
            `DISP_DATA:       reg_datao = fb_rd_data;
            `DISP_STATUS:     reg_datao = status_byte;
            `DISP_PAGESEL:    reg_datao = {5'b0, page_sel};
            `DISP_GAP_THRESH: reg_datao = gap_thresh[reg_bytecnt*8 +: 8];
            default:          reg_datao = 8'd0;
         endcase
      end
      else
         reg_datao = 8'd0;
   end

endmodule
`default_nettype wire
