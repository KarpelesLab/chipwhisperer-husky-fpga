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
    OLED_CS  = TIO4 = target_io4 -> oled_cs   (the 1k-coupled chip select)
    OLED_DC  = HS1  = target_hs1 -> oled_dc   (real D/C, routed back on HS1)

All four wires reach the Husky, so command vs. data is READ, not inferred:
D/C low = command, high = data (DC_DATA_LEVEL=1), sampled per byte and
required to agree across it (DC_SAMPLE=0 drops a byte whose D/C moved, which
on a clean wire never happens). An earlier build with no D/C inferred it
from the SSD1306 structure; the real wire removed that fragility.

Byte framing: CS (1k-coupled on TIO4) pulses high briefly between bytes and
is too marginal to be the byte AUTHORITY (FRAME_MODE=0 closed bytes at 7/9
bits), so spi_display_rx runs FREE-RUN (FRAME_MODE=2) counting 8 clean sclk
edges per byte, and the real CS gap only RE-ALIGNS the bit counter (its
rising edge pulses the rx reset). sclk is the direct, clean TIO1; CS only
re-phases.

spi_display_rx + ssd1306_slave are reticle's own proven IP (the same
real-time framebuffer slave that made the FPGA card-bench's `g` dump
pixel-perfect). Everything runs on clk_usb (96 MHz) — the register-bus
clock — so there is no CDC between the framebuffer BRAM and the readout
mux, and 96 MHz oversamples the ~2 MHz OLED SPI by ~48x.

Register map (clk_usb domain, big-OR readback like every other block)
---------------------------------------------------------------------
  DISP_STATUS      read, muxed by reg_bytecnt:
                     0: {overrun,dc_error,framing_error,framed,
                         inverse,charge_pump,all_on,display_on}
                     1: {2'b0, start_line[5:0]}   (snooped 0x40-0x7f)
                     2..3: frame_count   (16-bit LE, panel frames drawn)
                     4..5: bit_error_count  (spi_display_rx; want 0)
                     6..7: overrun_count
                     8..9: data_byte_count  (ssd1306_slave)
                    10..11: cmd_byte_count
                    12: col_ptr   13: page_ptr
                    14..15: rx_byte_count  (total bytes framed by CS)
  DISP_PAGESEL     write page 0..7 (which 128-byte page DISP_DATA returns)
  DISP_DATA        read 128 bytes = GDDRAM[page_sel*128 + reg_bytecnt]
  DISP_GAP_THRESH  read/write 16-bit INTER-FRAME gap threshold (clk cycles):
                   sclk idle this long resets the D/C inferer to command
                   state. Default 4096 (~43 us): safely above the ~115-clk
                   inter-byte gap and far below the ~1 s inter-frame gap.
  DISP_CONTROL     write bit0 = soft reset (one-cycle clear of rx/slave
                   framing + counters; BRAM is overwritten by the SE's
                   own screen-clear draw, so no explicit wipe is needed)
*************************************************************************/

module reg_sca_display #(
   parameter pBYTECNT_SIZE = 7
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
   input  wire         oled_cs,        // TIO4 / OLED_CS (active-low frame)
   input  wire         oled_dc         // HS1  / OLED_DC (real D/C: low=cmd, high=data)
);

   // ------------------------------------------------------------------
   // config / write side
   // ------------------------------------------------------------------
   reg [15:0] gap_thresh;              // inter-FRAME gap (clk) -> resync inferer
   reg [4:0]  page_sel;             // [4]=raw-mode, [3:0]=chunk; fb uses [2:0]
   reg        soft_rst;
   reg [5:0]  dc_delay;              // now: sclk/mosi/cs glitch-filter depth
   reg        use_real_dc;          // DISP_CONTROL[1]: tag cmd/data from real D/C
                                    // (robust to byte-count drift) vs structural

   always @(posedge clk_usb) begin
      if (reset) begin
         gap_thresh  <= 16'd4096;  // ~43 us @96MHz; >> 1.2us inter-byte gap,
                                   // << ~1 s inter-frame gap. Tunable at runtime.
         page_sel    <= 5'd0;
         soft_rst    <= 1'b0;
         dc_delay    <= 6'd4;       // sclk glitch-filter depth ON by default
                                    // (K~=4 @96MHz removes the CLK ringing that
                                    // added extra edges -> pixel-perfect bytes)
         use_real_dc <= 1'b1;       // real D/C tag ON (clean now the resistor's gone)
      end
      else begin
         soft_rst <= 1'b0;         // one-cycle pulse
         if (reg_write) begin
            case (reg_address)
               `DISP_GAP_THRESH: gap_thresh[reg_bytecnt*8 +: 8] <= reg_datai;
               `DISP_PAGESEL:    page_sel <= reg_datai[4:0];  // [4]=raw, [3:0]=chunk
               `DISP_CONTROL:    begin soft_rst    <= reg_datai[0];
                                       use_real_dc <= reg_datai[1]; end
               `DISP_DC_DELAY:   dc_delay <= reg_datai[5:0];
            endcase
         end
      end
   end

   wire sub_rst_n = ~(reset | soft_rst);

   // ------------------------------------------------------------------
   // synchronize the three async pins ONCE into clk_usb (2 FF), so the
   // front-end sees glitch-free signals. (Feeding spi_display_rx the raw
   // async sclk let a metastable first flop inject 1-cycle glitches ->
   // spurious edges -> margin_lost/overrun + per-byte bit errors. Sync
   // first.)  spi_display_rx re-synchronises internally too; a clean
   // input simply keeps its instrumentation honest.
   // ------------------------------------------------------------------
   reg [1:0] sclk_sync, mosi_sync, cs_sync, dc_sync;
   always @(posedge clk_usb) begin
      sclk_sync <= {sclk_sync[0], oled_sclk};
      mosi_sync <= {mosi_sync[0], oled_mosi};
      cs_sync   <= {cs_sync[0],   oled_cs};
      dc_sync   <= {dc_sync[0],   oled_dc};
   end
   wire sclk_clean = sclk_sync[1];
   wire mosi_clean = mosi_sync[1];
   wire cs_clean   = cs_sync[1];
   wire dc_clean   = dc_sync[1];

   // ------------------------------------------------------------------
   // One pipeline register on the synced pins (keeps them aligned). The old
   // dc_delay shift-register tap is gone: our own gap-reframed recovery samples
   // the real D/C at the 8th bit and needs no sclk/mosi delay, and `dc_delay`
   // (DISP_DC_DELAY) is now repurposed to tune the sclk glitch filter below.
   // ------------------------------------------------------------------
   reg        sclk_d, mosi_d, cs_d;
   always @(posedge clk_usb) begin
      sclk_d <= sclk_clean;
      mosi_d <= mosi_clean;
      cs_d   <= cs_clean;
   end

   // ------------------------------------------------------------------
   // Byte framing: count 8 clean sclk edges per byte (FRAME_MODE=2,
   // free-run). sclk is the direct, clean TIO1 and the SE always clocks
   // exactly 8 edges per byte, so a free-run counter that starts on a byte
   // boundary stays aligned across a whole 1041-byte frame. We RE-ALIGN it
   // only ONCE PER FRAME, in the ~1 s silent gap between frames -- a pulse
   // that resets the rx while NO byte is in flight. (An earlier build reset
   // the rx at every inter-byte CS gap; that occasionally clipped a byte,
   // and a dropped mode-set command (0x20) scrambled horizontal-addressed
   // screens like "Low battery". Per-frame re-align never clips.) PHASE_MARGIN=2
   // tolerates the edge jitter/ringing that overran at 8.
   //
   // CS (1k-coupled on TIO4) is too marginal to be the byte AUTHORITY
   // (FRAME_MODE=0 closed bytes at 7/9 bits); it is left connected for the
   // instrumentation but framing does not depend on it.
   // ------------------------------------------------------------------
   // Glitch filter on the synced sclk. The fabric input sees multiple threshold
   // crossings (ringing) on the slow ~1.6 MHz OLED sclk edges; a direct LA tap
   // and the 200 MHz SLogic16 see clean edges, but the FPGA input toggles, which
   // over-frames the bytes. Require the level to hold SCLK_FILT clk periods
   // before the filtered copy follows -> exactly one clean edge per real
   // transition. (Validated: the recovery algorithm on a clean 96 MHz capture
   // recovers the bytes exactly; only the fabric input needs this filter.) High/
   // low sclk times are ~20/38 clk, far above the filter, so no real edge is lost.
   // SCLK_FILT is runtime-tunable via the (otherwise unused) dc_delay register
   // (DISP_DC_DELAY), so the filter depth can be swept on hardware without a
   // rebuild. 0 = no filtering.
   wire [5:0] sclk_filt = dc_delay;
   reg [5:0] sclk_fcnt;
   reg       sclk_f;
   always @(posedge clk_usb) begin
      if (!sub_rst_n)                    begin sclk_f <= 1'b0; sclk_fcnt <= 6'd0; end
      else if (sclk_d == sclk_f)         sclk_fcnt <= 6'd0;
      else if (sclk_fcnt >= sclk_filt)   begin sclk_f <= sclk_d; sclk_fcnt <= 6'd0; end
      else                               sclk_fcnt <= sclk_fcnt + 6'd1;
   end
   // Matching debounce on MOSI so it stays aligned with the (filter-delayed)
   // sclk_f edge and its own ringing is rejected. Sampled as mosi_f at sclk_rise.
   reg [5:0] mosi_fcnt;
   reg       mosi_f;
   always @(posedge clk_usb) begin
      if (!sub_rst_n)                    begin mosi_f <= 1'b0; mosi_fcnt <= 6'd0; end
      else if (mosi_d == mosi_f)         mosi_fcnt <= 6'd0;
      else if (mosi_fcnt >= sclk_filt)   begin mosi_f <= mosi_d; mosi_fcnt <= 6'd0; end
      else                               mosi_fcnt <= mosi_fcnt + 6'd1;
   end
   // Debounce on CS (the 1k-coupled per-byte frame). CS reliably pulses once per
   // byte (bracketing exactly 8 sclk rises), so its FALLING edge is the byte
   // start -- a far more robust frame than the inter-byte idle gap, which dropped
   // 5-11 bytes/page. Debounced with the same runtime depth.
   reg [5:0] cs_fcnt;
   reg       cs_f;
   always @(posedge clk_usb) begin
      if (!sub_rst_n)                    begin cs_f <= 1'b1; cs_fcnt <= 6'd0; end
      else if (cs_d == cs_f)             cs_fcnt <= 6'd0;
      else if (cs_fcnt >= sclk_filt)     begin cs_f <= cs_d; cs_fcnt <= 6'd0; end
      else                               cs_fcnt <= cs_fcnt + 6'd1;
   end
   reg cs_f_q;
   always @(posedge clk_usb) cs_f_q <= cs_f;
   wire cs_fall = ~cs_f & cs_f_q;   // CS 1->0 : start of a byte

   reg sclk_d_q;
   always @(posedge clk_usb) sclk_d_q <= sclk_f;
   wire sclk_any_edge = sclk_f ^ sclk_d_q;

   // Idle gap tracking (clk since the last sclk edge). TWO thresholds:
   //   * inter-BYTE gap  (>= BYTE_GAP_CYCLES, ~1 us): the SE idles sclk / pulses
   //     CS between every byte, so this marks a byte boundary. We resync the rx
   //     bit counter here so the free-run 8-edges-per-byte framing can NEVER
   //     drift across a frame. PROVEN on real captures: plain free-run misframes
   //     (b6 c1 aa 41 ...), gap-reframe recovers the exact bytes (b6 d5 20 81
   //     42 a6 c0 a1 b0 ...). Robust for any threshold in ~60..128 clk.
   //   * inter-FRAME gap (>= gap_thresh, ~1 s): between whole screen redraws;
   //     resyncs the STRUCTURAL inferer (data_rem/arg_rem) so each frame starts
   //     in command state. The inferer must persist across byte gaps, so it is
   //     reset ONLY on the frame gap, never on the byte gap.
   localparam [15:0] BYTE_GAP_CYCLES = 16'd96;
   reg [15:0] idle_ctr;
   reg        frame_gap, frame_gap_q;
   reg        byte_gap,  byte_gap_q;
   always @(posedge clk_usb) begin
      if (!sub_rst_n) begin
         idle_ctr <= 16'd0; frame_gap <= 1'b1; byte_gap <= 1'b1;
      end
      else if (sclk_any_edge) begin
         idle_ctr <= 16'd0; frame_gap <= 1'b0; byte_gap <= 1'b0;
      end
      else begin
         if (idle_ctr != 16'hffff) idle_ctr <= idle_ctr + 16'd1;
         if (idle_ctr >= gap_thresh)       frame_gap <= 1'b1;
         if (idle_ctr >= BYTE_GAP_CYCLES)  byte_gap  <= 1'b1;
      end
   end
   always @(posedge clk_usb) begin
      frame_gap_q <= frame_gap;
      byte_gap_q  <= byte_gap;
   end
   wire frame_gap_enter = frame_gap & ~frame_gap_q;   // 0->1 : inter-frame silence
   wire byte_gap_enter  = byte_gap  & ~byte_gap_q;    // 0->1 : inter-byte gap

   // frame-level re-align (structural inferer): on the inter-FRAME gap only.
   reg [2:0] align_rst;
   always @(posedge clk_usb) begin
      if (reset | soft_rst)     align_rst <= 3'b111;
      else if (frame_gap_enter) align_rst <= 3'b111;
      else if (align_rst != 0)  align_rst <= align_rst - 3'b1;
   end

   // (byte-level re-align is applied directly in our own byte recovery below,
   // via byte_gap_enter resetting the bit counter -- no spi_display_rx reset.)

   wire [15:0] bit_errors, overruns, rx_bytes, dc_changes;
   wire        framed, framing_error, overrun, dc_err;

   // spi_display_rx kept for instrumentation only (bit_errors/overruns/framed);
   // its free-run framing drifts across a frame and its per-reset re-arming made
   // gap-reset unreliable, so the framebuffer uses OUR OWN recovery below.
   wire [7:0]  sdr_byte;
   wire        sdr_valid, sdr_is_data;
   spi_display_rx #(
      .SAMPLE_EDGE(0), .MSB_FIRST(1), .FRAME_MODE(2),
      .CS_ACTIVE_LOW(1), .CS_PULSE(0), .DC_SAMPLE(2),
      .DC_DATA_LEVEL(1), .PHASE_MARGIN(2), .SYNC_STAGES(2),
      .COUNT_WIDTH(16)
   ) u_rx (
      .clk(clk_usb), .rst_n(sub_rst_n),
      .sclk(sclk_d), .mosi(mosi_d), .dc(dc_clean), .cs_n(cs_d),
      .rx_byte(sdr_byte), .rx_is_data(sdr_is_data), .rx_valid(sdr_valid),
      .cmd_byte_count(), .data_byte_count(),
      .frame_count(rx_bytes), .bit_error_count(bit_errors),
      .dc_change_count(dc_changes), .overrun_count(overruns),
      .bit_count(), .last_bit_count(),
      .framed(framed), .framing_error(framing_error),
      .dc_error(dc_err), .overrun(overrun)
   );

   // ------------------------------------------------------------------
   // Byte recovery: GAP-REFRAME + emit-every-8. Shift MOSI in on each FILTERED
   // sclk rising edge (MSB first); emit a byte every 8 rises, and resync the bit
   // counter to a byte boundary on the inter-byte idle gap (byte_gap_enter). With
   // the sclk glitch filter ON (dc_delay~=4 removes the CLK ringing that added
   // extra edges) this recovers the bytes PIXEL-PERFECT -- validated bit-for-bit
   // against the 96 MHz wire capture (`60 d5 20 81 42 a6 c0 a1 b0 ...`). The gap
   // (not CS) frames: CS-window framing dropped held-CS bytes and CS ringing
   // doubled bytes; the gap is robust and emit-every-8 handles any run length.
   // real D/C (clean now the resistor's gone) is latched at the 8th bit.
   // ------------------------------------------------------------------
   reg        sclk_d_q2;
   always @(posedge clk_usb) sclk_d_q2 <= sclk_f;
   wire sclk_rise = sclk_f & ~sclk_d_q2;   // rising edge of the FILTERED sclk

   reg  [7:0] shreg;
   reg  [3:0] bitcnt;
   reg  [7:0] rx_byte;
   reg        rx_valid, rx_is_data;
   always @(posedge clk_usb) begin
      rx_valid <= 1'b0;                       // 1-cycle strobe
      if (!sub_rst_n) begin
         bitcnt <= 4'd0; shreg <= 8'd0;
      end
      else begin
         if (byte_gap_enter) bitcnt <= 4'd0;  // inter-byte gap -> resync boundary
         if (sclk_rise) begin
            shreg <= {shreg[6:0], mosi_f};
            if (bitcnt == 4'd7) begin
               rx_byte    <= {shreg[6:0], mosi_f};
               rx_is_data <= dc_clean;        // D/C at the 8th (last) bit
               rx_valid   <= 1'b1;
               bitcnt     <= 4'd0;
            end
            else bitcnt <= bitcnt + 4'd1;
         end
      end
   end

   // ------------------------------------------------------------------
   // A STRUCTURAL tag runs ALONGSIDE the real D/C -- not to replace it, but
   // to (a) gate the start-line snoop to true command opcodes (so argument
   // bytes like 0x60/0x42 don't masquerade as 0x40-0x7f start-line commands)
   // and (b) feed a DC-vs-structure MISMATCH counter that tells us, live,
   // whether dc_delay has the real D/C aligned to the bytes (mismatch -> 0).
   // The SE draws every screen in SSD1306 PAGE mode: a page-set (0xB0-0xB7)
   // is followed by 128 data bytes; multi-byte commands consume their args;
   // else a single command. Re-synced each frame in the silent gap.
   // ------------------------------------------------------------------
   reg [7:0] data_rem;
   reg [1:0] arg_rem;
   wire in_arg = (arg_rem != 2'd0);
   wire in_data = (data_rem != 8'd0);
   wire infer_is_data = !in_arg && in_data;            // combinational tag
   wire infer_is_cmd  = rx_valid && !in_arg && !in_data; // a true command opcode

   // Tag fed to the framebuffer: real D/C (robust to byte-count drift) when
   // use_real_dc, else the structural inference. real-cmd = valid byte with D/C low.
   wire fb_is_data = use_real_dc ? rx_is_data : infer_is_data;
   wire fb_is_cmd  = use_real_dc ? (rx_valid && !rx_is_data) : infer_is_cmd;

   // Resync on the SAME per-frame pulse as the rx realign (align_rst, active in
   // the silent inter-frame gap) rather than the frame_gap LEVEL -- so nothing
   // can clear data_rem mid-frame, and the inferer and the rx start each frame
   // together in command state on the first byte.
   always @(posedge clk_usb) begin
      if (!sub_rst_n || (align_rst != 3'b0)) begin
         data_rem <= 8'd0;
         arg_rem  <= 2'd0;
      end
      else if (rx_valid) begin
         if (in_arg)       arg_rem  <= arg_rem - 2'd1;
         else if (in_data) data_rem <= data_rem - 8'd1;
         else begin
            casez (rx_byte)
               8'h21, 8'h22:              arg_rem  <= 2'd2;
               8'h20, 8'h81, 8'h8d, 8'ha8,
               8'hd3, 8'hd5, 8'hd9, 8'hda,
               8'hdb:                     arg_rem  <= 2'd1;
               8'hb0, 8'hb1, 8'hb2, 8'hb3,
               8'hb4, 8'hb5, 8'hb6, 8'hb7: data_rem <= 8'd128;
               default: ;
            endcase
         end
      end
   end

   // DC-vs-structure mismatch: bytes the real D/C tagged differently from the
   // structural tag. With dc_delay aligned this is ~0; a high count means the
   // real D/C is still mis-phased. Saturating; cleared by soft reset.
   reg [15:0] dc_mismatch;
   always @(posedge clk_usb) begin
      if (!sub_rst_n)
         dc_mismatch <= 16'd0;
      else if (rx_valid && (rx_is_data != infer_is_data) && dc_mismatch != 16'hffff)
         dc_mismatch <= dc_mismatch + 16'd1;
   end

   // ------------------------------------------------------------------
   // Raw-byte ring: capture every recovered byte (rx_byte on rx_valid) into a
   // 2048-byte block RAM for host-side STRUCTURAL decode (oled_decode.py). The
   // in-fabric inferer desyncs across a frame (unfixable FSM bug); the host
   // infer_tags matched ground truth 0/1041 and ignores the real D/C entirely,
   // so host decode on the correctly-recovered byte stream is the proven path.
   //
   // Capture starts at index 0 on soft reset and FREEZES when full (2048 bytes
   // ~ 2 frames), so the host reads a stable, chronological window: soft reset,
   // wait for >=1 full frame, read DISP_RAWPTR (count + full), then 16 chunks of
   // 128 bytes via DISP_RAWSEL/DISP_RAW. Synchronous 1-cycle read, mirroring
   // ssd1306_slave so the register-bus read timing matches DISP_DATA.
   // ------------------------------------------------------------------
   (* ram_style = "block" *)
   reg  [7:0]  raw_buf [0:2047];
   reg  [10:0] raw_wptr;
   reg         raw_full;
   reg  [7:0]  raw_rd_q;
   always @(posedge clk_usb) begin
      if (!sub_rst_n) begin
         raw_wptr <= 11'd0;
         raw_full <= 1'b0;
      end
      else if (rx_valid && !raw_full) begin
         raw_buf[raw_wptr] <= rx_byte;
         if (raw_wptr == 11'd2047) raw_full <= 1'b1;
         else                      raw_wptr <= raw_wptr + 11'd1;
      end
      raw_rd_q <= raw_buf[{page_sel[3:0], reg_bytecnt[6:0]}]; // 1-cycle BRAM read
   end

   // ------------------------------------------------------------------
   // reticle IP: bytes -> framebuffer
   // ------------------------------------------------------------------
   wire [15:0] fb_rd_addr = {6'b0, page_sel[2:0], reg_bytecnt[6:0]}; // page*128+col
   wire [7:0]  fb_rd_data;
   wire [7:0]  col_ptr, page_ptr;
   wire        display_on, inverse, all_on, charge_pump, seg_remap, com_reverse;
   wire [15:0] panel_frames, panel_cmds, panel_datas;

   ssd1306_slave #(
      .COLUMNS(128), .PAGES(8), .COUNT_WIDTH(16), .BLOCK_RAM(1)
   ) u_panel (
      .clk(clk_usb), .rst_n(sub_rst_n),
      .in_byte(rx_byte), .in_is_data(fb_is_data), .in_valid(rx_valid), // real-DC or structural
      .rd_addr(fb_rd_addr), .rd_data(fb_rd_data),
      .frame_done(),
      .display_on(display_on), .inverse(inverse), .all_on(all_on),
      .contrast(), .addr_mode(), .charge_pump(charge_pump),
      .seg_remap(seg_remap), .com_reverse(com_reverse),
      .col_ptr(col_ptr), .page_ptr(page_ptr), .pending_args(),
      .cmd_count(panel_cmds), .data_count(panel_datas),
      .frame_count(panel_frames), .unknown_cmds()
   );

   // ------------------------------------------------------------------
   // snoop the display start line (0x40-0x7f) -- ONLY at a true command
   // opcode (infer_is_cmd), never on an argument byte (e.g. 0x60 after db,
   // 0x42 after 81), which previously set start_line to garbage.
   // ------------------------------------------------------------------
   reg [5:0] start_line;
   always @(posedge clk_usb) begin
      if (reset | soft_rst)
         start_line <= 6'd0;
      else if (fb_is_cmd && (rx_byte[7:6] == 2'b01))
         start_line <= rx_byte[5:0];
   end

   // ------------------------------------------------------------------
   // read side (combinational; 0 unless addressed and reading)
   // ------------------------------------------------------------------
   reg [7:0] status_byte;
   always @(*) begin
      case (reg_bytecnt)
          0: status_byte = {overrun, dc_err, framing_error, framed,
                            inverse, charge_pump, all_on, display_on};
          1: status_byte = {com_reverse, seg_remap, start_line};
          2: status_byte = panel_frames[7:0];
          3: status_byte = panel_frames[15:8];
          4: status_byte = bit_errors[7:0];
          5: status_byte = bit_errors[15:8];
          6: status_byte = overruns[7:0];
          7: status_byte = overruns[15:8];
          8: status_byte = panel_datas[7:0];
          9: status_byte = panel_datas[15:8];
         10: status_byte = panel_cmds[7:0];
         11: status_byte = panel_cmds[15:8];
         12: status_byte = col_ptr;
         13: status_byte = page_ptr;
         14: status_byte = dc_mismatch[7:0];
         15: status_byte = dc_mismatch[15:8];
         // raw-ring capture pointer (host reads these to know how many bytes
         // were captured before the ring froze): count low, then {full, hi}.
         16: status_byte = raw_wptr[7:0];
         17: status_byte = {raw_full, 4'b0, raw_wptr[10:8]};
         default: status_byte = 8'd0;
      endcase
   end

   always @(*) begin
      if (reg_read) begin
         case (reg_address)
            // DISP_DATA returns the raw ring in raw-mode (page_sel[4]=1), else
            // the decoded framebuffer. Both reads are 1-cycle registered, so the
            // register-bus timing is identical.
            `DISP_DATA:       reg_datao = page_sel[4] ? raw_rd_q : fb_rd_data;
            `DISP_STATUS:     reg_datao = status_byte;
            `DISP_PAGESEL:    reg_datao = {3'b0, page_sel};
            `DISP_GAP_THRESH: reg_datao = gap_thresh[reg_bytecnt*8 +: 8];
            `DISP_DC_DELAY:   reg_datao = {2'b0, dc_delay};
            default:          reg_datao = 8'd0;
         endcase
      end
      else
         reg_datao = 8'd0;
   end

endmodule
`default_nettype wire
