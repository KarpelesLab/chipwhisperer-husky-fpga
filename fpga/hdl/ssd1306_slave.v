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
// Vendored from reticle: ip/video/ssd1306_slave/rtl/ssd1306_slave.v
// Source commit: b8133da  copied 2026-11-02T17:32Z
// DO NOT EDIT HERE — edit in reticle and re-vendor. Used by the Husky OLED framebuffer build.
// An SSD1306-compatible mono display, as a slave: it decodes the command
// set, keeps the display memory, and lets something else read it out.
//
// What it is for: a part that pretends to be a 128x64 OLED so that a
// driver written for one can be developed, watched and tested without the
// glass — or so that the pixels a real driver would have drawn can be
// captured and sent somewhere else. The frame buffer is readable, and a
// strobe says when a whole frame has been written, so a host can take a
// copy after each redraw.
//
// What it is NOT: a controller for a real panel, and not a transport. It
// takes **bytes with a command/data tag**, not wires, so the link is
// somebody else's problem:
//
//   `ip/bus/spi_display_rx`  four-wire SPI, which is what an SSD1306 in
//                            4-wire mode uses
//   anything else             I2C, a parallel bus, a test fixture, a CPU
//                            writing bytes — all the same to this block
//
// That split is deliberate. The protocol and the wire are independent
// questions and mixing them would make this block untestable without a
// serial link and unusable behind any other one.
//
// ===================================================================
// WHAT IS QUOTED AND WHAT IS CHECKED
// ===================================================================
//
// The command set below is **QUOTED** from the Solomon Systech SSD1306
// datasheet revision 1.1, section 10 ("Command Table") and section 8.7
// ("Graphic Display Data RAM"). Nothing here has been checked against a
// real SSD1306, because the point of the block is to stand in for one;
// what has been checked is that it behaves as the datasheet describes,
// and `ssd1306_slave_tb.v` is that check — it drives the initialisation
// sequence a real driver sends, writes known pixels, and reads the memory
// back.
//
// **No part has run this.** When one has, this header says so.
//
// ===================================================================
// THE DISPLAY MEMORY, AND WHY IT IS PAGES
// ===================================================================
//
// An SSD1306's memory is not a bitmap in scanline order. It is `PAGES`
// pages of `COLUMNS` bytes, and **each byte is eight vertically stacked
// pixels** — bit 0 the topmost. So the pixel at (x, y) is bit `y % 8` of
// the byte at page `y / 8`, column `x`, which is address
// `page * COLUMNS + column`.
//
// For 128x64 that is 8 pages of 128 bytes: 1024 bytes, which fits in one
// `RAMB18E1` on a 7-series part with room to spare. A reader that wants
// scanlines has to transpose, and doing that here would be inventing a
// format no SSD1306 driver speaks.
//
// ===================================================================
// THE ADDRESS POINTER
// ===================================================================
//
// Every data byte is written at the pointer, which then advances. How it
// advances is the addressing mode (command 0x20):
//
//   0  HORIZONTAL  column++; at the column window's end, back to its
//                  start and page++; at the page window's end, back to
//                  its start. This is what nearly every driver uses to
//                  push a whole frame.
//   1  VERTICAL    page++; at the page window's end, back to its start
//                  and column++; at the column window's end, back to its
//                  start.
//   2  PAGE        column++ only, wrapping within the page at COLUMNS.
//                  The page changes only when told to (0xB0..0xB7). This
//                  is the power-on mode and the one the old `0x00..0x0F`
//                  and `0x10..0x1F` column commands are for.
//
// The windows are set by 0x21 (column start, end) and 0x22 (page start,
// end) and default to the whole display.
//
// ===================================================================
// WHEN A FRAME IS DONE
// ===================================================================
//
// `frame_done` is a one-cycle strobe, and it fires when the pointer
// **wraps back to the start of both windows** — which in horizontal mode
// is exactly the moment a full frame has been written. That is the signal
// to take a copy.
//
// It is not a guess about timing and it needs no idle detector: a driver
// that writes 1024 bytes in horizontal mode produces exactly one strobe,
// whatever rate it writes at and however the bytes are split across
// transfers. In page mode a driver that fills all eight pages also
// produces one, on the last wrap. A driver that writes part of the screen
// produces none, which is correct — nothing was redrawn.
//
// `frame_count` counts the strobes.
//
// ===================================================================
// WHAT IT DOES WITH A COMMAND IT DOES NOT KNOW
// ===================================================================
//
// Counts it in `unknown_cmds` and otherwise ignores it, which is what a
// real SSD1306 does with most of the reserved space. The counter exists
// so that a driver using a command this block has not implemented is a
// number somebody can read rather than a silent misbehaviour — the same
// reason every other counter here exists.
//
// A multi-byte command whose argument never arrives leaves the block
// waiting for it, exactly as a real one would, and `pending_args` is
// visible so that a stuck sequence can be seen.

module ssd1306_slave #(
    // Columns and pages. 128x8 is an SSD1306 at 128x64; 128x4 is the
    // 128x32 part; `COLUMNS` of 132 with 8 pages is an SH1106, which is
    // the same protocol over a wider memory.
    parameter COLUMNS     = 128,
    parameter PAGES       = 8,
    // Width of the diagnostic counters. They saturate rather than wrap,
    // so a reader can tell "many" from "a few".
    parameter COUNT_WIDTH = 16,
    // 1 puts the display memory in a block RAM, which is right. 0 puts it
    // in lookup tables, which is a way round a 7-series backend defect —
    // see the memory's own comment below.
    parameter BLOCK_RAM   = 1
) (
    input  wire                   clk,
    input  wire                   rst_n,

    // One byte from the link, with the tag that says what it is.
    input  wire [7:0]             in_byte,
    input  wire                   in_is_data,
    input  wire                   in_valid,

    // The display memory, read by whatever wants a copy. Registered, so
    // `rd_data` is the byte at `rd_addr` one cycle later.
    //
    // Sixteen bits wide whatever the geometry, and the low
    // `clog2(COLUMNS * PAGES)` of them are used. A port cannot be sized
    // from a `localparam` computed in the body, and sizing it from a
    // parameter the instantiator must keep consistent with `COLUMNS` and
    // `PAGES` would be one more thing to get wrong for no gain.
    input  wire [15:0]            rd_addr,
    output wire [7:0]             rd_data,

    // A whole frame has just been written. One cycle.
    output wire                   frame_done,

    // What the display has been told to be.
    output wire                   display_on,
    output wire                   inverse,
    output wire                   all_on,
    output wire [7:0]             contrast,
    output wire [1:0]             addr_mode,
    output wire                   charge_pump,
    output wire                   seg_remap,
    output wire                   com_reverse,

    // Where the next data byte will go.
    // Likewise eight bits each, with the low bits used.
    output wire [7:0]             col_ptr,
    output wire [7:0]             page_ptr,
    // How many argument bytes the command in progress is still waiting
    // for; zero when none is.
    output wire [1:0]             pending_args,

    output wire [COUNT_WIDTH-1:0] cmd_count,
    output wire [COUNT_WIDTH-1:0] data_count,
    output wire [COUNT_WIDTH-1:0] frame_count,
    output wire [COUNT_WIDTH-1:0] unknown_cmds
);
    // ---- Widths, from the geometry rather than hard-coded. ----
    function integer bits_for;
        input integer n;
        integer v;
        begin
            bits_for = 1;
            v = n - 1;
            while (v > 1) begin
                v = v >> 1;
                bits_for = bits_for + 1;
            end
        end
    endfunction

    localparam integer COL_BITS  = bits_for(COLUMNS);
    localparam integer PAGE_BITS = bits_for(PAGES);
    localparam integer ADDR_BITS = COL_BITS + PAGE_BITS;
    localparam integer WORDS     = COLUMNS * PAGES;

    localparam [COUNT_WIDTH-1:0] CNT_ZERO = {COUNT_WIDTH{1'b0}};
    localparam [COUNT_WIDTH-1:0] CNT_MAX  = {COUNT_WIDTH{1'b1}};
    localparam [COUNT_WIDTH-1:0] CNT_ONE  = {{(COUNT_WIDTH-1){1'b0}}, 1'b1};

    localparam [1:0] MODE_H = 2'd0, MODE_V = 2'd1, MODE_PAGE = 2'd2;

    // The last column and the last page, at the width of the registers
    // that hold them. A sized `localparam` is how Verilog-2001 says this;
    // `COLUMNS - 1` on its own is a 32-bit integer expression and
    // assigning it truncates, which the simulator rightly warns about.
    // The values fit by construction, since COL_BITS and PAGE_BITS are
    // derived from COLUMNS and PAGES.
    localparam [COL_BITS-1:0]  COL_LAST  = COLUMNS - 1;
    localparam [PAGE_BITS-1:0] PAGE_LAST = PAGES - 1;

    // ---- The display memory. ----
    //
    // 1024 bytes is above the size a mapper puts in lookup tables by
    // itself, so `BLOCK_RAM = 1` asks for a block RAM and a `RAMB18E1`
    // holds it with half the block left over. That is the right choice and
    // the default.
    //
    // **`BLOCK_RAM = 0` exists as a way round a backend defect**, not as a
    // preference. On the 7-series, tying a block RAM's unused address bit
    // to ground searches only two pips for a ground path and fails when the
    // block lands somewhere without one:
    //
    //     error: block RAM `panel.gddram$ram_w0_d0`: tying `p0_addr2` to
    //     Zero: no free GND_WIRE path reaches
    //     `X75Y103/BRAM_RAMB18_ADDRARDADDR2` within two pips
    //
    // It is **placement-dependent** — the same design built, then failed
    // after an unrelated edit moved the placement, then built again — which
    // makes it worse than a reproducible failure. Until it is fixed,
    // `BLOCK_RAM = 0` puts the memory in lookup tables instead and builds
    // every time, at the cost of roughly a hundred and thirty `SLICEM`s
    // where one block RAM would do. `docs/fpga-xray.md` records the defect.
    reg [ADDR_BITS-1:0] wr_addr;
    reg [7:0]           wr_data;
    reg                 wr_en;

    wire [7:0] rd_q;

    generate
        if (BLOCK_RAM != 0) begin : as_block
            (* ram_style = "block" *)
            reg [7:0] mem [0:WORDS-1];
            reg [7:0] q;
            always @(posedge clk) begin
                if (wr_en) mem[wr_addr] <= wr_data;
                q <= mem[rd_addr[ADDR_BITS-1:0]];
            end
            assign rd_q = q;
        end else begin : as_lut
            (* ram_style = "distributed" *)
            reg [7:0] mem [0:WORDS-1];
            always @(posedge clk) begin
                if (wr_en) mem[wr_addr] <= wr_data;
            end
            // ASYNC (combinational) read: distributed/LUT RAM supports it, so
            // rd_data tracks rd_addr with ZERO latency. The block-RAM branch's
            // 1-cycle read latency interacts with the CW USB register bus's 4-byte
            // burst timing (reg_bytecnt advances per byte, but the DWORD-boundary
            // byte settles a cycle earlier) and SKIPS one column per 4-byte word on
            // readout. A 0-latency read makes reg_datao = mem[reg_bytecnt] uniformly,
            // exactly like the combinational gap_thresh/status reads, so the host
            // reads all COLUMNS straight with no drop-first and no periodic skip.
            assign rd_q = mem[rd_addr[ADDR_BITS-1:0]];
        end
    endgenerate

    // Block-RAM branch: 1 clock from rd_addr to rd_data. LUT branch: 0 (async).
    assign rd_data = rd_q;

    // ---- State the commands set. ----
    reg                 on_q;
    reg                 inverse_q;
    reg                 all_on_q;
    reg                 pump_q;
    reg                 remap_q;
    reg                 comrev_q;
    reg [7:0]           contrast_q;
    reg [1:0]           mode_q;

    reg [COL_BITS-1:0]  col_q;
    reg [PAGE_BITS-1:0] page_q;
    reg [COL_BITS-1:0]  col_start_q, col_end_q;
    reg [PAGE_BITS-1:0] page_start_q, page_end_q;

    // A command waiting for arguments: how many, and which command.
    reg [1:0]           args_q;
    reg [7:0]           opcode_q;
    // The first of a two-argument pair, held until the second arrives.
    reg [7:0]           arg0_q;

    reg                 frame_q;

    reg [COUNT_WIDTH-1:0] cmds_q, datas_q, frames_q, unknown_q;

    // ---- Where the pointer goes next, and whether that wraps a frame. ----
    wire at_col_end  = (col_q  == col_end_q);
    wire at_page_end = (page_q == page_end_q);

    // Page mode wraps the column at the memory's width, not at the
    // window: the datasheet is explicit that the column window applies to
    // the horizontal and vertical modes.
    wire page_mode_wrap = (col_q == (COLUMNS - 1));

    reg [COL_BITS-1:0]  next_col;
    reg [PAGE_BITS-1:0] next_page;
    reg                 next_wraps;

    always @(*) begin
        next_col   = col_q;
        next_page  = page_q;
        next_wraps = 1'b0;
        case (mode_q)
            MODE_H: begin
                if (at_col_end) begin
                    next_col = col_start_q;
                    if (at_page_end) begin
                        next_page  = page_start_q;
                        next_wraps = 1'b1;
                    end else begin
                        next_page = page_q + 1'b1;
                    end
                end else begin
                    next_col = col_q + 1'b1;
                end
            end
            MODE_V: begin
                if (at_page_end) begin
                    next_page = page_start_q;
                    if (at_col_end) begin
                        next_col   = col_start_q;
                        next_wraps = 1'b1;
                    end else begin
                        next_col = col_q + 1'b1;
                    end
                end else begin
                    next_page = page_q + 1'b1;
                end
            end
            default: begin
                // Page mode: the column wraps and the page stays. A wrap
                // here is not a frame — only the eighth page's wrap would
                // be, and nothing says the driver walked the pages in
                // order, so this reports none.
                next_col = page_mode_wrap ? {COL_BITS{1'b0}} : (col_q + 1'b1);
            end
        endcase
    end

    // The column pointer widened to eight bits, so the two halves of the
    // page-mode column command can be built without slicing a register
    // that may be narrower than the slice. Both results are truncated back
    // to `COL_BITS` where they are used.
    wire [7:0] col8       = {{(8 - COL_BITS){1'b0}}, col_q};
    wire [7:0] col_set_lo = {col8[7:4], in_byte[3:0]};
    wire [7:0] col_set_hi = {in_byte[3:0], col8[3:0]};

    // ---- The decoder. ----
    wire is_cmd  = in_valid && !in_is_data;
    wire is_data = in_valid &&  in_is_data;
    wire taking  = is_cmd && (args_q != 2'd0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Power-on state, QUOTED from the datasheet section 8.5
            // ("Reset Circuit") and the command table's defaults: display
            // off, page addressing mode, no remap, contrast 0x7F, the
            // charge pump off, and the pointer at the origin.
            on_q         <= 1'b0;
            inverse_q    <= 1'b0;
            all_on_q     <= 1'b0;
            pump_q       <= 1'b0;
            remap_q      <= 1'b0;
            comrev_q     <= 1'b0;
            contrast_q   <= 8'h7F;
            mode_q       <= MODE_PAGE;
            col_q        <= {COL_BITS{1'b0}};
            page_q       <= {PAGE_BITS{1'b0}};
            col_start_q  <= {COL_BITS{1'b0}};
            col_end_q    <= COL_LAST;
            page_start_q <= {PAGE_BITS{1'b0}};
            page_end_q   <= PAGE_LAST;
            args_q       <= 2'd0;
            opcode_q     <= 8'd0;
            arg0_q       <= 8'd0;
            frame_q      <= 1'b0;
            wr_en        <= 1'b0;
            wr_addr      <= {ADDR_BITS{1'b0}};
            wr_data      <= 8'd0;
            cmds_q       <= CNT_ZERO;
            datas_q      <= CNT_ZERO;
            frames_q     <= CNT_ZERO;
            unknown_q    <= CNT_ZERO;
        end else begin
            wr_en   <= 1'b0;
            frame_q <= 1'b0;

            // ---- A data byte: into memory, then advance. ----
            if (is_data) begin
                wr_en   <= 1'b1;
                wr_addr <= {page_q, col_q};
                wr_data <= in_byte;
                col_q   <= next_col;
                page_q  <= next_page;
                if (next_wraps) begin
                    frame_q <= 1'b1;
                    if (frames_q != CNT_MAX) frames_q <= frames_q + CNT_ONE;
                end
                if (datas_q != CNT_MAX) datas_q <= datas_q + CNT_ONE;
            end

            // ---- A command byte. ----
            if (is_cmd) begin
                if (cmds_q != CNT_MAX) cmds_q <= cmds_q + CNT_ONE;
            end

            if (taking) begin
                // An argument of the command in `opcode_q`.
                case (opcode_q)
                    8'h20: mode_q <= (in_byte[1:0] == 2'd3) ? MODE_PAGE
                                                            : in_byte[1:0];
                    8'h81: contrast_q <= in_byte;
                    8'h8D: pump_q <= in_byte[2];
                    8'h21: begin
                        if (args_q == 2'd2) begin
                            arg0_q      <= in_byte;
                            col_start_q <= in_byte[COL_BITS-1:0];
                            col_q       <= in_byte[COL_BITS-1:0];
                        end else begin
                            col_end_q <= in_byte[COL_BITS-1:0];
                        end
                    end
                    8'h22: begin
                        if (args_q == 2'd2) begin
                            arg0_q       <= in_byte;
                            page_start_q <= in_byte[PAGE_BITS-1:0];
                            page_q       <= in_byte[PAGE_BITS-1:0];
                        end else begin
                            page_end_q <= in_byte[PAGE_BITS-1:0];
                        end
                    end
                    // The rest take an argument and change nothing this
                    // block models: multiplex ratio, display offset,
                    // clock divide, pre-charge, COM pins, VCOMH. They are
                    // consumed so that the byte after them is read as a
                    // command and not as one of these.
                    default: ;
                endcase
                args_q <= args_q - 2'd1;
            end else if (is_cmd) begin
                // A new command.
                casez (in_byte)
                    // Two arguments.
                    8'h21, 8'h22: begin
                        opcode_q <= in_byte;
                        args_q   <= 2'd2;
                    end
                    // One argument.
                    8'h20, 8'h81, 8'h8D, 8'hA8, 8'hD3, 8'hD5, 8'hD9,
                    8'hDA, 8'hDB: begin
                        opcode_q <= in_byte;
                        args_q   <= 2'd1;
                    end
                    // Lower and higher column start, page addressing mode.
                    //
                    // Through `col8` rather than slicing `col_q` and
                    // `in_byte` directly, because the direct form only
                    // works at one geometry: at `COLUMNS = 128`,
                    // `COL_BITS` is 7 and `in_byte[COL_BITS-5:0]` is
                    // `in_byte[2:0]`, but at `COLUMNS = 16` it is
                    // `in_byte[-1:0]` and at `COLUMNS = 8` even
                    // `col_q[3:0]` is out of range. The manifest offers
                    // `COLUMNS` from 1 to 512, so the narrow cases are
                    // supported widths and not hypothetical ones.
                    //
                    // Found by `examples/basys3/ssd1306_console_tb.v`,
                    // which runs this block at 16x2 to keep a full dump
                    // simulable — a defect that one geometry hides and a
                    // second one shows immediately.
                    8'h0?: col_q <= col_set_lo[COL_BITS-1:0];
                    8'h1?: col_q <= col_set_hi[COL_BITS-1:0];
                    // Display start line: modelled as state only, because
                    // this block has no scan-out to offset.
                    8'b01??_????: ;
                    8'hA0: remap_q   <= 1'b0;
                    8'hA1: remap_q   <= 1'b1;
                    8'hA4: all_on_q  <= 1'b0;
                    8'hA5: all_on_q  <= 1'b1;
                    8'hA6: inverse_q <= 1'b0;
                    8'hA7: inverse_q <= 1'b1;
                    8'hAE: on_q      <= 1'b0;
                    8'hAF: on_q      <= 1'b1;
                    8'hC0: comrev_q  <= 1'b0;
                    8'hC8: comrev_q  <= 1'b1;
                    8'hB0, 8'hB1, 8'hB2, 8'hB3,
                    8'hB4, 8'hB5, 8'hB6, 8'hB7:
                        page_q <= in_byte[PAGE_BITS-1:0];
                    8'hE3: ;  // NOP
                    default:
                        if (unknown_q != CNT_MAX)
                            unknown_q <= unknown_q + CNT_ONE;
                endcase
            end
        end
    end

    assign frame_done   = frame_q;
    assign display_on   = on_q;
    assign inverse      = inverse_q;
    assign all_on       = all_on_q;
    assign contrast     = contrast_q;
    assign addr_mode    = mode_q;
    assign charge_pump  = pump_q;
    assign seg_remap    = remap_q;
    assign com_reverse  = comrev_q;
    assign col_ptr      = {{(8 - COL_BITS){1'b0}}, col_q};
    assign page_ptr     = {{(8 - PAGE_BITS){1'b0}}, page_q};
    assign pending_args = args_q;
    assign cmd_count    = cmds_q;
    assign data_count   = datas_q;
    assign frame_count  = frames_q;
    assign unknown_cmds = unknown_q;
endmodule
