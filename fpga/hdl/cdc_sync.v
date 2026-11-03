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
// Vendored from reticle: ip/util/cdc_sync/rtl/cdc_sync.v
// Source commit: 1696ca2  copied 2026-11-03T02:30Z
// Dependency of spi_display_rx. DO NOT EDIT HERE — re-vendor from reticle.
// cdc_sync — an N-flop clock domain crossing synchroniser.
//
// What it does
//   Registers `d` through STAGES flip-flops clocked by `clk`, so a signal
//   that changes asynchronously to `clk` is resolved before the rest of
//   the destination domain sees it. The first flop is the one allowed to
//   go metastable; every further flop buys another clock period for it to
//   settle. `q` is the last flop in the chain.
//
//   WIDTH bits are synchronised side by side, each through its own chain.
//   INIT is the value every stage takes while `rst_n` is low, which is
//   how an active-low signal (a reset, a `ready`) is held at its safe
//   level out of reset.
//
// What it does not do
//   It does not make a multi-bit value safe to cross. Each bit is
//   resolved independently, so two bits that change in the same source
//   cycle may arrive in different destination cycles. Crossing a bus
//   needs a gray code (at most one bit changes per source clock, which is
//   what `fifo_async` does with its pointers) or a handshake (which is
//   what `cdc_pulse` does). Reticle's own `timing::analyze_cdc` says as
//   much: a multi-bit crossing through this block is reported as a gray
//   bus it cannot verify.
//
//   It does not synchronise the *deassertion* of a reset into this
//   domain by itself — tie `d` to 1 and `rst_n` to the asynchronous reset
//   to build that, which is the usual reset bridge.
//
//   It adds STAGES cycles of latency and does not preserve pulses
//   shorter than one destination clock period. Use `cdc_pulse` for those.
//
// Why the stages are written out rather than generated
//   A chain written as `sync_q <= {sync_q[STAGES-2:0], d}` infers one
//   multi-bit register rather than a chain of flops. Reticle's
//   `timing::analyze_cdc` recognises that shape too, so either spelling
//   is checked correctly here, but not every external CDC tool does, and
//   separate registers also let a placer keep the pair close. The stages
//   beyond STAGES drive nothing and are removed by dead code
//   elimination, so the cost is exactly STAGES * WIDTH flip-flops.
module cdc_sync #(
    // Bits synchronised side by side.
    parameter WIDTH  = 1,
    // Flip-flops in each chain, 2 to 4.
    parameter STAGES = 2,
    // Value held in every stage while `rst_n` is low (0 or 1).
    parameter INIT   = 0
) (
    input  wire             clk,
    input  wire             rst_n,
    input  wire [WIDTH-1:0] d,
    output wire [WIDTH-1:0] q
);
    localparam INIT_BIT = (INIT != 0) ? 1'b1 : 1'b0;

    reg [WIDTH-1:0] stage1_q;
    reg [WIDTH-1:0] stage2_q;
    reg [WIDTH-1:0] stage3_q;
    reg [WIDTH-1:0] stage4_q;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stage1_q <= {WIDTH{INIT_BIT}};
            stage2_q <= {WIDTH{INIT_BIT}};
            stage3_q <= {WIDTH{INIT_BIT}};
            stage4_q <= {WIDTH{INIT_BIT}};
        end else begin
            stage1_q <= d;
            stage2_q <= stage1_q;
            stage3_q <= stage2_q;
            stage4_q <= stage3_q;
        end
    end

    assign q = (STAGES <= 2) ? stage2_q :
               (STAGES == 3) ? stage3_q :
                               stage4_q;
endmodule
