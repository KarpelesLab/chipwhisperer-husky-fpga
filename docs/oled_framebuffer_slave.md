# OLED / SSD1306 framebuffer slave for ChipWhisperer-Husky

This fork adds a **live display-RAM mirror** to the Husky FPGA: it taps an
SSD1306/SSD1312 OLED's 4-wire SPI off the target IO pins and keeps a copy of the
panel's GDDRAM in the FPGA, readable over the normal USB register bus. Reading a
screen is then a plain register read — no logic-analyzer capture window, no
stitching, no corruption from a partial redraw.

Originally built to read a Ledger Nano X secure-element's screen on a
`CW308T-QFN20-SOCKET`, but the block is a generic SSD1306 sniffer.

## What's added

| file | role |
|------|------|
| `fpga/hdl/reg_sca_display.v` | register block: taps TIO1/2/4, synthesizes `cs_n` from the sclk gap, mirrors GDDRAM, exposes it + diagnostics on the reg bus |
| `fpga/hdl/spi_display_rx.v` | 4-wire SPI receiver → bytes + D/C tag (vendored from the MIT-licensed `reticle` project) |
| `fpga/hdl/ssd1306_slave.v` | SSD1306 command/data decode into a BRAM framebuffer (vendored from `reticle`) |
| `fpga/hdl/cdc_sync.v` | small synchronizer used by `spi_display_rx` (vendored from `reticle`) |

Our files are MIT (Karpelès Lab); the rest of the repo remains GPLv3.

### Pin tap

`reg_sca_display` samples three of the four display wires as plain async inputs
(the socket used here doesn't route OLED_CS to the FPGA, so the byte frame is
recovered from the idle gap between sclk bursts):

```
OLED_CLK = TIO1 = target_io1 -> oled_sclk
OLED_DIN = TIO2 = target_io2 -> oled_mosi
OLED_DC  = TIO4 = target_io4 -> oled_dc
```

Everything runs on `clk_usb_buf` (96 MHz), the register-bus clock, so there is no
CDC between the framebuffer BRAM and the readout, and 96 MHz heavily oversamples a
~1 MHz display SPI. `cs_n` is synthesized with a register-tunable gap threshold
(`DISP_GAP_THRESH`); the sampled bus is pipeline-delayed a few cycles so the
gap-derived frame opening leads the first sampled edge (so `FRAME_MODE=0` never
drops the first bit). See the header of `reg_sca_display.v` for the details.

### Register map (main CW bank)

| name | addr | access |
|------|------|--------|
| `DISP_STATUS` | 85 | read, muxed by `reg_bytecnt` (flags, start line, bit-error/overrun/frame/data/cmd counters, col/page ptr) |
| `DISP_PAGESEL` | 86 | write: which 128-byte page `DISP_DATA` returns (0–7) |
| `DISP_DATA` | 87 | read 128 bytes = `GDDRAM[page*128 + reg_bytecnt]` |
| `DISP_GAP_THRESH` | 94 | read/write 16-bit cs-synth gap threshold (clk cycles, default 96) |
| `DISP_CONTROL` | 97 | write bit0 = soft reset (rx/slave framing + counters) |

Read all 8 pages to get the 1024-byte page-major framebuffer. The diagnostic
counters in `DISP_STATUS` let a host confirm a clean decode and tune
`DISP_GAP_THRESH` live.

## Building (Vivado 2026.1)

The committed `cwhusky_plus.xpr` still points at a stale sibling
`../../../plus_optimization/...` directory for a number of IP — that's the
pre-merge local checkout of the `plus_optimization` *branch*, which is merged
into `develop`, so the sources are in-tree now and only the paths are stale.
`open_project` fails on the missing paths, but a **fresh project** built from
the in-tree `develop` sources works (the Plus defines `PLUS INTERVAL_MATCHING=1
ESAD`, without `TINYFIFO`/`SEMIPRO`, select the in-tree IP).
`fpga/vivado/build_oled.tcl` does this (part `xc7a100tftg256-3`, top
`cwhusky_top`, XPM libs, `Performance_Retiming`).

Fixes this fork needed to build/close timing in 2026.1 (see the commits):

1. **`clk_usb` global buffer** — `cwhusky_top.v` instantiated only an `IBUFG`
   for `clk_usb_buf` and relied on Vivado auto-inserting the `BUFG`. 2026.1 does
   not (clk_usb_buf is also tapped as a debug data bit, which suppresses
   auto-insertion), leaving the 96 MHz clock on general routing (fanout ~5944,
   ~14 ns skew) and breaking the whole `clk_usb` domain. Fix: explicit `BUFG`
   after the `IBUFG` + `CLOCK_DEDICATED_ROUTE ANY_CMT_COLUMN` on the IBUF→BUFG
   net. **This is a latent fragility independent of the OLED feature.**
2. **Clock-mux cascades** — NewAE's chained BUFGMUX clock selects
   (glitch/LA/trace) need `CLOCK_DEDICATED_ROUTE ANY_CMT_COLUMN` overrides
   (exactly Vivado's suggested workarounds), in `cwhusky_100.xdc`.
3. `fifo_generator_semipro` (used by the trace FIFO under `` `elsif PLUS ``) has
   no `.xci` in `develop` — looks like it didn't get committed with the rest of
   the merged branch. Here it's a renamed copy of the in-tree `fifo_generator_0`
   (identical ports; trace is unused on this bench).

## Host readout

A reference Python reader (ChipWhisperer `scope.sc.msg_read/msg_write`) is in the
Ledger analysis bench; the protocol is just: write `DISP_PAGESEL`, read 128 bytes
from `DISP_DATA`, repeat for 8 pages; read `DISP_STATUS` for the start line and
health counters.
