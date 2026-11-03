// -----------------------------------------------------------------------------
// Copyright (c) 2026 Karpelès Lab Inc.   SPDX-License-Identifier: MIT
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps
`default_nettype none
`include "includes.v"

/***********************************************************************
reg_seph_terminal -- register-bus wrapper around reticle's iso7816_terminal,
so the Husky FPGA brings a Ledger Nano X SE up END-TO-END in hardware:
power -> 8 MHz clock -> cold reset -> ATR -> PPS -> SESSION_START + continuous
SEPROXYHAL tickers/status (via the embedded seph_mcu). The host only writes a
few command bytes and reads the ATR/status; the hardware keeps the SE serviced
forever, so the link never goes silent.

The terminal runs on clk_usb (96 MHz); params retuned from the 112 MHz board:
CARD_DIV 14->12 (96/12 = 8 MHz, even => 50% duty), SEPH_MS_CYCLES 112000->96000
(100 ms ticker), SEPH_TURN_CYCLES 11200->9600 (100 us). Its clk_card / vcc_en /
rst_card / io are muxed onto the Husky's target_hs2 / target_poweron /
target_PDIC / target_io3 in cwhusky_top when SEPH_CTRL[0] (enable) is set.

Command bytes (write to SEPH_CMD): 'A'=0x41 activate (VCC->clock->cold reset->
ATR), 'P'=0x50 send PPS (FF 10 97 78), 'F'=0x46 switch to 1 Mbaud, 'W'=0x57 start
seph_mcu, 'm'=0x6D mute, 'D'=0x44 deactivate. Minimal boot: A,(read ATR),P,
(verify echo),F,W. Writing 'A' also restarts the 16-byte ATR capture.

Register map (clk_usb domain, big-OR readback):
  SEPH_CMD     write: one command byte -> cmd_valid pulse to the terminal
  SEPH_CTRL    write bit0 = enable (hand the SE pins to the FPGA terminal)
               read back bit0
  SEPH_STATUS  read, muxed by reg_bytecnt:
                 0: {seph_active,seph_started,card_active,2'b0,state[2:0]}
                 1: seph_last_tag
                 2..3: card_bytes (16-bit LE, total card bytes seen)
                 4: {3'b0, atr_count[4:0]}
  SEPH_ATR     read 16 bytes = ATR capture buffer[reg_bytecnt]
*************************************************************************/

module reg_seph_terminal #(
   parameter pBYTECNT_SIZE = 7
)(
   input  wire         reset,          // active-high, synchronous
   input  wire         clk_usb,        // 96 MHz
   input  wire [7:0]   reg_address,
   input  wire [pBYTECNT_SIZE-1:0] reg_bytecnt,
   input  wire [7:0]   reg_datai,
   output reg  [7:0]   reg_datao,
   input  wire         reg_read,
   input  wire         reg_write,

   // SE pin signals (muxed onto Husky pins by cwhusky_top when enabled)
   output wire         seph_enable,    // SEPH_CTRL[0] -> top mux select
   output wire         clk_card,       // -> target_hs2  (8 MHz)
   output wire         vcc_en,         // -> target_poweron (active high)
   output wire         rst_card,       // -> target_PDIC (HIGH=run, LOW=reset)
   output wire         io_oe,          // target_io3 drive-enable
   output wire         io_o,           // target_io3 drive value (const 0)
   input  wire         io_i            // target_io3 as read
);

   // ---- config / command ------------------------------------------------
   reg        enable;
   reg  [7:0] cmd_data;
   reg        cmd_valid;               // 1-cycle strobe to the terminal

   always @(posedge clk_usb) begin
      if (reset) begin
         enable    <= 1'b0;
         cmd_valid <= 1'b0;
      end
      else begin
         cmd_valid <= 1'b0;            // default: pulse only on the write cycle
         if (reg_write) begin
            case (reg_address)
               `SEPH_CMD:  begin cmd_data <= reg_datai; cmd_valid <= 1'b1; end
               `SEPH_CTRL: enable <= reg_datai[0];
            endcase
         end
      end
   end
   assign seph_enable = enable;

   // ---- the terminal (iso7816_uart + seph_mcu), hardware MCU ------------
   wire [7:0]  o_card_rx;
   wire        o_card_rx_valid;
   wire [2:0]  o_state;
   wire        o_card_active, o_seph_started, o_seph_active;
   wire [7:0]  o_seph_last_tag;
   wire [15:0] o_card_bytes;

   iso7816_terminal #(
      .CARD_DIV(12),            // 96/12 = 8 MHz (even -> 50% duty)
      .ETU_CYCLES(372),
      .FAST_ETU_CYCLES(8),
      .PPS1(8'h97),
      .SEPH_MS_CYCLES(96000),   // 96 MHz -> 1 ms  (100 ms ticker)
      .SEPH_TURN_CYCLES(9600)   // 100 us at 96 MHz
   ) u_term (
      .clk(clk_usb), .locked(1'b1),
      .clk_card(clk_card), .rst_card(rst_card), .vcc_en(vcc_en),
      .io_i(io_i), .io_oe(io_oe), .io_o(io_o),
      .cmd_valid(cmd_valid), .cmd_data(cmd_data),
      .seph_buttons(2'b00),    // GPIO buttons (pdid/nrst) handled outside
      .buttons_in(2'b00),
      .out_valid(), .out_data(), .out_ready(1'b1),   // drop the ASCII stream
      .hex_run(),
      .led(), .seg(), .dp(), .an(),
      .o_card_rx(o_card_rx), .o_card_rx_valid(o_card_rx_valid),
      .o_state(o_state), .o_card_active(o_card_active),
      .o_seph_started(o_seph_started), .o_seph_active(o_seph_active),
      .o_seph_last_tag(o_seph_last_tag), .o_card_bytes(o_card_bytes)
   );

   // ---- ATR capture: first 16 card bytes after each 'A' -----------------
   // ram_style=registers: 16 bytes is tiny, and forcing FFs avoids the
   // distributed-RAM write-address hold path (RAMD32 WADR vs WCLK) that
   // physopt could not close (WHS ~ -0.05 ns at the fast corner).
   (* ram_style = "registers" *)
   reg [7:0] atr [0:15];
   reg [4:0] atr_wptr;
   wire a_cmd = reg_write && (reg_address == `SEPH_CMD) && (reg_datai == 8'h41);
   always @(posedge clk_usb) begin
      if (reset || a_cmd)
         atr_wptr <= 5'd0;
      else if (o_card_rx_valid && atr_wptr < 5'd16) begin
         atr[atr_wptr[3:0]] <= o_card_rx;
         atr_wptr <= atr_wptr + 5'd1;
      end
   end

   // ---- read side -------------------------------------------------------
   reg [7:0] status_byte;
   always @(*) begin
      case (reg_bytecnt)
         0: status_byte = {o_seph_active, o_seph_started, o_card_active,
                           2'b0, o_state};
         1: status_byte = o_seph_last_tag;
         2: status_byte = o_card_bytes[7:0];
         3: status_byte = o_card_bytes[15:8];
         4: status_byte = {3'b0, atr_wptr};
         default: status_byte = 8'd0;
      endcase
   end

   always @(*) begin
      if (reg_read) begin
         case (reg_address)
            `SEPH_CMD:    reg_datao = cmd_data;
            `SEPH_CTRL:   reg_datao = {7'b0, enable};
            `SEPH_STATUS: reg_datao = status_byte;
            `SEPH_ATR:    reg_datao = atr[reg_bytecnt[3:0]];
            default:      reg_datao = 8'd0;
         endcase
      end
      else
         reg_datao = 8'd0;
   end

endmodule
`default_nettype wire
