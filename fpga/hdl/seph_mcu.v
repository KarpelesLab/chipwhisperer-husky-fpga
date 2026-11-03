// seph_mcu: the MCU's end of Ledger's SEPROXYHAL link, enough of it to
// bring a secure element up and keep it running.
//
// A Nano X is two chips: a secure element (the SE, the card on an ISO 7816
// contact) and an MCU that owns the buttons, the power and the radios. The
// two talk SEPROXYHAL over that contact once the PPS has set its rate. This
// block is the MCU's side, as bytes: it does not know about ISO 7816, and
// `iso7816_uart` or anything else that moves bytes carries them.
//
// ===================================================================
// THE PROTOCOL, AS FAR AS THIS BLOCK NEEDS IT
// ===================================================================
//
// Every packet is a tag, a 16-bit big-endian length and that many bytes.
// The link is **turn based**, and the turns are what this block is about:
//
//   - the MCU sends **one event** (tags 0x01..0x1F);
//   - the SE answers with any number of commands (0x30..0x5F) and ends
//     its turn with a **status** (0x60..0x6F; `60 00 02 00 00`,
//     GENERAL_STATUS "last command", is the usual one);
//   - then, and only then, the MCU may send its next event.
//
// The one exception is the first event: `start` sends SESSION_START with
// no status before it.
//
// What the MCU sends on its turn, in this order of priority:
//
//   1. BLE_RECV_EVENT (0x18), an HCI "command complete" with status
//      success, if the SE sent a BLE_SEND (0x38) since the last one. The
//      SE configures a radio this block does not have; it waits for each
//      command's completion before sending the next, so each has to be
//      answered, and answering "success" is how the configuration is
//      accepted without a radio.
//   2. STATUS_EVENT (0x15), the `STATUS` parameter with the flags below
//      added to its low flag byte, if the SE sent REQUEST_STATUS (0x52)
//      since the last one. It does that about once a second.
//   3. USB_EP_XFER_EVENT (0x10) "IN done" for an IN transfer the SE
//      prepared (USB_EP_PREPARE, 0x50, type 0x20) -- endpoint 0's first,
//      as though the host had read it at once. The data itself reaches a
//      host the way every SE byte does: whoever carries this block's bytes
//      prints them.
//   4. USB_EVENT (0x0F) reset, once, when `usb_plugged` is high and the SE
//      has connected (USB_CONFIG 0x4F, 01): what a host does when a device
//      appears on its bus.
//   5. An injected event, sent verbatim: whatever a host outside wants the
//      SE to see -- a SETUP or an OUT transfer, or anything else.
//   6. BUTTON_PUSH_EVENT (0x05) if `buttons` differs from what was last
//      reported.
//   7. TICKER_EVENT (0x0E), milliseconds since `start` as 32 bits, once
//      the SE's own interval has elapsed since the last ticker. The SE sets
//      that interval with SET_TICKER_INTERVAL (0x4E); 100 ms until it does.
//
// If none is due the turn waits: the SE expects nothing sooner. Every other
// SE command -- MCU lock, MORE_TIME, display power, the rest of USB
// configuration -- is read, counted and needs no answer.
//
// USB
// ---
// This block is not a USB host. It does what the MCU does for one -- the
// bus reset, the IN completions, the status flags -- and leaves the host's
// side, enumeration and the APDUs, to whoever drives `inj_*`. That split is
// the capture's: everything the MCU sends for USB is either one of those
// three or a SETUP or OUT transfer carrying the host's own bytes.
//
// STATUS_EVENT's low flag byte follows the cable: 0x09 (charging, USB
// powered) while it is in, 0x02 (USB on) added once the reset has gone,
// 0x04 (BLE on) once the SE has configured the radio. A real MCU sent
// 0x0d, then 0x0f after the host's reset, in the capture.
//
// Where this comes from: a capture of a real Nano X's MCU and SE
// (`visgrok`, `capture-20261009-125946.seph.log`, 44143 packets), read for
// what the MCU sends in answer to what. It is CHECKED against a real SE
// 2.5.1 on 9 October 2026 from a host-side prototype of exactly this
// policy: SESSION_START, then 182 tickers and 18 status events over
// 20 s, every one answered with a status, and the SE drew its PIN screen.
// The BLE answers are from the capture, not from that run: that SE sent no
// BLE command. `seph_mcu_replay_tb.v` replays the capture's configuration
// and checks all fifteen answers byte for byte.
//
// ===================================================================
// WHAT IS A PARAMETER, AND WHY
// ===================================================================
//
// `SESSION` and `STATUS` are whole packets, right-aligned in a 512-bit
// constant with their lengths beside them. They depend on which MCU the SE
// expects: SESSION_START names the MCU's firmware and bootloader versions
// and an SE checks them -- the capture shows an SE ordering an MCU into
// its bootloader to be updated -- and STATUS_EVENT's layout changed between
// MCU versions (20 bytes of payload with 2.39.0, 10 with 2.40.3). The
// defaults are what worked against SE 2.5.1: MCU "2.28", bootloader
// "1.16", and a status saying on battery at 80 %, 4000 mV, not charging,
// with the radio's flag (0x04) added once the SE has configured it.
//
// **The status used to say USB-powered and charging (flags 0x09)**, on the
// reasoning that the SE would then neither expect a USB host nor warn about
// the battery. A real MCU never sends that: in 44143 packets it reports
// USB-powered only with USB on or about to be (0x0d, 0x0e, 0x0f, after a
// host's bus reset), BLE on whenever the SE has started the radio, and 0x04
// alone on battery. With 0x09 an SE 2.5.1 drew its PIN screen and took
// digits, then went back to "Welcome" at the onboarding's PIN confirmation,
// 15 October 2026 -- the first step that writes anything -- and the
// status was suspected. **It was not the cause:** with flags 0 the same SE
// went back to "Welcome" at the same step, the same day, with the same
// traffic. The default stays changed because it is what a real MCU says;
// the restart is still open.
//
// `BUTTON_SHIFT`: the button byte is `buttons << BUTTON_SHIFT`. Ledger's
// SDK reads it as `byte >> 1`; that is QUOTED from the SDK, not measured
// on a part.
module seph_mcu #(
    // System clocks per millisecond: 112 000 at 112 MHz.
    parameter integer MS_CYCLES     = 112000,
    // System clocks between the end of the SE's status and the first byte
    // of the MCU's event: 11 200 is 100 us at 112 MHz, what a real MCU
    // leaves (`5.526154` status, `5.526256` event in the capture). An SE
    // needs the line turned round; an event sent the cycle after its
    // status lost its first four bytes against a model card.
    parameter integer TURN_CYCLES   = 11200,
    parameter integer SESSION_LEN   = 51,
    parameter [511:0] SESSION       = {104'd0,
        408'h010030000800030304322e323804f4d8aa4304312e313604f1308974100702020014150c090106312e31312e3005312e322e30},
    parameter integer STATUS_LEN    = 23,
    parameter [511:0] STATUS        = {328'd0,
        184'h15001400000000000000000000000fa050db1bffffffdb},
    parameter integer BUTTON_SHIFT  = 1
) (
    input  wire        clk,
    input  wire        rst_n,

    // A one-cycle pulse: send SESSION_START and run. `stop` goes idle.
    input  wire        start,
    input  wire        stop,

    // Bytes from the SE, one-cycle strobe.
    input  wire [7:0]  rx_data,
    input  wire        rx_valid,

    // Bytes to the SE: `tx_valid` is held with `tx_data` until a cycle in
    // which `tx_ready` is high too.
    output reg  [7:0]  tx_data,
    output reg         tx_valid,
    input  wire        tx_ready,

    // Bit 0 the left button, bit 1 the right, high while pressed.
    input  wire [1:0]  buttons,

    // A USB cable to a host, as a level. See "USB" above.
    input  wire        usb_plugged,
    // The host has reset the bus since the SE connected: STATUS's USB on.
    output reg         usb_on,

    // One event from outside, sent whole on a turn of its own: `inj_start`
    // empties the buffer, each `inj_valid` appends `inj_data`, and
    // `inj_commit` offers what was appended. `inj_bad` pulses the cycle
    // after a commit that was refused; `inj_busy` is high from an accepted
    // commit until the event's last byte has gone.
    input  wire        inj_start,
    input  wire [7:0]  inj_data,
    input  wire        inj_valid,
    input  wire        inj_commit,
    output wire        inj_busy,
    output reg         inj_bad,

    output reg         active,
    // The SE has ended a turn since `start`: it took the session.
    output reg         started,
    output reg  [15:0] rx_packets,
    output reg  [15:0] tx_events,
    output reg  [7:0]  last_rx_tag,
    output reg  [31:0] ms
);
    // ---- The millisecond clock ----
    localparam integer PW = (MS_CYCLES > 1) ? $clog2(MS_CYCLES) : 1;
    reg  [PW-1:0] pre      = {PW{1'b0}};
    wire          ms_pulse = (pre == MS_CYCLES - 1);

    // ---- What the SE has asked for ----
    reg        our_turn    = 1'b0;
    reg        want_status = 1'b0;
    reg        want_ble    = 1'b0;
    // The radio is on once the SE has configured it: STATUS_EVENT's flag
    // 0x04, which a real MCU sets in its first status after the SE's BLE
    // commands (`6.677650` in the capture, flags 0x00000004).
    reg        ble_on      = 1'b0;
    reg [15:0] ble_op      = 16'd0;
    reg [7:0]  char_handle = 8'h0d;
    reg [15:0] interval    = 16'd100;
    reg [15:0] since_tick  = 16'd0;
    reg [1:0]  reported    = 2'b00;
    localparam integer TW = (TURN_CYCLES > 1) ? $clog2(TURN_CYCLES + 1) : 1;
    localparam [TW-1:0] TURN_LOAD = TURN_CYCLES;
    reg [TW-1:0] turn_wait = {TW{1'b0}};

    // ---- Receiving: tag, two length bytes, payload ----
    localparam [1:0] R_TAG = 2'd0, R_LENH = 2'd1, R_LENL = 2'd2, R_BODY = 2'd3;
    reg [1:0]  rstate = R_TAG;
    reg [7:0]  rtag   = 8'd0;
    reg [7:0]  rlenh  = 8'd0;
    reg [15:0] rleft  = 16'd0;
    reg [15:0] rlen   = 16'd0;
    reg [7:0]  p0     = 8'd0;
    reg [7:0]  p1     = 8'd0;
    reg [7:0]  p2     = 8'd0;
    // ACI_GATT_ADD_CHAR's Char_Properties, byte 23 of a BLE_SEND's payload
    // (opcode 2, service handle 2, UUID type 1, UUID 16, value length 2).
    reg [7:0]  p23    = 8'd0;
    reg [7:0]  ble_props = 8'd0;
    reg [15:0] rpos   = 16'd0;

    // A packet has just ended, with these: the tag, its length and the
    // first two payload bytes, which are all any answer here depends on.
    wire       end_empty = rx_valid && rstate == R_LENL && {rlenh, rx_data} == 16'd0;
    wire       end_body  = rx_valid && rstate == R_BODY && rleft == 16'd1;
    wire       done      = active && (end_empty || end_body);
    wire [7:0] d_p0      = (end_body && rpos == 16'd0) ? rx_data : p0;
    wire [7:0] d_p1      = (end_body && rpos == 16'd1) ? rx_data : p1;
    wire [7:0] d_p2      = (end_body && rpos == 16'd2) ? rx_data : p2;
    wire [7:0] d_p23     = (end_body && rpos == 16'd23) ? rx_data : p23;
    wire [15:0] d_len    = end_empty ? 16'd0 : rlen;

    // ---- Sending ----
    localparam [2:0] K_SESSION = 3'd0, K_STATUS = 3'd1, K_TICKER = 3'd2,
                     K_BUTTON  = 3'd3, K_BLE    = 3'd4, K_IN_DONE = 3'd5,
                     K_USB_RESET = 3'd6, K_INJECT = 3'd7;
    reg        sending = 1'b0;
    reg [2:0]  kind    = K_SESSION;
    reg [10:0] idx     = 11'd0;
    reg [10:0] last    = 11'd0;   // index of the final byte
    reg [31:0] snap    = 32'd0;   // the ticker's milliseconds
    reg [1:0]  bsnap   = 2'b00;
    reg [15:0] bop     = 16'd0;
    reg [7:0]  bhandle = 8'd0;

    // The HCI return parameters that the capture shows mattering: the GAP
    // init's three handles, a service's handle and a characteristic's.
    function [3:0] ble_ret_len;
        input [15:0] op;
        case (op)
            16'hfc8a: ble_ret_len = 4'd6;
            16'hfd02: ble_ret_len = 4'd2;
            16'hfd04: ble_ret_len = 4'd2;
            default:  ble_ret_len = 4'd0;
        endcase
    endfunction

    function [7:0] ble_ret;
        input [15:0] op;
        input [3:0]  at;
        input [7:0]  handle;
        case (op)
            16'hfc8a: case (at)
                4'd0: ble_ret = 8'h05;  4'd2: ble_ret = 8'h06;
                4'd4: ble_ret = 8'h08;  default: ble_ret = 8'h00;
            endcase
            16'hfd02: ble_ret = (at == 4'd0) ? 8'h0c : 8'h00;
            16'hfd04: ble_ret = (at == 4'd0) ? handle : 8'h00;
            default:  ble_ret = 8'h00;
        endcase
    endfunction

    wire [3:0] brl = ble_ret_len(bop);

    function [7:0] byte_at;
        input [2:0]  k;
        input [6:0]  i;
        input [31:0] t;
        input [1:0]  b;
        input [15:0] op;
        input [3:0]  rl;
        input [7:0]  handle;
        input [7:0]  usb_flags;
        reg   [8:0]  shifted;
        case (k)
            K_SESSION: byte_at = SESSION[8 * (SESSION_LEN - 1 - i) +: 8];
            // Byte 6 is the low byte of the flags, where the radio's bit is.
            K_STATUS:  byte_at = STATUS[8 * (STATUS_LEN - 1 - i) +: 8]
                               | ((i == 7'd6) ? usb_flags : 8'h00);
            K_TICKER:  case (i)
                7'd0: byte_at = 8'h0e;  7'd1: byte_at = 8'h00;  7'd2: byte_at = 8'h04;
                7'd3: byte_at = t[31:24];  7'd4: byte_at = t[23:16];
                7'd5: byte_at = t[15:8];   default: byte_at = t[7:0];
            endcase
            K_BUTTON:  begin
                shifted = {7'd0, b} << BUTTON_SHIFT;
                case (i)
                    7'd0: byte_at = 8'h05;  7'd1: byte_at = 8'h00;  7'd2: byte_at = 8'h01;
                    default: byte_at = shifted[7:0];
                endcase
            end
            // USB_EP_XFER_EVENT, "IN done": the endpoint with its IN bit,
            // type 02, the length -- carried in `op` as {ep, len}.
            K_IN_DONE: case (i)
                7'd0: byte_at = 8'h10;  7'd1: byte_at = 8'h00;  7'd2: byte_at = 8'h03;
                7'd3: byte_at = op[15:8];  7'd4: byte_at = 8'h02;
                default: byte_at = op[7:0];
            endcase
            // USB_EVENT, reset.
            K_USB_RESET: byte_at = (i == 7'd2 || i == 7'd3) ? 8'h01 : (i == 7'd0 ? 8'h0f : 8'h00);
            default:   case (i)    // K_BLE
                // BLE_RECV_EVENT carrying an HCI event packet: indicator
                // 04, event 0E (command complete), its length, one
                // command allowed, the opcode little-endian, status 00.
                7'd0: byte_at = 8'h18;  7'd1: byte_at = 8'h00;
                7'd2: byte_at = 8'd7 + {4'd0, rl};
                7'd3: byte_at = 8'h04;  7'd4: byte_at = 8'h0e;
                7'd5: byte_at = 8'd4 + {4'd0, rl};
                7'd6: byte_at = 8'h01;  7'd7: byte_at = op[7:0];
                7'd8: byte_at = op[15:8];  7'd9: byte_at = 8'h00;
                default: byte_at = ble_ret(op, i[3:0] - 4'd10, handle);
            endcase
        endcase
    endfunction

    localparam [10:0] session_last = SESSION_LEN - 1;
    localparam [10:0] status_last  = STATUS_LEN - 1;

    // ---- USB ----
    //
    // `se_usb` follows the SE's USB_CONFIG connect and disconnect: whether
    // its pull-up is on, so whether a host would see it. A host resets the
    // bus when it sees a device, so a reset is due while the cable is in,
    // the SE is connected and no reset has been sent since.
    reg        se_usb    = 1'b0;
    wire       want_reset = usb_plugged && se_usb && !usb_on;
    // STATUS's low flag byte: charging and USB powered (0x09) with the
    // cable in, USB on (0x02) after the reset, BLE on (0x04) once the SE
    // has configured the radio -- 0x0f with all three, as a real MCU sends.
    wire [7:0] flags_low = (usb_plugged ? 8'h09 : 8'h00) | (usb_on ? 8'h02 : 8'h00)
                         | (ble_on ? 8'h04 : 8'h00);

    // An IN transfer the SE prepared is reported done on a later turn, as
    // if the host had read it at once. One for endpoint 0 and one for any
    // other: the SE prepares at most one of each before it hears back.
    reg        in0_want = 1'b0;
    reg [7:0]  in0_len  = 8'd0;
    reg        inx_want = 1'b0;
    reg [7:0]  inx_ep   = 8'd0;

    // ---- The injected event ----
    //
    // A block RAM, written from the host side and read one byte ahead of
    // `idx`: the read address is the byte that will be loaded next, so
    // that `inj_q` holds it by the cycle `tx_data` takes it.
    //
    // **2048 bytes, though a HID transfer is 70**, because that is exactly
    // what a `RAMB18E1` holds eight bits wide, so every address pin of its
    // mode is one the design drives. At 256 bytes the three above were left
    // for the flow to tie to ground, and on `iso_display.v` one of them
    // could not be: the router had taken the ground fan its pin needed,
    // and the build stopped (`p0_addr12`, 16 October 2026).
    (* ram_style = "block" *)
    reg  [7:0] inj_mem [0:2047];
    reg  [7:0] inj_q    = 8'd0;
    reg  [11:0] inj_w   = 12'd0;    // bytes appended; 2048 means too many
    reg  [10:0] inj_len1 = 11'd0;   // index of the last byte
    reg  [15:0] inj_hdr = 16'd0;    // the event's own length field
    reg        inj_want = 1'b0;     // committed, not yet started
    reg        inj_refuse = 1'b0;   // this run started while one was pending
    assign inj_busy = inj_want || (sending && kind == K_INJECT);

    // On our turn, what goes, in priority order; see the header.
    wire       tick_due  = since_tick >= interval;
    wire       go        = active && our_turn && !sending && turn_wait == {TW{1'b0}};
    wire       go_ble    = go && want_ble;
    wire       go_status = go && !want_ble && want_status;
    wire       go_in0    = go && !want_ble && !want_status && in0_want;
    wire       go_inx    = go && !want_ble && !want_status && !in0_want && inx_want;
    wire       pre_usb   = want_ble || want_status || in0_want || inx_want;
    wire       go_reset  = go && !pre_usb && want_reset;
    wire       go_inject = go && !pre_usb && !want_reset && inj_want;
    wire       pre_btn   = pre_usb || want_reset || inj_want;
    wire       go_button = go && !pre_btn && buttons != reported;
    wire       go_ticker = go && !pre_btn && buttons == reported && tick_due;
    wire       go_any    = go_ble || go_status || go_in0 || go_inx || go_reset
                        || go_inject || go_button || go_ticker;

    wire       handshake = sending && !go_any && tx_valid && tx_ready;
    wire [10:0] inj_ra   = go_inject ? 11'd0
                         : (handshake && idx != last) ? idx + 11'd1 : idx;
    wire [10:0] inj_wa   = inj_w[10:0];

    always @(posedge clk) begin
        if (inj_valid && !inj_w[11] && !inj_want && !inj_refuse) inj_mem[inj_wa] <= inj_data;
        inj_q <= inj_mem[inj_ra];
    end

    always @(posedge clk) begin
        if (!rst_n || stop) begin
            pre         <= {PW{1'b0}};
            ms          <= 32'd0;
            active      <= 1'b0;
            started     <= 1'b0;
            our_turn    <= 1'b0;
            want_status <= 1'b0;
            want_ble    <= 1'b0;
            ble_on      <= 1'b0;
            char_handle <= 8'h0d;
            se_usb      <= 1'b0;
            usb_on      <= 1'b0;
            in0_want    <= 1'b0;
            inx_want    <= 1'b0;
            inj_want    <= 1'b0;
            inj_refuse  <= 1'b0;
            inj_w       <= 12'd0;
            inj_bad     <= 1'b0;
            interval    <= 16'd100;
            since_tick  <= 16'd0;
            reported    <= 2'b00;
            turn_wait   <= {TW{1'b0}};
            rstate      <= R_TAG;
            sending     <= 1'b0;
            tx_valid    <= 1'b0;
            rx_packets  <= 16'd0;
            tx_events   <= 16'd0;
            last_rx_tag <= 8'd0;
        end else if (start) begin
            pre         <= {PW{1'b0}};
            ms          <= 32'd0;
            active      <= 1'b1;
            started     <= 1'b0;
            our_turn    <= 1'b0;
            want_status <= 1'b0;
            want_ble    <= 1'b0;
            ble_on      <= 1'b0;
            char_handle <= 8'h0d;
            se_usb      <= 1'b0;
            usb_on      <= 1'b0;
            in0_want    <= 1'b0;
            inx_want    <= 1'b0;
            inj_want    <= 1'b0;
            inj_refuse  <= 1'b0;
            inj_w       <= 12'd0;
            inj_bad     <= 1'b0;
            interval    <= 16'd100;
            since_tick  <= 16'd0;
            reported    <= 2'b00;
            turn_wait   <= {TW{1'b0}};
            rstate      <= R_TAG;
            sending     <= 1'b1;
            kind        <= K_SESSION;
            idx         <= 11'd0;
            last        <= session_last;
            tx_valid    <= 1'b0;
            rx_packets  <= 16'd0;
            tx_events   <= 16'd0;
            last_rx_tag <= 8'd0;
        end else begin
            // Milliseconds, and how many since the last ticker.
            if (active) begin
                if (ms_pulse) begin
                    pre <= {PW{1'b0}};
                    ms  <= ms + 32'd1;
                    if (since_tick != 16'hFFFF) since_tick <= since_tick + 16'd1;
                end else begin
                    pre <= pre + {{(PW - 1){1'b0}}, 1'b1};
                end
            end

            // Receiving.
            if (rx_valid && active) begin
                case (rstate)
                    R_TAG:  begin rtag <= rx_data; rstate <= R_LENH; end
                    R_LENH: begin rlenh <= rx_data; rstate <= R_LENL; end
                    R_LENL: begin
                        rlen  <= {rlenh, rx_data};
                        rleft <= {rlenh, rx_data};
                        rpos  <= 16'd0;
                        rstate <= ({rlenh, rx_data} == 16'd0) ? R_TAG : R_BODY;
                    end
                    default: begin
                        if (rpos == 16'd0) p0 <= rx_data;
                        if (rpos == 16'd1) p1 <= rx_data;
                        if (rpos == 16'd2) p2 <= rx_data;
                        if (rpos == 16'd23) p23 <= rx_data;
                        rpos  <= rpos + 16'd1;
                        rleft <= rleft - 16'd1;
                        if (rleft == 16'd1) rstate <= R_TAG;
                    end
                endcase
            end

            // A packet ended: note what it asks for.
            if (done) begin
                rx_packets  <= rx_packets + 16'd1;
                last_rx_tag <= rtag;
                case (rtag)
                    8'h4e: if (d_len >= 16'd2 && {d_p0, d_p1} != 16'd0)
                               interval <= {d_p0, d_p1};
                    8'h52: want_status <= 1'b1;
                    // USB_CONFIG: 01 connect, 02 disconnect.
                    8'h4f: if (d_len >= 16'd1) begin
                               if (d_p0 == 8'h01) se_usb <= 1'b1;
                               if (d_p0 == 8'h02) begin
                                   se_usb <= 1'b0;
                                   usb_on <= 1'b0;
                               end
                           end
                    // USB_EP_PREPARE: endpoint, type, length. Type 0x20
                    // hands the host IN data.
                    8'h50: if (d_len >= 16'd3 && d_p1 == 8'h20) begin
                               if (d_p0[6:0] == 7'd0) begin
                                   in0_want <= 1'b1;
                                   in0_len  <= d_p2;
                               end else begin
                                   inx_want <= 1'b1;
                                   inx_ep   <= d_p0 | 8'h80;
                               end
                           end
                    8'h38: if (d_len >= 16'd2) begin
                               want_ble <= 1'b1;
                               ble_op   <= {d_p0, d_p1};
                               ble_props <= d_p23;
                           end
                    default: ;
                endcase
                if (rtag[7:4] == 4'h6) begin
                    our_turn  <= 1'b1;
                    started   <= 1'b1;
                    turn_wait <= TURN_LOAD;
                end
            end

            if (turn_wait != {TW{1'b0}} && !done)
                turn_wait <= turn_wait - {{(TW - 1){1'b0}}, 1'b1};

            // Our turn: start one event.
            if (go_any) begin
                our_turn <= 1'b0;
                sending  <= 1'b1;
                idx      <= 11'd0;
                if (go_ble) begin
                    want_ble <= 1'b0;
                    ble_on   <= 1'b1;
                    kind     <= K_BLE;
                    bop      <= ble_op;
                    bhandle  <= char_handle;
                    // A characteristic takes a declaration and a value
                    // handle, and a third for its client configuration
                    // descriptor if it can notify or indicate (0x10, 0x20).
                    // The capture's three came back 0d, 10, 12 for
                    // properties 10, 08, 04; adding 2 every time gave 0f
                    // and 11, which an SE would have used for the wrong
                    // attributes.
                    if (ble_op == 16'hfd04)
                        char_handle <= char_handle + ((ble_props & 8'h30) != 8'd0 ? 8'd3 : 8'd2);
                    last     <= 11'd9 + {7'd0, ble_ret_len(ble_op)};
                end else if (go_status) begin
                    want_status <= 1'b0;
                    kind        <= K_STATUS;
                    last        <= status_last;
                end else if (go_in0) begin
                    in0_want <= 1'b0;
                    kind     <= K_IN_DONE;
                    bop      <= {8'h80, in0_len};
                    last     <= 11'd5;
                end else if (go_inx) begin
                    // The capture's endpoint 2 IN completions all carry
                    // length 0, whatever was sent; endpoint 0's carry the
                    // length. This copies both.
                    inx_want <= 1'b0;
                    kind     <= K_IN_DONE;
                    bop      <= {inx_ep, 8'h00};
                    last     <= 11'd5;
                end else if (go_reset) begin
                    usb_on   <= 1'b1;
                    kind     <= K_USB_RESET;
                    last     <= 11'd3;
                end else if (go_inject) begin
                    inj_want <= 1'b0;
                    kind     <= K_INJECT;
                    last     <= inj_len1;
                end else if (go_button) begin
                    reported <= buttons;
                    bsnap    <= buttons;
                    kind     <= K_BUTTON;
                    last     <= 11'd3;
                end else begin
                    since_tick <= 16'd0;
                    snap       <= ms;
                    kind       <= K_TICKER;
                    last       <= 11'd6;
                end
            end

            // The cable out: the host is gone, and the next plug resets
            // the bus again.
            if (!usb_plugged) usb_on <= 1'b0;

            // The injected event's buffer. A run that starts while one is
            // still pending is refused whole rather than mixed into it.
            inj_bad <= 1'b0;
            if (inj_start) begin
                inj_refuse <= inj_busy;
                if (!inj_busy) inj_w <= 12'd0;
            end else if (inj_valid && !inj_refuse && !inj_want && !inj_w[11]) begin
                inj_w <= inj_w + 12'd1;
                if (inj_w == 12'd1) inj_hdr[15:8] <= inj_data;
                if (inj_w == 12'd2) inj_hdr[7:0]  <= inj_data;
            end
            // **Refused unless it is one well-formed packet**: a tag, a
            // length, and exactly that many bytes. Anything else would put
            // the link out of step with the SE for good.
            if (inj_commit) begin
                inj_refuse <= 1'b0;
                if (inj_refuse || !active || inj_w < 12'd3 || inj_w[11]
                    || inj_hdr != {4'd0, inj_w} - 16'd3) begin
                    inj_bad <= 1'b1;
                end else begin
                    inj_want <= 1'b1;
                    inj_len1 <= inj_w[10:0] - 11'd1;
                end
            end

            // Sending: one byte at a time through the handshake.
            if (sending && !go_any) begin
                if (!tx_valid) begin
                    tx_data  <= (kind == K_INJECT) ? inj_q
                              : byte_at(kind, idx[6:0], snap, bsnap, bop, brl, bhandle, flags_low);
                    tx_valid <= 1'b1;
                end else if (tx_ready) begin
                    tx_valid <= 1'b0;
                    if (idx == last) begin
                        sending   <= 1'b0;
                        tx_events <= tx_events + 16'd1;
                    end else begin
                        idx <= idx + 11'd1;
                    end
                end
            end
        end
    end
endmodule
