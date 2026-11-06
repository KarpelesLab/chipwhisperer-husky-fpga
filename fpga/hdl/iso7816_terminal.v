// A working ISO 7816 terminal on a Digilent Basys 3: activate a card, read
// its answer, negotiate a faster rate, then talk to it — all from a host
// over the board's serial port.
//
// `iso7816_probe.v` beside this one only listens, which was enough to prove
// the wiring, the clock, the supply switch, the reset timing and the
// character framing against a real device. This one transmits as well, so
// it can do the two things that needed a transmitter: the PPS exchange that
// raises the rate, and sending command bytes afterwards.
//
// **This module has no serial port of its own.** It takes received bytes
// and gives characters to send, and something above it owns the pins:
// `iso7816_terminal_pad.v` for a board that does nothing else, or
// `iso_display.v` where this core and `ssd1306_console.v` share one port
// through an arbiter. That split exists because the board has one serial
// port and a card that must be initialised and then driven cannot be split
// across two bitstreams — reloading the part drops `vcc_en` and
// power-cycles the device.
//
// ===================================================================
// WIRING
// ===================================================================
//
//   JXADC1  J3   io         the card's single line, open drain, with an
//                           external 1k pull-up to 3.3 V
//   JXADC2  L3   clk_card   112 MHz / CARD_DIV; /14 is exactly 8 MHz
//   JXADC3  M2   rst_card   active low
//   JXADC4  N2   vcc_en     an AQV210 PhotoMOS relay's LED, through a
//                           series resistor
//   JXADC5  —    GND
//
// ===================================================================
// THE CONVERSATION
// ===================================================================
//
//   A          activate: power, clock, hold reset 400 card clocks, release
//   P          send PPS `FF 10 97 78` (`PPS1`). The card's echo comes back as hex
//              like anything else; **the host compares it**, and sends `F`
//              if it agrees.
//   F          switch to the fast rate. `S` switches back.
// A byte printed with `>` in front came from the monitor, a second receiver
// on this contact that never drives it -- so it is what this terminal itself
// put on the wire. A byte with no prefix came from the card. `!` means the
// parity was wrong, whichever receiver saw it.
//
//   L          drive the line low for 18 ms, read it, release, read again:
//              `+PAD` if it went low and came back high, `-PAD` otherwise.
//              This is the only check that the pad drives at all, as
//              opposed to this design believing it transmitted.
//   :<hex>     send those bytes to the card. `:00A4040C` sends four. Hex
//              until anything that is not a hex digit ends it, so a newline
//              is a fine terminator.
//   D          deactivate: reset, clock, line, then power — in that order
//   s          status
//
// `A`, `C`, `D` and `F` are themselves hex digits, which is why sending
// bytes needs the `:` prefix: inside a `:` run every character is data,
// outside it every character is a command. Without that, `:0A` and a
// request to activate would be indistinguishable.
//
// **That rule is enforced, and for a while it was not.** Every command
// decoder used to look at the received byte alone, so the uppercase digits
// inside a run ran their commands as well — see the gate near the top of
// the module for what that did and how it was found. A run ends at the
// first character that is not a hex digit, and that character is the
// terminator and nothing else: end a run with CR, LF or a space.
//
// Everything the card says comes back as two hex characters per byte, with
// `!` before a byte whose parity was wrong, and a CRLF once the line has
// been idle for `GAP_ETU` etu. Banners mark what the terminal did:
//
//   +VCC +CLK +RST   the activation sequence, step by step
//   +FST             the rate is now the fast one, from `F`
//   +SLW             back to the default, from `S`
//   -OFF             deactivated
//
// ===================================================================
// THE RATE, AND WHY BOTH VALUES ARE EXACT
// ===================================================================
//
// An etu is `F/D` card clock cycles, so in system cycles it is
// `(F/D) * CARD_DIV` — an integer at every rate this board offers, because
// 112 MHz was chosen so that every card frequency is an even integer
// division of it:
//
//   /14 = 8 MHz   /16 = 7   /20 = 5.6   /28 = 4
//   /32 = 3.5     /56 = 2   /64 = 1.75  /112 = 1
//
// Before PPS, `F/D` is 372: etu = 372 * 14 = **5208** system cycles, which
// is 21505 baud. After the PPS this sends, `F/D` is 8: etu = 8 * 14 =
// **112**, which is 1 Mbaud. Changing `CARD_DIV` changes both rates together and
// keeps both exact, which is the whole reason the etu is expressed in card
// cycles rather than in a baud number.
//
// **The card offers `TA1 = 87`, and this asks for `97`.** `DI = 7` is Di =
// 64 as the standard says, but **`FI = 8` is RFU** in ISO 7816-3's Fi table;
// the device's own documentation and its owner make it `F/D = 4`, 2 Mbaud.
// That was the first PPS this sent, and at 2 Mbaud a card byte lands every
// 6 us while printing one costs 10 us on the 2 Mbaud host port, so a long
// reply overflowed the one-byte buffer below. `97` is the standard's own
// pair — FI = 9 is Fi = 512, so `F/D = 8` — and the owner's word is that the
// card accepts it and talks at 1 Mbaud, a byte every 12 us, which the host
// port keeps up with. `PPS1` and `FAST_ETU_CYCLES` are parameters, and they
// must agree: `87` with 4, `97` with 8.
//
// ===================================================================
// WHAT IS CHECKED
// ===================================================================
//
// The activation sequence, the 8E2 framing and the receive path are CHECKED
// against the owner's own device — `docs/fpga-xray.md` has the measurement,
// including the fourteen-byte answer-to-reset it produced. **The transmit
// path and the PPS exchange have not run against a card yet**; they are
// verified in simulation against a model that echoes a correct PPS and,
// separately, one that does not.

module iso7816_terminal #(
    // 112 MHz / CARD_DIV. 14 is 8 MHz, even so the duty is exactly 50 %.
    parameter CARD_DIV         = 14,
    // F/D before PPS. 372 is the standard default.
    parameter ETU_CYCLES       = 372,
    // F/D after this card's PPS, and the PPS1 that asks for it; see above.
    // They must agree: `8'h97` is F/D = 8, `8'h87` is F/D = 4 on this card.
    parameter FAST_ETU_CYCLES  = 8,
    parameter [7:0] PPS1       = 8'h97,
    // System clocks per millisecond for `W`'s SEPROXYHAL engine: 112 000
    // at 112 MHz. A testbench shrinks it so that a ticker is not 100 ms.
    parameter integer SEPH_MS_CYCLES = 112000,
    // And between the end of the SE's status and the engine's answer:
    // 11 200 is 100 us at 112 MHz, what a real MCU leaves.
    parameter integer SEPH_TURN_CYCLES = 11200,
    parameter RST_HOLD         = 400,
    parameter VCC_BITS         = 20,
    parameter GAP_ETU          = 256,
    // No `HOST_DIV` here any more: the serial port moved out, so the rate
    // it runs at is the business of whoever instantiates one.
    parameter WDOG_BITS        = 31
) (
    // The system clock, 112 MHz on the board. The top makes it, because a
    // PLL request is only honoured on the top module's nets.
    input  wire        clk,
    // High once that clock is trustworthy: the top wires the PLL's own
    // `LOCKED` here, and a testbench ties it high.
    input  wire        locked,

    output wire        clk_card,
    output wire        rst_card,
    output wire        vcc_en,

    // **The pad's three wires, not an `inout`.** Tristate belongs at the
    // top level: `examples/basys3/iso7816_terminal_pad.v` is the thin
    // wrapper that turns these into one bidirectional ball.
    //
    // Keeping it out of here is what makes the block testable. A testbench
    // that wants two devices on one open-drain wire cannot use two tristate
    // drivers — `reticle sim` refuses a net driven from more than one place,
    // and rightly, since without a pull-up two released drivers are X. With
    // `io_oe`/`io_o` exposed, a testbench models the bus as the wired-AND
    // that a pull-up physically is, and two devices can share it.
    input  wire        io_i,
    output wire        io_oe,
    output wire        io_o,

    // **Bytes, not pins.** The serial port itself is one level up, for the
    // same reason the tristate is: a board has one serial port and
    // `iso_display.v` puts two of these cores on it. So this module takes a
    // received-byte stream and gives a transmit stream, and whatever owns
    // the port decides who gets to speak.
    //
    // `cmd_valid` is a one-cycle strobe with no back pressure, which is
    // what `ip/bus/uart`'s receiver gives: a byte is offered for one cycle
    // and lost if nobody looks. Every user of it here looks on the cycle it
    // arrives.
    //
    // `out_valid` is held with `out_data` stable until a cycle in which
    // `out_ready` is also high — not pulsed, because the cycle a pulse
    // appears in is not the cycle its ready was read in.
    input  wire        cmd_valid,
    input  wire [7:0]  cmd_data,
    // The device's two buttons as `W`'s engine reports them to the SE:
    // bit 0 left, bit 1 right, high while pressed.
    input  wire [1:0]  seph_buttons,
    // The device's two button lines **as read**, already synchronised:
    // bit 0 left, bit 1 right. Reported in `s`, so a host can see each line's
    // idle level and which way a press moves it before anything drives one.
    input  wire [1:0]  buttons_in,
    output wire        out_valid,
    output wire [7:0]  out_data,
    input  wire        out_ready,

    // High while a `:` run is being read, so that whoever shares the port
    // can hold its other users off the characters that belong to the card.
    output wire        hex_run,

    output wire [14:0] led,
    output wire [6:0]  seg,
    output wire        dp,
    output wire [3:0]  an,

    // ---- Husky port taps (added for the register-bus wrapper) ----
    output wire [7:0]  o_card_rx,       // raw card byte (ATR / SE->MCU)
    output wire        o_card_rx_valid, // 1-cycle strobe
    output wire [2:0]  o_state,         // S_IDLE/VCC/CLK/RUN/DOWN/OFF
    output wire        o_card_active,
    output wire        o_seph_started,
    output wire        o_seph_active,
    output wire [7:0]  o_seph_last_tag,
    output wire [15:0] o_card_bytes,
    output wire        o_inj_trig,      // seph_mcu: high while an injected event is on the wire
    output wire        o_inj_commit     // seph_mcu: 1-cycle pulse when an inject is committed
);
    assign o_card_rx       = card_rx;
    assign o_card_rx_valid = card_rx_valid;
    assign o_state         = state;
    assign o_card_active   = card_active;
    assign o_seph_started  = seph_started;
    assign o_seph_active   = seph_active;
    assign o_seph_last_tag = seph_last_tag;
    assign o_card_bytes    = bytes_q;
    assign o_inj_trig      = seph_inj_trig;
    assign o_inj_commit    = inj_commit;
    localparam integer SLOW_DIV = ETU_CYCLES * CARD_DIV;
    localparam integer FAST_DIV = FAST_ETU_CYCLES * CARD_DIV;
    localparam [15:0]  SLOW_DIV16 = SLOW_DIV;
    localparam [15:0]  FAST_DIV16 = FAST_DIV;
    localparam [7:0]   HALF_LAST  = (CARD_DIV / 2) - 1;
    localparam [15:0]  CARD_DIV16 = CARD_DIV;

    // **No PLL here.** `clock_mhz` and `clock_locked` are requests to the
    // backend and are only honoured on the **top** module's nets — wrapping
    // this core in `iso7816_terminal_pad` moved them into a submodule and
    // the build failed with `core.clk_pll` driven by nothing. So the top
    // owns the clocking as well as the pads, and this module is pure logic
    // with a clock input, which is the better split anyway: it is what lets
    // a testbench drive it from an ordinary clock with no PLL in sight.
    wire sys = clk;

    reg [15:0] por = 16'h0000;
    always @(posedge sys) por <= {por[14:0], locked};
    wire rst_n = por[15];

    // =================================================================
    // Inside a `:` run, every character is data
    // =================================================================
    //
    // **This is a gate on every command, and it was missing.** The header
    // has always said that `A`, `C`, `D` and `F` being hexadecimal digits is
    // why sending bytes needs a `:` prefix — inside a run every character is
    // data, outside it every character is a command — but only the byte
    // assembler below honoured it. Every command decoder looked at
    // `cmd_data` alone, so the uppercase digits in a run were **also**
    // executed: `:00A4040C` ran `C`, which drives the contact low for 18 ms
    // and overwrote the byte going out on the wire — the model card
    // received `00 A4 04 00`. That is how it was found: by a testbench that
    // drives this half and the display console on one serial port.
    //
    // `D` is the one that matters: a `D` nibble in an APDU would have
    // deactivated the card mid-frame, dropping `vcc_en` and power-cycling
    // whatever is on the other end. `F` and `S` would have changed the rate
    // mid-frame. Lowercase hexadecimal collides with nothing, which is
    // probably why a 51-byte frame went out on real hardware intact.
    //
    // The run's **terminating** character is data too, by the same rule:
    // `in_hex` is still high on the cycle it arrives, so it closes the run
    // and does nothing else. End a run with CR, LF or a space and then send
    // the next command.
    //
    // `in_hex` is exported so that `iso_display.v` can hold the display half
    // off for exactly the same characters, with the same bit rather than a
    // second copy of this state.
    reg       in_hex   = 1'b0;      // inside a `:` or `!` run
    reg       run_seph = 1'b0;      // ... and it is a `!` run
    assign    hex_run  = in_hex;
    // Every command below is decoded through this and never through
    // `cmd_valid` alone.
    wire      cmd_now  = cmd_valid && !in_hex;

    // =================================================================
    // `L`: can this pad actually pull the line low?
    // =================================================================
    //
    // **This exists because `sent_q` is not evidence.** The terminal
    // counted four PPS bytes transmitted with no errors and the card
    // answered nothing -- but that counter reflects this design's own state
    // machine, not the wire. Receiving is proven on silicon by the ATR;
    // driving never has been, and it is the one link in the chain with no
    // measurement behind it.
    //
    // Against the line's 1 k pull-up the test is unambiguous: drive low and
    // the line must read low, release and it must read high. A pad that
    // cannot do that explains the silence completely, and one that can
    // moves the question to the card.
    localparam [23:0] PAD_HOLD = 24'd2_000_000;   // ~18 ms at 112 MHz

    reg        pad_drive = 1'b0;
    reg [1:0]  pad_step  = 2'd0;
    reg [23:0] pad_t     = 24'd0;
    reg        pad_lo    = 1'b1;   // the level read while driving low
    reg        pad_hi    = 1'b0;   // and after releasing

    wire blk_oe, blk_o;
    // `pad_const` asserts the enable while leaving the data input on the
    // **bare constant** leg, which is what the block's own transmission
    // uses. `pad_drive` instead selects a `1'b0` through this mux, making
    // the data input a LUT output -- genuinely driven. The two differ in
    // exactly one thing: whether anything drives the pad's data input.
    //
    // That matters because `techcells::drive_constant_data` collects pins
    // only from flip-flop, LUT-RAM and carry bels; `BelRole::Io` is absent,
    // so a constant feeding an `IOBUF`'s data input has no driver at all.
    // If an undriven input reads one, the pad drives the contact high when
    // enabled -- indistinguishable, on an open-drain line with a pull-up,
    // from releasing it. `L` drives low and worked; this says whether the
    // constant leg does too.
    assign io_oe = blk_oe | pad_drive | pad_const;
    assign io_o  = pad_drive ? 1'b0 : blk_o;

    // `C`: the same two samples as `L`, but with the data input left on the
    // bare constant.
    reg        pad_const = 1'b0;
    reg [1:0]  cst_step  = 2'd0;
    reg [23:0] cst_t     = 24'd0;
    reg        cst_lo    = 1'b1;
    reg        cst_hi    = 1'b0;

    always @(posedge sys) begin
        if (!rst_n) begin
            pad_const <= 1'b0;
            cst_step  <= 2'd0;
            cst_t     <= 24'd0;
        end else case (cst_step)
            2'd0: if (cmd_now && cmd_data == 8'h43) begin     // 'C'
                pad_const <= 1'b1;
                cst_t     <= PAD_HOLD;
                cst_step  <= 2'd1;
            end
            2'd1: begin
                cst_t <= cst_t - 1'b1;
                if (cst_t == 24'd1) begin
                    cst_lo    <= io_i;
                    pad_const <= 1'b0;
                    cst_t     <= PAD_HOLD;
                    cst_step  <= 2'd2;
                end
            end
            2'd2: begin
                cst_t <= cst_t - 1'b1;
                if (cst_t == 24'd1) begin
                    cst_hi   <= io_i;
                    cst_step <= 2'd3;
                end
            end
            2'd3: cst_step <= 2'd0;
        endcase
    end

    wire cst_done = (cst_step == 2'd3);
    // `+CST` only if the constant leg pulled the contact low and let it rise.
    wire cst_good = (cst_lo == 1'b0) && (cst_hi == 1'b1);

    always @(posedge sys) begin
        if (!rst_n) begin
            pad_drive <= 1'b0;
            pad_step  <= 2'd0;
            pad_t     <= 24'd0;
        end else case (pad_step)
            2'd0: if (cmd_now && cmd_data == 8'h4C) begin     // 'L'
                pad_drive <= 1'b1;
                pad_t     <= PAD_HOLD;
                pad_step  <= 2'd1;
            end
            2'd1: begin
                pad_t <= pad_t - 1'b1;
                if (pad_t == 24'd1) begin
                    pad_lo    <= io_i;      // sampled while still driving
                    pad_drive <= 1'b0;
                    pad_t     <= PAD_HOLD;
                    pad_step  <= 2'd2;
                end
            end
            2'd2: begin
                pad_t <= pad_t - 1'b1;
                if (pad_t == 24'd1) begin
                    pad_hi   <= io_i;       // and after letting it rise
                    pad_step <= 2'd3;
                end
            end
            2'd3: pad_step <= 2'd0;         // the banner is raised below
        endcase
    end

    // `+PAD` only when it went low under drive and high when released.
    wire pad_done = (pad_step == 2'd3);
    wire pad_good = (pad_lo == 1'b0) && (pad_hi == 1'b1);

    // =================================================================
    // A monitor on the same contact
    // =================================================================
    //
    // **Reusing the receiver a real card already validated, instead of
    // instrumentation written this afternoon.** A second `iso7816_uart`
    // that never transmits -- `tx_valid` tied low, its `io_oe` and `io_o`
    // left unconnected, so it cannot touch the wire. It decodes everything
    // that appears there, including this terminal's own transmission, which
    // the main instance suppresses from its own receiver.
    //
    // It answers the question a trace buffer was being built for: are the
    // four PPS characters well formed on the wire? And it answers with a
    // block that has conformance evidence -- it read a real card's 14-byte
    // ATR with no parity or framing error -- rather than with new logic
    // that had none. The trace buffer it replaces could not be checked in
    // simulation, because its sample interval was written out instead of
    // derived; it then failed on the board for a third reason, an edit that
    // silently never applied, leaving the thing that requested the dump
    // declared, read, and never assigned. "The instrument is broken" and
    // "the line never moved" look identical from the far end of a serial
    // port, and telling them apart cost two board runs.
    //
    // Its bytes print with `>` in front and the card's with nothing, so a
    // single line says who drove the contact.
    wire [7:0] mon_rx;
    wire       mon_rx_valid, mon_parity_err;

    iso7816_uart #(.ETU_DIV(SLOW_DIV), .PARITY_RETRY(0)) monitor (
        .clk(sys), .rst_n(rst_n),
        .active(card_active),
        .etu_div(etu_div),
        .guard_etu(8'd0),
        .convention(1'b0),
        .wt_etu(24'd0),                   // it watches; it never waits
        .tx_data(8'd0), .tx_valid(1'b0), .tx_ready(),
        .tx_abort(),
        .rx_data(mon_rx), .rx_ready(1'b1), .rx_valid(mon_rx_valid),
        .rx_parity_error(mon_parity_err), .rx_overrun(),
        .rx_timeout(),
        .io_i(io_i), .io_oe(), .io_o(),
        .tx_char_count(), .rx_char_count(), .parity_error_count(),
        .repeat_count(), .timeout_count());

    // =================================================================
    // The card clock
    // =================================================================

    reg [7:0] cdiv   = 8'd0;
    reg       cclk   = 1'b0;
    reg       crise  = 1'b0;
    reg       clk_on = 1'b0;

    always @(posedge sys) begin
        crise <= 1'b0;
        if (!rst_n || !clk_on) begin
            cdiv <= 8'd0;
            cclk <= 1'b0;
        end else if (cdiv == HALF_LAST) begin
            cdiv  <= 8'd0;
            cclk  <= ~cclk;
            crise <= ~cclk;
        end else begin
            cdiv <= cdiv + 8'd1;
        end
    end

    assign clk_card = cclk;

    // =================================================================
    // The host's byte streams
    // =================================================================
    //
    // The emitter below was written against a `uart` instance's
    // `tx_data`/`tx_valid`/`tx_ready`, and these three lines are all that
    // is left of it: the port moved out, the handshake did not change.
    reg  [7:0] host_data  = 8'd0;
    reg        host_valid = 1'b0;

    assign out_data  = host_data;
    assign out_valid = host_valid;
    wire   host_ready = out_ready;

    function is_hex_char;
        input [7:0] c;
        is_hex_char = (c >= 8'd48 && c <= 8'd57)      // 0-9
                   || (c >= 8'd65 && c <= 8'd70)      // A-F
                   || (c >= 8'd97 && c <= 8'd102);    // a-f
    endfunction

    // Through an eight-bit local and then a slice: `c - 8'd48` is eight
    // bits wide and assigning it straight to a four-bit result truncates,
    // which the simulator rightly warns about even though the value always
    // fits.
    function [3:0] hex_val;
        input [7:0] c;
        reg [7:0] sub;
        begin
            sub = (c <= 8'd57) ? (c - 8'd48)
                : (c <= 8'd70) ? (c - 8'd55)
                :                (c - 8'd87);
            hex_val = sub[3:0];
        end
    endfunction

    // =================================================================
    // The card's line
    // =================================================================

    reg  [15:0] etu_div = SLOW_DIV16;
    reg         card_active = 1'b0;

    wire [7:0]  card_rx;
    wire        card_rx_valid, card_parity_err, card_timeout;
    wire        card_tx_ready;
    reg  [7:0]  card_tx = 8'd0;
    reg         card_tx_valid = 1'b0;
    // **`PARITY_RETRY = 0`: this terminal never signals a parity error.**
    //
    // T=0 lets a receiver report a bad character by pulling the contact low
    // for 1-2 etu at 10.5 etu, and the block implements it. On this link it
    // is actively harmful, and a 200 MHz capture of the contact shows why:
    // after the card's `3B 1B` came a low pulse 11.48 etu later -- exactly
    // an error signal's position -- and from there the ATR read
    // `3B 1B FF 1D EE 2B FE ...` against the `3B 1B 87 05 32 2E ...` the
    // card sent. The pulse lands **while the card is still transmitting**,
    // so one bad character becomes every character after it. The board's
    // owner confirmed the pulses were ours.
    //
    // Nothing here needs it: a character the terminal mishears is asked for
    // again by the layer above, which is a host's business and not this
    // module's. The monitor instance keeps it off for the same reason, and
    // because an instance that cannot drive cannot corrupt what it watches.
    iso7816_uart #(.ETU_DIV(SLOW_DIV), .PARITY_RETRY(0)) line (
        .clk(sys), .rst_n(rst_n),
        .active(card_active),
        .etu_div(etu_div),
        .guard_etu(8'd0),
        .convention(1'b0),            // direct: TS = 3B
        .wt_etu(24'd9600),            // T=0 work waiting time
        .tx_data(card_tx), .tx_valid(card_tx_valid), .tx_ready(card_tx_ready),
        .tx_abort(),
        .rx_data(card_rx), .rx_ready(1'b1), .rx_valid(card_rx_valid),
        .rx_parity_error(card_parity_err), .rx_overrun(),
        .rx_timeout(card_timeout),
        .io_i(io_i), .io_oe(blk_oe), .io_o(blk_o),
        .tx_char_count(), .rx_char_count(), .parity_error_count(),
        .repeat_count(), .timeout_count());

    // =================================================================
    // `W`: this terminal becomes the SE's MCU
    // =================================================================
    //
    // A Nano X's secure element is the card on this contact, and once the
    // PPS has set the rate it expects an MCU to talk SEPROXYHAL to it.
    // `ip/bus/seph_mcu` is that MCU: `W` starts it, it sends
    // SESSION_START and from then on answers every turn the SE ends --
    // tickers, status, BLE command completes, button changes -- with no
    // host involved. The card's bytes are still printed (`m` mutes them),
    // and its own bytes come back with `>` through the monitor like any
    // other transmission.
    //
    // **`W` only after `F`.** The engine does not know about rates; it
    // starts talking at whatever `etu_div` is. A host must check the PPS
    // echo and switch first, exactly as before. It stops the moment the
    // card is deactivated, by `D` or by the watchdog.
    wire [7:0]  seph_tx_data;
    wire        seph_tx_valid, seph_active, seph_started;
    wire        seph_tx_ready;
    wire [15:0] seph_rx_packets, seph_tx_events;
    wire [7:0]  seph_last_tag;
    wire        seph_start = cmd_now && cmd_data == 8'h57 && card_active;   // 'W'

    // `U` puts a USB cable in and `X` takes it out. Nothing is on the other
    // end but a host at this console: the engine does the MCU's part --
    // the bus reset, IN completions, the status flags -- and a `!` run
    // carries the host's own transfers. See `seph_mcu.v`, "USB".
    // The `:` and `!` runs' half-assembled byte, declared here because the
    // engine below takes `!` bytes from it.
    reg       have_hi  = 1'b0;
    reg [3:0] hi_nib   = 4'd0;
    reg         usb_plugged = 1'b0;
    wire        seph_usb_on, seph_inj_busy, seph_inj_bad, seph_inj_trig;
    wire        inj_start, inj_valid, inj_end, inj_commit, inj_odd;
    always @(posedge sys) begin
        if (!rst_n)                                usb_plugged <= 1'b0;
        else if (cmd_now && cmd_data == 8'h55)     usb_plugged <= 1'b1;   // 'U'
        else if (cmd_now && cmd_data == 8'h58)     usb_plugged <= 1'b0;   // 'X'
    end

    seph_mcu #(.MS_CYCLES(SEPH_MS_CYCLES), .TURN_CYCLES(SEPH_TURN_CYCLES)) seph (
        .clk(sys), .rst_n(rst_n),
        .start(seph_start), .stop(!card_active),
        .rx_data(card_rx), .rx_valid(card_rx_valid),
        .tx_data(seph_tx_data), .tx_valid(seph_tx_valid), .tx_ready(seph_tx_ready),
        .buttons(seph_buttons),
        .usb_plugged(usb_plugged), .usb_on(seph_usb_on),
        .inj_start(inj_start), .inj_data({hi_nib, hex_val(cmd_data)}),
        .inj_valid(inj_valid), .inj_commit(inj_commit),
        .inj_busy(seph_inj_busy), .inj_bad(seph_inj_bad), .inj_trig(seph_inj_trig),
        .active(seph_active), .started(seph_started),
        .rx_packets(seph_rx_packets), .tx_events(seph_tx_events),
        .last_rx_tag(seph_last_tag), .ms());

    // =================================================================
    // Sending bytes: a `:` run from the host, or the PPS
    // =================================================================

    localparam [1:0] SRC_IDLE = 2'd0, SRC_HOST = 2'd1, SRC_PPS = 2'd2,
                     SRC_SEPH = 2'd3;

    reg [1:0] tx_src   = SRC_IDLE;
    reg [1:0]  pps_idx = 2'd0;
    // Four etu of silence between the characters of a sequence this module
    // sends itself, which is comfortably more than the two the standard
    // requires and costs nothing at either rate.
    reg [19:0] tx_wait = 20'd0;
    wire [19:0] gap_etu_4 = {etu_div, 2'd0};

    // `FF 10 97 78`: PPSS, then PPS0 saying PPS1 follows and the protocol
    // is T=0, then PPS1 carrying the Fi/Di asked for, then the check byte,
    // which is the exclusive-or of the three before it — computed, so that
    // changing `PPS1` cannot leave a stale one.
    function [7:0] pps_byte;
        input [1:0] at;
        case (at)
            2'd0: pps_byte = 8'hFF;
            2'd1: pps_byte = 8'h10;
            2'd2: pps_byte = PPS1;
            default: pps_byte = 8'hFF ^ 8'h10 ^ PPS1;
        endcase
    endfunction

    // What the card must echo, and how much of it has matched so far.
    reg       pps_armed = 1'b0;

    always @(posedge sys) begin
        if (!rst_n) begin
            in_hex  <= 1'b0;
            have_hi <= 1'b0;
        end else if (cmd_valid) begin
            if (cmd_data == 8'h3A) begin        // ':'
                in_hex   <= 1'b1;
                run_seph <= 1'b0;
                have_hi  <= 1'b0;
            end else if (cmd_data == 8'h21 && !(in_hex && run_seph)) begin   // '!'
                in_hex   <= 1'b1;
                run_seph <= 1'b1;
                have_hi  <= 1'b0;
            end else if (in_hex && !is_hex_char(cmd_data)) begin
                in_hex  <= 1'b0;
                have_hi <= 1'b0;
            end else if (in_hex && is_hex_char(cmd_data)) begin
                if (!have_hi) begin
                    hi_nib  <= hex_val(cmd_data);
                    have_hi <= 1'b1;
                end else begin
                    have_hi <= 1'b0;
                end
            end
        end
    end

    // A byte is complete when the second nibble of a pair arrives.
    wire host_byte_ready = cmd_valid && in_hex && !run_seph && is_hex_char(cmd_data)
                        && have_hi;

    // A `!` run goes to the engine instead: one whole event, offered when
    // the run ends. An odd number of digits is refused here, since the
    // engine never sees the half byte.
    assign inj_start  = cmd_valid && cmd_data == 8'h21 && !(in_hex && run_seph);
    assign inj_valid  = cmd_valid && in_hex && run_seph && is_hex_char(cmd_data) && have_hi;
    assign inj_end    = cmd_valid && in_hex && run_seph && !is_hex_char(cmd_data);
    assign inj_commit = inj_end && !have_hi;
    assign inj_odd    = inj_end && have_hi;
    wire [7:0] host_byte = {hi_nib, hex_val(cmd_data)};

    // The engine's byte is taken exactly when the branch below that loads
    // it runs: everything ahead of it in that chain must be idle.
    wire seph_take = rst_n && !card_tx_valid && seph_tx_valid
                  && !(tx_src == SRC_PPS && tx_wait != 0)
                  && !(tx_src == SRC_SEPH && tx_wait != 0)
                  && !host_byte_ready
                  && !(pps_armed && tx_src != SRC_PPS);
    assign seph_tx_ready = seph_take;

    // The guard-time countdown has to run while `card_tx_valid` is **low**,
    // which is why it is inside that branch and not after it. The first
    // version put it in a later `else if`, reachable only when valid was
    // high — so after the first byte the timer never decremented and the
    // sequence stalled. The testbench reported only "did not finish",
    // which is why `card_send` there is now bounded and says why it gave up.
    always @(posedge sys) begin
        if (!rst_n) begin
            card_tx_valid <= 1'b0;
            tx_src        <= SRC_IDLE;
            pps_idx       <= 2'd0;
            tx_wait       <= 20'd0;
        end else if (!card_tx_valid) begin
            if (tx_src == SRC_PPS && tx_wait != 0) begin
                tx_wait <= tx_wait - 1'b1;
                if (tx_wait == 20'd1) begin
                    card_tx       <= pps_byte(pps_idx);
                    card_tx_valid <= 1'b1;
                    pps_idx       <= (pps_idx == 2'd3) ? 2'd0 : (pps_idx + 2'd1);
                end
            end else if (tx_src == SRC_SEPH && tx_wait != 0) begin
                tx_wait <= tx_wait - 1'b1;
            end else if (host_byte_ready) begin
                card_tx       <= host_byte;
                card_tx_valid <= 1'b1;
                tx_src        <= SRC_HOST;
            end else if (pps_armed && tx_src != SRC_PPS) begin
                card_tx       <= pps_byte(2'd0);
                card_tx_valid <= 1'b1;
                tx_src        <= SRC_PPS;
                pps_idx       <= 2'd1;
            end else if (seph_take) begin
                card_tx       <= seph_tx_data;
                card_tx_valid <= 1'b1;
                tx_src        <= SRC_SEPH;
            end else if (tx_src == SRC_PPS) begin
                // The fourth byte has gone and no wait is pending.
                tx_src <= SRC_IDLE;
            end
        end else if (card_tx_ready) begin
            card_tx_valid <= 1'b0;
            // Four etu of silence before the next character of our own
            // sequence. Presenting it the instant `tx_ready` rises assumes
            // the block enforces the guard, and `pair_tb` shows bytes
            // separated by several etu exchange cleanly while back-to-back
            // ones draw a parity error and a T=0 repeat.
            if (tx_src == SRC_PPS && pps_idx != 2'd0) tx_wait <= gap_etu_4;
            // The engine's bytes are spaced the same way: four etu.
            if (tx_src == SRC_SEPH) tx_wait <= gap_etu_4;
        end
    end

    // **The host judges the echo, not the hardware.**
    //
    // This module used to match the card's PPS response itself and switch
    // the rate when all four bytes agreed. That cost four separate bugs and
    // a day: the echo arrives at the old rate, the switch must happen after
    // the last byte and not before, a mis-received byte makes the terminal
    // signal a parity error that makes the card repeat, and repeats that
    // exhaust leave the card unable to transmit at all. Every one of those
    // is state this module does not need to hold.
    //
    // Instead `P` transmits the four bytes and the echo comes back as hex
    // like anything else, and `F` switches to the fast rate once the host
    // has read it. A host comparing four bytes is three lines of script; the
    // same comparison here was a state machine that could desynchronise
    // from the card in a way neither side could recover from. This is the
    // rule every other design here follows — complexity belongs where it is
    // cheap — and abandoning it for this one was the mistake.
    //
    // Nothing is lost in safety: the rate cannot change by accident, only
    // on an explicit command, and the host that sends it has just seen the
    // echo it is deciding on.
    // 1 +VCC 2 +CLK 3 +RST 4 -OFF 5 +FAST 6 +SLOW 7 +PAD 8 +CST.
    // Four bits because slot 0 means `none`, so eight was already full.
    reg [3:0] banner = 4'd0;

    // =================================================================
    // The sequencer
    // =================================================================

    localparam [2:0] S_IDLE = 3'd0, S_VCC = 3'd1, S_CLK = 3'd2,
                     S_RUN  = 3'd3, S_DOWN = 3'd4, S_OFF = 3'd5;

    reg [2:0]        state  = S_IDLE;
    reg [VCC_BITS:0] settle = {(VCC_BITS + 1){1'b0}};
    reg [15:0]       held   = 16'd0;
    reg              vcc_q  = 1'b0;
    reg              rst_q  = 1'b0;

    reg [WDOG_BITS:0] wdog = {(WDOG_BITS + 1){1'b0}};
    wire wdog_bite = (state == S_RUN || state == S_VCC || state == S_CLK)
                   && (wdog == 0);

    always @(posedge sys) begin
        if (!rst_n) wdog <= {(WDOG_BITS + 1){1'b0}};
        else if (cmd_valid || card_rx_valid) wdog <= {1'b1, {WDOG_BITS{1'b0}}};
        else if (wdog != 0) wdog <= wdog - 1'b1;
    end

    always @(posedge sys) begin
        if (!rst_n) begin
            state       <= S_IDLE;
            vcc_q       <= 1'b0;
            rst_q       <= 1'b0;
            clk_on      <= 1'b0;
            card_active <= 1'b0;
            settle      <= {(VCC_BITS + 1){1'b0}};
            held        <= 16'd0;
            banner      <= 4'd0;
            etu_div     <= SLOW_DIV16;
            pps_armed   <= 1'b0;
        end else begin
            banner <= 4'd0;      // a pulse, not a level
            case (state)
                S_IDLE:
                    if (cmd_now && cmd_data == 8'h41) begin     // 'A'
                        vcc_q   <= 1'b1;
                        settle  <= {1'b1, {VCC_BITS{1'b0}}};
                        etu_div <= SLOW_DIV16;
                        banner  <= 4'd1;
                        state   <= S_VCC;
                    end
                S_VCC:
                    if (wdog_bite) begin
                        vcc_q  <= 1'b0; clk_on <= 1'b0;
                        banner <= 4'd4; state <= S_OFF;
                    end else if (settle != 0) settle <= settle - 1'b1;
                    else begin
                        clk_on <= 1'b1;
                        held   <= 16'd0;
                        banner <= 4'd2;
                        state  <= S_CLK;
                    end
                S_CLK:
                    if (wdog_bite) begin
                        rst_q <= 1'b0; clk_on <= 1'b0; vcc_q <= 1'b0;
                        banner <= 4'd4; state <= S_OFF;
                    end else if (crise) begin
                        if (held == RST_HOLD - 1) begin
                            rst_q       <= 1'b1;
                            card_active <= 1'b1;
                            banner      <= 4'd3;
                            state       <= S_RUN;
                        end else begin
                            held <= held + 16'd1;
                        end
                    end
                S_RUN: begin
                    if (cmd_now && cmd_data == 8'h50) pps_armed <= 1'b1;
                    if (pps_armed && tx_src == SRC_PPS && pps_idx == 2'd0
                        && !card_tx_valid)
                        pps_armed <= 1'b0;     // all four have gone
                    // The host decides, having seen the echo.
                    if (cmd_now && cmd_data == 8'h46) begin        // 'F'
                        etu_div <= FAST_DIV16;
                        banner  <= 4'd5;
                    end
                    if (cmd_now && cmd_data == 8'h53) begin        // 'S'
                        etu_div <= SLOW_DIV16;
                        banner  <= 4'd6;
                    end
                    if (pad_done) banner <= 4'd7;
                    if (cst_done) banner <= 4'd8;
                    if ((cmd_now && cmd_data == 8'h44) || wdog_bite) begin
                        rst_q       <= 1'b0;
                        clk_on      <= 1'b0;
                        card_active <= 1'b0;
                        settle      <= {1'b1, {VCC_BITS{1'b0}}};
                        state       <= S_DOWN;
                    end
                end
                S_DOWN:
                    if (settle != 0) settle <= settle - 1'b1;
                    else begin
                        vcc_q  <= 1'b0;
                        banner <= 4'd4;
                        state  <= S_OFF;
                    end
                default: state <= S_IDLE;
            endcase
        end
    end

    assign vcc_en   = vcc_q;
    assign rst_card = rst_q;

    // =================================================================
    // Counters
    // =================================================================

    reg [15:0] bytes_q = 16'd0, perr_q = 16'd0, sent_q = 16'd0,
               tmo_q   = 16'd0;
    always @(posedge sys) begin
        if (!rst_n) begin
            bytes_q <= 16'd0; perr_q <= 16'd0; sent_q <= 16'd0;
            tmo_q   <= 16'd0;
        end else begin
            if (card_rx_valid) begin
                if (bytes_q != 16'hFFFF) bytes_q <= bytes_q + 16'd1;
                if (card_parity_err && perr_q != 16'hFFFF)
                    perr_q <= perr_q + 16'd1;
            end
            if (card_tx_valid && card_tx_ready && sent_q != 16'hFFFF)
                sent_q <= sent_q + 16'd1;
            if (card_timeout && tmo_q != 16'hFFFF) tmo_q <= tmo_q + 16'd1;
        end
    end

    // =================================================================
    // Printing
    // =================================================================

    localparam [2:0] P_IDLE = 3'd0, P_BANNER = 3'd1, P_BYTE = 3'd2,
                     P_EOL  = 3'd3, P_STATUS = 3'd4;

    reg [2:0]   phase = P_IDLE;
    reg [3:0]   pos   = 4'd0;
    reg [7:0]   pend_byte = 8'd0;
    reg         pend_bad  = 1'b0;
    reg         pend_mon  = 1'b0;
    // `m` stops card bytes being printed. Counters are untouched.
    //
    // **One bad receiver can starve a shared port.** This half shares
    // one serial port with a display half, and a mis-sampling receiver
    // emits three characters per phantom byte: a real card's 14-byte ATR
    // came back as thirteen hundred characters, which left the arbiter no
    // room, so the display half's status line came back empty and its
    // frame dump truncated to two lines of eight. The counters in `s`
    // still say what arrived; only the hex stops.
    reg         mute      = 1'b0;
    reg         have_byte = 1'b0;
    reg [127:0] status_sh = 128'd0;
    reg [5:0]   status_left = 6'd0;
    reg [31:0]  gap = 32'd0;
    reg         line_open = 1'b0;

    localparam [31:0] GAP_LOAD_SLOW = SLOW_DIV * GAP_ETU;
    // The gap is in etu, so it must shrink with the rate too, or a fast
    // exchange would wait a slow line's worth of time before ending a line.
    wire [31:0] gap_load = (etu_div == FAST_DIV16)
                         ? (FAST_DIV * GAP_ETU)
                         : GAP_LOAD_SLOW;

    reg p_vcc = 1'b0, p_clk = 1'b0, p_rst = 1'b0, p_off = 1'b0,
        p_fast = 1'b0, p_slow = 1'b0, p_pad = 1'b0, p_cst = 1'b0,
        p_inj = 1'b0, p_usb_in = 1'b0, p_usb_out = 1'b0;
    wire [3:0] want_banner = p_vcc ? 4'd1 : p_clk ? 4'd2 : p_rst ? 4'd3
                           : p_fast ? 4'd5 : p_slow ? 4'd6 : p_pad ? 4'd7
                           : p_cst ? 4'd8 : p_off ? 4'd4
                           : p_inj ? 4'd9 : p_usb_in ? 4'd10 : p_usb_out ? 4'd11
                           : 4'd0;

    wire banner_done = (phase == P_BANNER) && (pos == 4'd5) && host_valid && host_ready;
    wire byte_done   = (phase == P_BYTE)   && (pos == 4'd2) && host_valid && host_ready;
    wire eol_done    = (phase == P_EOL)    && (pos == 4'd1) && host_valid && host_ready;

    always @(posedge sys) begin
        if (!rst_n) begin
            p_vcc <= 1'b0; p_clk <= 1'b0; p_rst <= 1'b0; p_off <= 1'b0;
            p_fast <= 1'b0; p_slow <= 1'b0; p_pad <= 1'b0; p_cst <= 1'b0;
            p_inj <= 1'b0; p_usb_in <= 1'b0; p_usb_out <= 1'b0;
        end else begin
            // `-INJ`: a `!` run the engine refused, or an odd digit count.
            if (seph_inj_bad || inj_odd)                 p_inj     <= 1'b1;
            if (cmd_now && cmd_data == 8'h55)            p_usb_in  <= 1'b1;
            if (cmd_now && cmd_data == 8'h58)            p_usb_out <= 1'b1;
            case (banner)
                4'd1: p_vcc <= 1'b1;
                4'd2: p_clk <= 1'b1;
                4'd3: p_rst <= 1'b1;
                4'd4: p_off <= 1'b1;
                4'd5: p_fast <= 1'b1;
                4'd6: p_slow <= 1'b1;
                4'd7: p_pad  <= 1'b1;
                4'd8: p_cst  <= 1'b1;
                default: ;
            endcase
            if (banner_done) begin
                case (want_banner)
                    4'd1: p_vcc <= 1'b0;
                    4'd2: p_clk <= 1'b0;
                    4'd3: p_rst <= 1'b0;
                    4'd5: p_fast <= 1'b0;
                    4'd6: p_slow <= 1'b0;
                    4'd7: p_pad  <= 1'b0;
                    4'd8: p_cst  <= 1'b0;
                    4'd9: p_inj  <= 1'b0;
                    4'd10: p_usb_in  <= 1'b0;
                    4'd11: p_usb_out <= 1'b0;
                    default: p_off <= 1'b0;
                endcase
            end
        end
    end

    // A queue of 1023 card bytes in one block RAM, and a count of what it
    // could not hold.
    //
    // **Two queues were tried here before and both were reverted.** A
    // 256-entry `reg [9:0] fifo [0:255]` read at `fifo[fifo_r]` is
    // distributed RAM, and it silenced this half on the part the moment the
    // card was activated. A 32-entry packed vector shifted on push and read
    // through a multiplexer -- flops and logic, nothing inferred -- flooded
    // the console with the banner table's `default` character instead. Both
    // passed `iso7816_terminal_tb.v`, and neither failure reproduced off the
    // board. Neither is evidence against a queue now: on 10 October 2026 the
    // router was found counting capacity per graph node rather than per piece
    // of metal, so two signals could share one long line (`docs/fpga-xray.md`,
    // "Two signals on one long line"), which is exactly the class of fault
    // that passes simulation and fails on the part.
    //
    // What one byte deep cost, measured: the card's reply to its
    // initialisation frame came back as `4E 00 00 4E 00 00 31 00 01 60 00 00`
    // with `lost_q` at 7 at F/D = 4, and at F/D = 8, where the port keeps up
    // with a steady reply, a minute of `W` on 11 October 2026 still lost 19
    // bytes, every one of them a reply that arrived while the display half
    // held the shared port for a frame dump -- about 10 ms, which at a byte
    // every 12 us is far more than one byte. `iso7816_terminal_tb.v` now holds
    // the port across a 40-byte reply, and fails against the one-byte buffer.
    // `lost_q` is still in `s`, so an overflow is never silent.
    reg  [15:0] lost_q = 16'd0;

    wire        take_card = card_rx_valid && !mute;
    // While `W`'s engine runs, the monitor's copies of the engine's own bytes
    // are not printed. The engine composed them, so they say nothing a host
    // does not know, and printing them as well as the SE's answers -- three
    // characters for each of ours, two for each of the SE's -- fell behind
    // the contact: 48 bytes counted lost in `s` in the first session, every
    // one of them a printing loss and none a protocol one.
    wire        take_mon  = !card_rx_valid && mon_rx_valid && !mute && !seph_active;
    wire        take      = take_card || take_mon;
    wire [7:0]  take_byte = take_card ? card_rx          : mon_rx;
    wire        take_bad  = take_card ? card_parity_err  : mon_parity_err;

    (* ram_style = "block" *)
    reg  [9:0]  fq [0:1023];
    reg  [9:0]  fq_w = 10'd0, fq_r = 10'd0;
    reg  [9:0]  fq_q = 10'd0;
    reg         fq_fetch = 1'b0;
    wire        fq_empty = fq_w == fq_r;
    wire        fq_full  = (fq_w + 10'd1) == fq_r;

    // The memory alone, with no reset, so it maps to a block RAM.
    always @(posedge sys) begin
        if (take && !fq_full) fq[fq_w] <= {take_bad, take_mon, take_byte};
        fq_q <= fq[fq_r];
    end

    always @(posedge sys) begin
        if (!rst_n) begin
            have_byte <= 1'b0;
            fq_w      <= 10'd0;
            fq_r      <= 10'd0;
            fq_fetch  <= 1'b0;
            lost_q    <= 16'd0;
            gap       <= 32'd0;
            line_open <= 1'b0;
        end else begin
            if (take) begin
                if (fq_full) begin
                    if (lost_q != 16'hFFFF) lost_q <= lost_q + 16'd1;
                end else begin
                    fq_w <= fq_w + 1'b1;
                end
                gap       <= gap_load;
                line_open <= 1'b1;
            end else if (gap != 0) begin
                gap <= gap - 1'b1;
            end
            // One read in flight at a time: a block RAM answers a cycle
            // after it is asked, so `fq_fetch` marks the cycle its output
            // is the entry at the old `fq_r`.
            if (fq_fetch) begin
                {pend_bad, pend_mon, pend_byte} <= fq_q;
                have_byte <= 1'b1;
            end else if (byte_done) begin
                have_byte <= 1'b0;
            end
            fq_fetch <= 1'b0;
            if (!fq_fetch && (!have_byte || byte_done) && !fq_empty) begin
                fq_r     <= fq_r + 1'b1;
                fq_fetch <= 1'b1;
            end
            if (eol_done)  line_open <= 1'b0;
        end
    end

    // `m` mutes and `u` unmutes, and activating clears it.
    //
    // **Not a toggle.** It was one, and because muting leaves the counters
    // moving while nothing prints, a session that ended muted looked exactly
    // like a receiver that had stopped working -- which cost three separate
    // measurements before the cause was noticed each time. State a person
    // cannot see must not be reachable by accident, so the commands set and
    // clear it outright, and `A` clears it because starting a card session is
    // when somebody wants to watch.
    always @(posedge sys) begin
        if (!rst_n) mute <= 1'b0;
        else if (cmd_now && cmd_data == 8'h6D) mute <= 1'b1;   // 'm'
        else if (cmd_now && cmd_data == 8'h75) mute <= 1'b0;   // 'u'
        else if (cmd_now && cmd_data == 8'h41) mute <= 1'b0;   // 'A'
    end

    // **`s` and `k` queue.** Each is pushed when it arrives and popped when
    // its own line *starts*, so one that arrives while an earlier answer is
    // still on the wire waits its turn. Two flags lost them: both were
    // cleared together at the end of either one's line, so on a Basys 3 at
    // 2 Mbaud an `s` sent straight after a `k` got no answer, and neither
    // did a second `k`. `iso7816_terminal_tb.v` sends `k`, `s`, `k` back to
    // back and wants all three, in order.
    //
    // One bit per request, oldest in bit 0: 0 the status word, 1 the known
    // constant. Sixteen deep; a seventeenth outstanding request is dropped,
    // since the port has no flow control. A host gets 34 characters back for
    // every one it sends, so only one that sends more than sixteen without
    // reading can fill it.
    localparam integer QUEUE = 16;
    reg [QUEUE-1:0] queue = {QUEUE{1'b0}};
    // Which entries are in use, as a thermometer: ones from bit 0 up. That
    // needs no adder, and the slot a new request lands in is the one bit a
    // push changes. A five-bit count was the first version, and its carry
    // chain was what the placer could not fit in `iso_display.v`.
    reg [QUEUE-1:0] q_held = {QUEUE{1'b0}};

    wire             q_push  = cmd_now && (cmd_data == 8'h73 || cmd_data == 8'h6B);
    wire             q_known = cmd_data == 8'h6B;
    // Exactly the emitter's own condition for starting a status line below.
    wire             q_pop   = !host_valid && phase == P_IDLE
                             && want_banner == 4'd0 && q_held[0];
    wire [QUEUE-1:0] q_kept  = q_pop ? {1'b0, q_held[QUEUE-1:1]}  : q_held;
    wire [QUEUE-1:0] q_shift = q_pop ? {1'b0, queue[QUEUE-1:1]} : queue;
    wire             q_fits  = !q_kept[QUEUE-1];
    wire [QUEUE-1:0] q_grown = {q_kept[QUEUE-2:0], 1'b1};
    wire [QUEUE-1:0] q_slot  = q_grown & ~q_kept;

    always @(posedge sys) begin
        if (!rst_n) begin
            queue <= {QUEUE{1'b0}};
            q_held  <= {QUEUE{1'b0}};
        end else begin
            queue <= (q_push && q_fits && q_known) ? (q_shift | q_slot) : q_shift;
            q_held  <= (q_push && q_fits) ? q_grown : q_kept;
        end
    end

    function [7:0] hex;
        input [3:0] n;
        hex = (n < 4'd10) ? (8'd48 + {4'd0, n}) : (8'd55 + {4'd0, n});
    endfunction

    function [7:0] banner_ch;
        input [3:0] which;
        input [1:0] at;
        case ({which, at})
            {4'd1, 2'd0}: banner_ch = "+";
            {4'd1, 2'd1}: banner_ch = "V";
            {4'd1, 2'd2}: banner_ch = "C";
            {4'd1, 2'd3}: banner_ch = "C";
            {4'd2, 2'd0}: banner_ch = "+";
            {4'd2, 2'd1}: banner_ch = "C";
            {4'd2, 2'd2}: banner_ch = "L";
            {4'd2, 2'd3}: banner_ch = "K";
            {4'd3, 2'd0}: banner_ch = "+";
            {4'd3, 2'd1}: banner_ch = "R";
            {4'd3, 2'd2}: banner_ch = "S";
            {4'd3, 2'd3}: banner_ch = "T";
            {4'd4, 2'd0}: banner_ch = "-";
            {4'd4, 2'd1}: banner_ch = "O";
            {4'd4, 2'd2}: banner_ch = "F";
            {4'd4, 2'd3}: banner_ch = "F";
            {4'd5, 2'd0}: banner_ch = "+";
            {4'd5, 2'd1}: banner_ch = "F";
            {4'd5, 2'd2}: banner_ch = "S";
            {4'd5, 2'd3}: banner_ch = "T";
            {4'd8, 2'd0}: banner_ch = cst_good ? "+" : "-";
            {4'd8, 2'd1}: banner_ch = "C";
            {4'd8, 2'd2}: banner_ch = "S";
            {4'd8, 2'd3}: banner_ch = "T";
            {4'd7, 2'd0}: banner_ch = pad_good ? "+" : "-";
            {4'd7, 2'd1}: banner_ch = "P";
            {4'd7, 2'd2}: banner_ch = "A";
            {4'd7, 2'd3}: banner_ch = "D";
            {4'd6, 2'd0}: banner_ch = "+";
            {4'd6, 2'd1}: banner_ch = "S";
            {4'd6, 2'd2}: banner_ch = "L";
            {4'd6, 2'd3}: banner_ch = "W";
            {4'd9, 2'd0}: banner_ch = "-";
            {4'd9, 2'd1}: banner_ch = "I";
            {4'd9, 2'd2}: banner_ch = "N";
            {4'd9, 2'd3}: banner_ch = "J";
            {4'd10, 2'd0}: banner_ch = "+";
            {4'd10, 2'd1}: banner_ch = "U";
            {4'd10, 2'd2}: banner_ch = "S";
            {4'd10, 2'd3}: banner_ch = "B";
            {4'd11, 2'd0}: banner_ch = "-";
            {4'd11, 2'd1}: banner_ch = "U";
            {4'd11, 2'd2}: banner_ch = "S";
            {4'd11, 2'd3}: banner_ch = "B";
            default:      banner_ch = "T";
        endcase
    endfunction

    wire [127:0] status = {
        bytes_q, perr_q, sent_q, tmo_q,
        etu_div, CARD_DIV16,
        // Bit 0 the fast rate; bits 7 and 6 the right and left button
        // lines as read, which is how a host learns their idle levels;
        // bit 5 the USB cable in (`U`), bit 4 the engine's bus reset sent,
        // bit 3 a `!` event still waiting to go.
        {buttons_in, usb_plugged, seph_usb_on, seph_inj_busy, 2'd0,
         etu_div == FAST_DIV16}, {5'd0, state},
        8'hA5,
        // The two samples `L` took, because `-PAD` alone does not say which
        // half failed and they mean different things: a line that would not
        // go low is an output that never reached the pin, while one that
        // stayed low is an enable that never released.
        // What the receive queue had to drop, saturating. Zero is the
        // only acceptable value once a card is talking at the fast rate.
        lost_q[7:0]
    };

    always @(posedge sys) begin
        if (!rst_n) begin
            phase      <= P_IDLE;
            pos        <= 4'd0;
            host_valid <= 1'b0;
        end else if (!host_valid) begin
            case (phase)
                P_IDLE:
                    if (want_banner != 4'd0)      begin phase <= P_BANNER; pos <= 4'd0; end
                    else if (q_held[0]) begin
                        // `k` is a constant with every nibble distinct. If
                        // it comes back altered, the fault is the path or
                        // the way this flow builds a constant, not the
                        // counters.
                        status_sh   <= queue[0]
                                     ? 128'h0123456789ABCDEFFEDCBA9876543210
                                     : status;
                        status_left <= 6'd32;
                        phase       <= P_STATUS;
                        pos         <= 4'd0;
                    end else if (have_byte) begin
                        phase <= P_BYTE;
                        pos   <= (pend_bad || pend_mon) ? 4'd0 : 4'd1;
                    end else if (line_open && gap == 0 && fq_empty && !fq_fetch) begin
                        // Not while the queue still holds bytes: between two
                        // reads `have_byte` is low for a cycle, and a gap that
                        // ran out while the port was held would otherwise
                        // break a reply in the middle.
                        phase <= P_EOL; pos <= 4'd0;
                    end
                P_BANNER: begin
                    host_data  <= (pos < 4'd4) ? banner_ch(want_banner, pos[1:0])
                                : (pos == 4'd4) ? 8'd13 : 8'd10;
                    host_valid <= 1'b1;
                end
                P_BYTE: begin
                    host_data  <= (pos == 4'd0) ? (pend_mon ? 8'h3E : 8'h21)
                                : (pos == 4'd1) ? hex(pend_byte[7:4])
                                :                 hex(pend_byte[3:0]);
                    host_valid <= 1'b1;
                end
                P_EOL: begin
                    host_data  <= (pos == 4'd0) ? 8'd13 : 8'd10;
                    host_valid <= 1'b1;
                end
                default: begin
                    host_data  <= (status_left != 0) ? hex(status_sh[127:124])
                                : (pos == 4'd0) ? 8'd13 : 8'd10;
                    host_valid <= 1'b1;
                end
            endcase
        end else if (host_ready) begin
            host_valid <= 1'b0;
            case (phase)
                P_BANNER: if (pos == 4'd5) begin phase <= P_IDLE; pos <= 4'd0; end
                          else pos <= pos + 4'd1;
                P_BYTE:   if (pos == 4'd2) begin phase <= P_IDLE; pos <= 4'd0; end
                          else pos <= pos + 4'd1;
                P_EOL:    if (pos == 4'd1) begin phase <= P_IDLE; pos <= 4'd0; end
                          else pos <= pos + 4'd1;
                default:
                    if (status_left != 0) begin
                        status_sh   <= {status_sh[123:0], 4'd0};
                        status_left <= status_left - 6'd1;
                    end else if (pos == 4'd1) begin
                        phase <= P_IDLE; pos <= 4'd0;
                    end else begin
                        pos <= pos + 4'd1;
                    end
            endcase
        end
    end

    // =================================================================
    // The board's own display
    // =================================================================

    reg [25:0] tick = 26'd0;
    always @(posedge sys) tick <= tick + 26'd1;

    assign led = {tick[25], perr_q != 0, etu_div == FAST_DIV16, locked,
                  rst_q, clk_on, vcc_q, bytes_q[7:0]};

    reg [17:0] mux = 18'd0;
    always @(posedge sys) mux <= mux + 18'd1;
    wire [1:0] digit = mux[17:16];

    wire [3:0] nibble =
        (digit == 2'd0) ? bytes_q[3:0]   :
        (digit == 2'd1) ? bytes_q[7:4]   :
        (digit == 2'd2) ? bytes_q[11:8]  :
                          bytes_q[15:12];

    function [6:0] segments;
        input [3:0] value;
        case (value)
            4'h0: segments = 7'b1000000;
            4'h1: segments = 7'b1111001;
            4'h2: segments = 7'b0100100;
            4'h3: segments = 7'b0110000;
            4'h4: segments = 7'b0011001;
            4'h5: segments = 7'b0010010;
            4'h6: segments = 7'b0000010;
            4'h7: segments = 7'b1111000;
            4'h8: segments = 7'b0000000;
            4'h9: segments = 7'b0010000;
            4'hA: segments = 7'b0001000;
            4'hB: segments = 7'b0000011;
            4'hC: segments = 7'b1000110;
            4'hD: segments = 7'b0100001;
            4'hE: segments = 7'b0000110;
            default: segments = 7'b0001110;
        endcase
    endfunction

    assign seg = segments(nibble);
    assign dp  = ~tick[25];
    assign an  = ~(4'd1 << digit);
endmodule
