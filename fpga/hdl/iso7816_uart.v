// iso7816_uart — the character layer of an ISO/IEC 7816-3 smart card
// interface, from the **terminal's** side.
//
// Everything this block believes about the protocol is **QUOTED** from
// ISO/IEC 7816-3 as it was described to this project, and **nothing here
// has been near a card or a part**. README.md says so where a reader
// cannot miss it, and its §2 is the list of the standard's own questions
// that this block had to answer by decision rather than by reading.
//
// What it does
//   One character at a time, on one wire, half duplex.
//
//   A character is QUOTED as: one start bit (low, 1 etu), eight data
//   bits, a parity bit **making the count of ones in those nine bits
//   even**, then a guard time of at least 2 etu during which the line is
//   high. That is the "8E2" a user of this block will call it. The etu —
//   the elementary time unit — is the bit period; on a card it is F/D
//   cycles of the card clock, which at the default F=372, D=1 and an
//   8 MHz clock is 21505 baud, and after a successful PPS exchange F/D
//   shrinks and the rate rises.
//
//   **The rate is a run-time input in system clock cycles**, not in card
//   clocks: `etu_div` is clocks per etu, which is `ip/bus/uart`'s `div`
//   convention and keeps every divisor a whole number.
//
//   On the intended board the system clock is **112 MHz**, chosen because
//   it divides by an even number to every card clock the owner wants — 8,
//   7, 5.6, 4, 3.5, 2, 1.75 and 1 MHz, at divisors 14, 16, 20, 28, 32,
//   56, 64 and 112 — so the card clock's duty cycle is exactly half at
//   every one. `etu_div` is then `(F/D) * divisor` and is an exact whole
//   number at every rate: 372 * 14 = **5208** for the default F/D at
//   8 MHz, which is 21505 baud, and 4 * 14 = **56** for F/D = 4 at the
//   same card clock, which is 2 Mbaud. Moving between them needs no
//   rebuild — a PPS exchange is exactly the design that cannot have its
//   rate compiled in. `etu_div` is read **once per character**, when a
//   byte is accepted or when a start edge is found, so a character in
//   flight keeps the rate it started at and a change takes effect from
//   the next character. That is the only definition under which a rate
//   change never corrupts a byte, and it is the rule `uart_frame_tx`
//   measured and this block copies.
//
//   **The card clock and the activation and reset sequence are not
//   here.** This is the character layer: a separate block drives CLK,
//   RST and VCC, and the division of labour is deliberate.
//
// The wire, and why `io_o` is a constant
//   QUOTED: the contact is one wire, half duplex, **open drain**. It
//   idles high through a pull-up; a transmitter pulls it low for a zero
//   and *releases* it for a one.
//
//   So the pad interface is `io_oe` and a **constant** `io_o`. The block
//   either pulls the contact to one level or lets go of it; it never
//   drives both, which is what lets the terminal and the card share one
//   wire without contention, and it is a structural property rather than
//   a promise: `io_o` is an `assign` of a parameter and no path reaches
//   it. `io_i` is the line as the pad reads it, synchronised here through
//   two flops before anything looks at it, because it is asynchronous to
//   `clk` by definition.
//
//   Three parameters put the **polarity of each of those three wires**
//   where a board needs it, because the pad is not always what drives the
//   contact: a discrete transistor pulling it down inverts, and so does a
//   buffer with an active-low enable. `OE_INVERT`, `OUT_INVERT` and
//   `IN_INVERT` are one exclusive or each and they are independent,
//   because the three paths are independent — an external pull-down FET
//   changes the output's sense while the input still taps the contact
//   directly. One parameter beats a second block.
//
// `active`, and why releasing the line is the safe state
//   The terminal switches the card's **VCC** through an external
//   high-side switch, so that it can do a proper cold reset, and the
//   ordering of that is mandated: VCC up, then CLK, then RST released,
//   and in reverse on the way down — RST low, CLK stopped low, **IO
//   released**, then VCC off. Driving a pin into an unpowered device
//   forward-biases its ESD diodes and can partially power it or damage
//   it, so "IO released before VCC drops" is a **hazard requirement** and
//   not a nicety.
//
//   The sequencer that owns that ordering is a separate block. What this
//   one provides is a way to be held **inert** so that sequencer can own
//   the line: `active` low releases the wire, idles the transmitter,
//   refuses a byte (`tx_ready` is low), abandons anything in flight with
//   `tx_abort`, and holds the receiver from starting a character. It
//   takes effect on the clock after `active` falls, because `active` is
//   registered — one system clock, against a VCC switch whose slew is
//   microseconds.
//
//   **Releasing the line is the safe state**, and that is the whole
//   reason this works: an open-drain bus idles high through its pull-up,
//   so a released line is indistinguishable from an idle one to the far
//   end. There is no bus turnaround to get wrong and no level being
//   driven at a device that is about to lose its supply.
//
//   **Coming out of it must not invent a character**, which is the trap
//   `ip/bus/spi_display_rx/README.md` §3a is the account of: an edge
//   compared against a register that was never loaded from a pin. Two
//   things make it safe here. The input synchroniser **keeps running**
//   while the block is inert, so it is never showing a reset value when
//   `active` rises; and `settle_sr` is held cleared while inert, so for
//   three clocks after `active` rises no edge is taken at all — by which
//   time both halves of the comparison were sampled while active, and an
//   edge that happened on an unpowered contact has passed out of the
//   window.
//
// The two conventions, and why it is a port
//   QUOTED. The terminal learns the convention by decoding the first ATR
//   byte TS at run time, so it cannot be a parameter:
//
//     direct  (TS = 0x3B)  a logic one is **high**, and the data bits
//                          arrive **least significant first**.
//     inverse (TS = 0x3F)  a logic one is **low**, and the data bits
//                          arrive **most significant first**. Both the
//                          polarity and the bit order invert.
//
//   The start bit is a *level* and not a logic value: the line is pulled
//   low for 1 etu in both conventions. So the two differ only over the
//   nine bits after it, and the arithmetic that falls out of that is the
//   whole of this block's convention handling:
//
//     * electrically, the inverse frame is the direct frame's data bits
//       **reversed and then complemented**, parity bit included;
//     * even parity over nine *logic* ones, complemented an odd number of
//       times, becomes **odd** parity over the nine bits on the wire. So
//       the check is `(^nine_bits) == convention` and not two checks —
//       one exclusive or and one comparison.
//
//   A block that inverted the polarity and forgot the bit order, or the
//   other way round, would still pass a test whose bytes happen to be
//   bit-symmetric or self-complementary; `iso7816_uart_tb.v`'s bytes are
//   neither, and it decodes one captured frame **all four ways** and
//   asserts that only the full inverse reading matches.
//
// T=0 parity error signalling
//   QUOTED: a receiver that sees bad parity pulls the line low for 1 to
//   2 etu starting 10.5 etu after the start bit's falling edge, and the
//   transmitter samples at about 11.5 etu and repeats the character if it
//   finds the line low.
//
//   Both halves are implemented, behind `PARITY_RETRY` so a link that
//   wants neither can have neither. This block drives a **2 etu** pulse,
//   the long end of the permitted range, because the two ends measure
//   10.5 and 11.5 etu from their own detection of the same edge and each
//   is late by its own synchroniser: a 2 etu pulse then covers the far
//   end's sample point for any `etu_div` at or above five, and `DIV_MIN`
//   is eight. README.md §4 has that arithmetic.
//
//   A character that failed its parity check is still **delivered**, with
//   `rx_parity_error` beside it, which is what `uart_frame_rx` does and
//   for the same reason: a wrong byte at a consumer is more informative
//   than silence, and the repeat arrives afterwards as a second
//   character. `rx_char_count` therefore counts characters and
//   `parity_error_count` the bad ones, so both stay comparable against a
//   driven total.
//
//   A transmitted character is repeated at most `MAX_REPEAT` times. The
//   standard as this project was given it fixes no limit, and a block
//   that repeated for ever would be the hang this interface exists not to
//   have, so the limit is a parameter and giving up raises `tx_abort`.
//
// Guard time and waiting time
//   QUOTED: the guard time beyond the mandatory 2 etu is TC1 from the
//   ATR, 0 to 254 extra etu, so `guard_etu` is a run-time input. The
//   obligation QUOTED is on the *interval between the leading edges of
//   two consecutive characters*, whichever direction each went in, so
//   this block holds `tx_ready` low for `2 + guard_etu` etu after **any**
//   character and not only after one of its own.
//
//   QUOTED: a card that never answers must become a reported timeout.
//   `wt_etu` is the waiting time in etu; the timer is armed when a
//   character ends in either direction, disarmed by the next start edge
//   or by a transmission, and `rx_timeout` strobes for one cycle when it
//   expires. `wt_etu` of zero disables it. One strobe per arming, so a
//   link that has simply finished talking produces one timeout and not a
//   stream of them — README.md §2 is honest that "when does the terminal
//   stop expecting a character" is a question about the *transport* layer
//   that this block cannot answer.
//
// What it does not do
//   No card clock, no activation, no reset, no VCC or VPP control, no ATR
//   decoding (so nothing here chooses the convention, the guard time or
//   the waiting time — it is told them), no PPS exchange, no T=0 or T=1
//   transport layer, no waiting time extension, no FIFO, and no
//   clock-stop handling. It is the character layer, and the bytes it
//   hands up and takes down are somebody else's protocol.
module iso7816_uart #(
    // System clock cycles per etu when `etu_div` does not give a usable
    // one. 5208 is 21505 baud on the intended board: a 112 MHz system
    // clock, an 8 MHz card clock (divisor 14) and the default F = 372,
    // D = 1. That is the rate a session opens at.
    parameter ETU_DIV = 5208,
    // 1 implements the T=0 parity error exchange in both directions: the
    // error pulse when this block receives a bad character, and the
    // sample and repeat when it sends one. 0 removes both, and then
    // `repeat_count` and `tx_abort` can never move.
    parameter PARITY_RETRY = 1,
    // How many times one character may be repeated before it is
    // abandoned with `tx_abort`. 0 never repeats.
    parameter MAX_REPEAT = 3,
    // Width of every counter. They saturate rather than wrap.
    parameter COUNT_WIDTH = 16,
    // 1 drives `io_oe` **low** to enable the driver, for a pad or a
    // buffer whose enable is active low.
    parameter OE_INVERT = 0,
    // 1 makes `io_o` a constant **one**, for an external transistor or an
    // inverting buffer that pulls the contact down when driven high.
    parameter OUT_INVERT = 0,
    // 1 takes `io_i` as inverted, for a receive path through an inverting
    // buffer. Independent of the two above: an external pull-down changes
    // the output's sense while the input still taps the contact directly.
    parameter IN_INVERT = 0,
    // A level must hold for `etu_div >> GLITCH_SHIFT` clocks before this
    // block believes it. 0 disables the filter.
    //
    // **Because a contact is a wire in the world and a glitch is a start
    // bit.** A capture of a real card's ATU line, 200 MHz, 20 ms, found 241
    // transitions shorter than a microsecond, the narrowest 5 ns. Two
    // synchronising flops reject only what is shorter than a clock period —
    // 18 ns at 112 MHz — so a 1 us spike is as good an edge as data. One
    // such spike makes this block flag bad parity, and with `PARITY_RETRY`
    // on it then pulls the contact low **while the card is still
    // transmitting**, which corrupts the next character, which fails too:
    // a captured ATR read `3B 1B FF 1D EE 2B FE ...` where the card sent
    // `3B 1B 87 05 32 2E ...`, with every `FF` an error pulse of this
    // block's own making, 11.5 etu after the character before it.
    //
    // 4 is a sixteenth of an etu: 2.9 us at 21505 baud, 31 ns at 2 Mbaud,
    // and nothing at all below `etu_div` 16, which degrades to no filter
    // rather than to a broken one. The cost is that much latency on every
    // edge, against a bit sampled at half an etu.
    //
    // **Default 0, because it fixes nothing that has been measured.** It was
    // written for the capture above and then shown not to be the cause:
    // `glitch_tb` drives a spike into a data bit and **both** a filtered and
    // an unfiltered instance read the character correctly, because a bit is
    // sampled at its centre and a spike at the quarter point is gone by
    // then. By the same arithmetic the microsecond spikes in that capture
    // are a fiftieth of an etu at 21505 baud and cannot reach a sample
    // point. The knob is kept because a line with a slow edge or a long stub
    // is a real thing a board may have, and because the arithmetic for
    // choosing it is written down here; it is off until something measured
    // asks for it.
    //
    // **It costs one flip-flop even at 0**, because the filtered line is a
    // third stage after the two synchronising flops: `dff` 38 to 39 at both
    // lookup-table widths, `SB_DFFS` 3 to 4 on an iCE40 HX1K, `TRELLIS_FF`
    // 271 to 272 on an ECP5 45F. `footprints_match_the_documentation` is
    // what caught that, and it is the point of its existing: a shared
    // block's cost should not change without somebody noticing.
    parameter GLITCH_SHIFT = 0
) (
    input  wire                   clk,
    input  wire                   rst_n,

    // Low holds the block **inert**: the wire released, the transmitter
    // idle and refusing bytes, anything in flight abandoned with
    // `tx_abort`, and the receiver held off. Synchronous to `clk` and
    // registered, so it takes effect on the next clock. The activation
    // and deactivation sequencer drives it; the header says why releasing
    // the line is the safe state and why coming back out of it cannot
    // invent a character.
    input  wire                   active,

    // System clock cycles per etu, read once per character. Anything
    // below DIV_MIN means "use ETU_DIV", which is how `uart_rx` treats a
    // `div` it cannot use: a rate that cannot be expressed has to leave
    // the port working rather than stop the block.
    input  wire [15:0]            etu_div,
    // Extra guard etu beyond the mandatory two: TC1 from the ATR.
    input  wire [7:0]             guard_etu,
    // 0 direct (TS = 0x3B), 1 inverse (TS = 0x3F).
    input  wire                   convention,
    // Waiting time in etu. Zero disables the timer.
    input  wire [23:0]            wt_etu,

    input  wire [7:0]             tx_data,
    input  wire                   tx_valid,
    output wire                   tx_ready,
    // One cycle when a character was repeated MAX_REPEAT times and the
    // far end was still signalling a parity error. The handshake has
    // completed and the character is gone.
    output reg                    tx_abort,

    output reg  [7:0]             rx_data,
    // A handshake and not a strobe: `rx_valid` stays high until
    // `rx_ready` is high in the same cycle. That is what makes
    // `rx_overrun` observable at all — a receiver with no holding
    // register cannot tell a character nobody wanted from one nobody
    // could take. Tie `rx_ready` to one for strobe behaviour, and then
    // `rx_overrun` can never fire.
    input  wire                   rx_ready,
    output reg                    rx_valid,
    // The delivered character's parity was wrong. Held with `rx_valid`
    // and cleared when the character is taken.
    output reg                    rx_parity_error,
    // One cycle when a character was assembled over one nobody had
    // taken. The new character is kept and the old one is lost.
    output reg                    rx_overrun,
    // One cycle when the waiting time expired with no character.
    output reg                    rx_timeout,

    input  wire                   io_i,
    output wire                   io_oe,
    output wire                   io_o,

    // Characters whose transmit handshake completed, repeats and aborts
    // included, so this is comparable against the bytes a producer drove.
    output wire [COUNT_WIDTH-1:0] tx_char_count,
    // Characters delivered, good parity or bad.
    output wire [COUNT_WIDTH-1:0] rx_char_count,
    // Of those, the ones whose parity was wrong.
    output wire [COUNT_WIDTH-1:0] parity_error_count,
    // Retransmissions, so a character sent twice is one `tx_char_count`
    // and one of these.
    output wire [COUNT_WIDTH-1:0] repeat_count,
    output wire [COUNT_WIDTH-1:0] timeout_count
);
    // -------------------------------------------------------------------
    // Sizes and constants, every one a sized localparam
    //
    // `reticle sim` rejects the SystemVerilog width cast that
    // `reticle check` accepts, so a constant that has to be a particular
    // width is declared at that width here rather than cast where it is
    // used.
    // -------------------------------------------------------------------

    // The least usable `etu_div`. Five is the arithmetic floor — the
    // receiver's error pulse must cover the transmitter's sample point
    // when each end is up to three clocks late off its own synchroniser,
    // which needs `11.5*d - 2 >= 10.5*d + 3` — and eight is that with
    // half an etu of margin at each end and a round number to state.
    // README.md §4 is the derivation.
    localparam [15:0] DIV_MIN = 16'd8;

    localparam [2:0] S_IDLE     = 3'd0;  // released, watching for a start edge
    localparam [2:0] S_TX_BIT   = 3'd1;  // driving start + 8 data + parity
    localparam [2:0] S_TX_GUARD = 3'd2;  // released; samples the error pulse
    localparam [2:0] S_RX_START = 3'd3;  // half an etu into the start bit
    localparam [2:0] S_RX_BIT   = 3'd4;  // sampling 8 data + parity
    localparam [2:0] S_RX_ERRW  = 3'd5;  // waiting out the etu to 10.5
    localparam [2:0] S_RX_PULSE = 3'd6;  // driving the 2 etu error pulse
    localparam [2:0] S_RX_END   = 3'd7;  // half an etu to the frame's end

    localparam [15:0] DIV_ZERO  = 16'd0;
    localparam [15:0] DIV_ONE   = 16'd1;
    localparam [15:0] ETU_DIV_W = ETU_DIV;

    localparam [3:0] BIT_ZERO = 4'd0;
    localparam [3:0] BIT_ONE  = 4'd1;
    // The last bit index of a transmitted frame: start plus eight data
    // plus parity is ten bits, 0 to 9.
    localparam [3:0] TX_LAST  = 4'd9;
    // The last index of a received frame's nine sampled bits.
    localparam [3:0] RX_LAST  = 4'd8;

    // The guard limit spans 1 to 256 etu, and two more once an error has
    // been seen, so nine bits.
    localparam [8:0] G_ZERO = 9'd0;
    localparam [8:0] G_ONE  = 9'd1;
    localparam [8:0] G_TWO  = 9'd2;

    localparam [23:0] WT_ZERO = 24'd0;
    localparam [23:0] WT_ONE  = 24'd1;

    localparam [3:0] REP_ZERO = 4'd0;
    localparam [3:0] REP_ONE  = 4'd1;
    localparam [3:0] REP_MAX  = MAX_REPEAT;

    localparam [COUNT_WIDTH-1:0] CNT_ZERO = {COUNT_WIDTH{1'b0}};
    localparam [COUNT_WIDTH-1:0] CNT_MAX  = {COUNT_WIDTH{1'b1}};
    localparam [COUNT_WIDTH-1:0] CNT_ONE  = {{(COUNT_WIDTH-1){1'b0}}, 1'b1};

    localparam RETRY = (PARITY_RETRY != 0);

    // -------------------------------------------------------------------
    // The line, synchronised, and its falling edge
    // -------------------------------------------------------------------

    // Two separately named registers, so the flip-flop inference produces
    // the two-flop chain a crossing checker recognises rather than one
    // two-bit register. They reset to one, the level a released wire has
    // — but **nothing below depends on that**, which is the point, and
    // the reason is in the next comment.
    reg io_meta_q;
    reg io_raw_q;          // the second synchronising flop
    reg io_sync_q;         // and the filtered line everything else reads
    reg io_prev_q;
    reg [15:0] glitch_q;

    // **A synchroniser's reset value is not an observation of the pin.**
    // `io_prev_q` only holds a value that came off the wire from the
    // third clock after reset release; before that, both it and
    // `io_sync_q` are showing what reset put there. So any edge taken in
    // that window is manufactured by reset values and says nothing about
    // the line — and the edge it manufactures here is the worst one
    // available: a contact held low by a dead card, or by the other end
    // of the wire, arrives as a start bit and decodes as a character of
    // nine zeros whose parity is legitimately even. A byte nobody sent,
    // on the one fault condition the waiting time exists to report.
    //
    // `ip/bus/spi_display_rx/README.md` §3a is this project's account of
    // that fault **found on a part**: a counter permanently one too high,
    // from a synchroniser's reset value compared against as if it were a
    // pin. This is the same shape and takes the same remedy — count the
    // blind window out and gate the one edge detector on it. Three bits,
    // because the chain is three registers deep.
    //
    // It is also held cleared while the block is **inert**, which is the
    // second half of the same argument: after `active` rises no edge is
    // taken for three clocks, by which time both halves of the comparison
    // were sampled while active. The chain itself keeps running
    // regardless, so it is never presenting a reset value when `active`
    // comes back — freezing or resetting it there would rebuild the fault
    // exactly.
    reg [2:0] settle_sr;
    reg       active_q;
    wire settled = settle_sr[2];

    // The pad's own sense, which a board may have inverted.
    wire io_in = IN_INVERT ? ~io_i : io_i;

    // How long a level must hold. Derived from the live `etu_div`, so it
    // follows a rate change without being told.
    wire [15:0] glitch_span = (GLITCH_SHIFT == 0) ? 16'd0
                                                  : (etu_div >> GLITCH_SHIFT);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            io_meta_q <= 1'b1;
            io_raw_q  <= 1'b1;
            io_sync_q <= 1'b1;
            io_prev_q <= 1'b1;
            glitch_q  <= 16'd0;
            settle_sr <= 3'b000;
            active_q  <= 1'b0;
        end else begin
            io_meta_q <= io_in;
            io_raw_q  <= io_meta_q;
            io_prev_q <= io_sync_q;
            // A change is believed once it has held `glitch_span` clocks.
            if (glitch_span == 16'd0) begin
                io_sync_q <= io_raw_q;
            end else if (io_raw_q == io_sync_q) begin
                glitch_q <= 16'd0;
            end else if (glitch_q >= glitch_span) begin
                io_sync_q <= io_raw_q;
                glitch_q  <= 16'd0;
            end else begin
                glitch_q <= glitch_q + 16'd1;
            end
            settle_sr <= active_q ? {settle_sr[1:0], 1'b1} : 3'b000;
            active_q  <= active;
        end
    end

    // A start bit is an **edge** and not a level, and an edge only once
    // both halves of the comparison have been loaded from the wire.
    // Watching the level instead would read the error pulse this block
    // drives, the tail of a character whose parity bit was electrically
    // low, and a line held low by a dead card, each as the beginning of a
    // character.
    wire fall = settled & io_prev_q & ~io_sync_q;

    // -------------------------------------------------------------------
    // The rate, the guard and the frame
    // -------------------------------------------------------------------

    wire [15:0] etu_used      = (etu_div < DIV_MIN) ? ETU_DIV_W : etu_div;
    wire [15:0] etu_last_next = etu_used - DIV_ONE;
    // Half an etu, as a shift rather than a divide: a divide by a
    // run-time value is what this arrangement exists not to need.
    wire [15:0] etu_half_next = {1'b0, etu_used[15:1]} - DIV_ONE;
    // The mandatory two etu plus TC1, one short.
    wire [8:0]  guard_last_next = {1'b0, guard_etu} + G_ONE;

    // The parity bit: even over the eight data bits and itself, which is
    // their exclusive or, in **both** conventions — the conventions
    // disagree about the level that carries a one, not about how many
    // ones there are.
    wire par_tx = ^tx_data;
    // The data bits most significant first, which is the order the
    // inverse convention puts on the wire.
    wire [7:0] tx_rev = {tx_data[0], tx_data[1], tx_data[2], tx_data[3],
                         tx_data[4], tx_data[5], tx_data[6], tx_data[7]};
    // Index 0 is the first bit after the start bit, so the shifter walks
    // it from the bottom. Direct: the data least significant first then
    // the parity bit, each at the level its logic value means. Inverse:
    // reversed and complemented, parity bit included.
    wire [8:0] tx_nine  = convention ? ~{par_tx, tx_rev} : {par_tx, tx_data};
    // The start bit is low in both conventions.
    wire [9:0] tx_frame = {tx_nine, 1'b0};

    reg [15:0] etu_last;
    reg [15:0] etu_half;
    reg [15:0] etu_cnt;
    reg [8:0]  guard_last;

    // The frame being sent, kept whole so a repeat needs no second read
    // of `tx_data` — the producer's handshake completed when the
    // character was accepted and it is entitled to have moved on.
    reg [9:0]  frame_q;
    reg [9:0]  shift_q;
    reg [3:0]  bit_idx;
    reg [8:0]  g_cnt;
    reg [3:0]  rep_left;
    reg        err_seen;
    // A character is in flight, in **either** direction. `tx_ready` is
    // built on it, so a byte is never accepted while the card is talking.
    reg        busy_q;
    reg [2:0]  state;

    // The eight bits of a received character sampled so far; the ninth is
    // the parity bit and is consumed in the cycle it arrives.
    reg [8:0]  rx_nine;

    // Etu left before this block may transmit: the interval QUOTED
    // between two consecutive characters' leading edges, counted down
    // while idle.
    reg [8:0]  hold_q;

    reg [23:0] wt_cnt;
    reg [23:0] wt_last;
    reg        wt_en;
    reg        wt_armed;

    // -------------------------------------------------------------------
    // Receive decode
    //
    // Read in the cycle the ninth bit is sampled, so it decodes the word
    // **as it will be** once that sample is in rather than the register,
    // which still holds eight bits.
    // -------------------------------------------------------------------

    wire [8:0] nine_next = {io_sync_q, rx_nine[8:1]};
    // Even parity over nine logic ones, complemented an odd number of
    // times, is odd parity over the nine bits on the wire — so one
    // exclusive or answers both conventions.
    wire rx_par_ok = (^nine_next) == convention;
    wire [7:0] rx_inverse = ~{nine_next[0], nine_next[1], nine_next[2],
                              nine_next[3], nine_next[4], nine_next[5],
                              nine_next[6], nine_next[7]};
    wire [7:0] rx_byte = convention ? rx_inverse : nine_next[7:0];

    // -------------------------------------------------------------------
    // The wire
    // -------------------------------------------------------------------

    // Driven for a zero of the frame being sent, and for the error pulse.
    // Released for everything else, including every one of a character:
    // that is what open drain means, and it is why two devices can share
    // this wire. Released unconditionally while inert, which is the
    // hazard requirement the header states.
    wire drive = active_q
               & (((state == S_TX_BIT) & ~shift_q[0]) | (state == S_RX_PULSE));
    assign io_oe = OE_INVERT ? ~drive : drive;
    // A constant, so the pad pulls the contact to one level or lets the
    // pull-up have it and can never drive both. Which level that is
    // belongs to the board: `OUT_INVERT` is for an external transistor or
    // an inverting buffer.
    assign io_o  = OUT_INVERT ? 1'b1 : 1'b0;

    assign tx_ready = active_q & ~busy_q & (hold_q == G_ZERO);

    // -------------------------------------------------------------------
    // The state machine
    // -------------------------------------------------------------------

    // The events the counters read, as one-cycle registered pulses, so
    // that each counter is one `if` over a named condition rather than a
    // second copy of the state machine's arithmetic.
    // Where the transmit guard ends. Two etu longer once the far end has
    // been seen signalling an error, because the far end's pulse runs to
    // 12.5 etu and a repeat that began at 12.0 would put its start bit
    // **inside** that pulse, where nothing could see it. Two etu is the
    // length of the longest permitted pulse, so the repeat's leading edge
    // lands at 14 etu at the earliest and the wire is high before it.
    wire [8:0] guard_limit = err_seen ? (guard_last + G_TWO) : guard_last;

    reg deliver;      // a character is being handed to the consumer
    reg deliver_bad;  // ...and its parity was wrong
    reg tx_done;      // a transmit handshake is completing
    reg repeated;     // a character is being sent again
    reg timed_out;    // the waiting time expired

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= S_IDLE;
            etu_last        <= ETU_DIV_W - DIV_ONE;
            etu_half        <= {1'b0, ETU_DIV_W[15:1]} - DIV_ONE;
            etu_cnt         <= DIV_ZERO;
            guard_last      <= G_ONE;
            frame_q         <= {10{1'b1}};
            shift_q         <= {10{1'b1}};
            bit_idx         <= BIT_ZERO;
            g_cnt           <= G_ZERO;
            rep_left        <= REP_ZERO;
            err_seen        <= 1'b0;
            busy_q          <= 1'b0;
            rx_nine         <= 9'd0;
            hold_q          <= G_ZERO;
            wt_cnt          <= WT_ZERO;
            wt_last         <= WT_ZERO;
            wt_en           <= 1'b0;
            wt_armed        <= 1'b0;
            rx_data         <= 8'd0;
            rx_valid        <= 1'b0;
            rx_parity_error <= 1'b0;
            rx_overrun      <= 1'b0;
            rx_timeout      <= 1'b0;
            tx_abort        <= 1'b0;
            deliver         <= 1'b0;
            deliver_bad     <= 1'b0;
            tx_done         <= 1'b0;
            repeated        <= 1'b0;
            timed_out       <= 1'b0;
        end else begin
            // The events that belong to no character.
            rx_overrun  <= 1'b0;
            rx_timeout  <= 1'b0;
            tx_abort    <= 1'b0;
            deliver     <= 1'b0;
            deliver_bad <= 1'b0;
            tx_done     <= 1'b0;
            repeated    <= 1'b0;
            timed_out   <= 1'b0;

            // The character is let go when it is taken, with the flag
            // that describes it. A character delivered in this same cycle
            // overrides this below, which is the right order: the
            // consumer took the old byte and the new one is now the one
            // being offered.
            if (rx_valid & rx_ready) begin
                rx_valid        <= 1'b0;
                rx_parity_error <= 1'b0;
            end

            if (!active_q) begin
                // Inert. Everything in flight is abandoned and the wire
                // is released; a character already delivered is the
                // consumer's and is left alone, as are the counters,
                // which only `rst_n` clears. A transmission that was
                // abandoned is reported, because `tx_abort` means "the
                // character is gone" whichever reason it went for — and
                // it is one cycle wide, since `state` is `S_IDLE` from
                // the next clock.
                tx_abort <= (state == S_TX_BIT) | (state == S_TX_GUARD);
                state    <= S_IDLE;
                busy_q   <= 1'b0;
                etu_cnt  <= DIV_ZERO;
                bit_idx  <= BIT_ZERO;
                g_cnt    <= G_ZERO;
                err_seen <= 1'b0;
                hold_q   <= G_ZERO;
                wt_armed <= 1'b0;
            end else case (state)
                S_IDLE: begin
                    if (fall) begin
                        // A character is arriving. The rate it will be
                        // timed at is taken here and nowhere else, so
                        // every sample of one frame is the same distance
                        // from the last.
                        state    <= S_RX_START;
                        busy_q   <= 1'b1;
                        etu_cnt  <= DIV_ZERO;
                        etu_last <= etu_last_next;
                        etu_half <= etu_half_next;
                        wt_armed <= 1'b0;
                    end else if (tx_valid & ~busy_q & (hold_q == G_ZERO)) begin
                        state      <= S_TX_BIT;
                        busy_q     <= 1'b1;
                        etu_cnt    <= DIV_ZERO;
                        etu_last   <= etu_last_next;
                        etu_half   <= etu_half_next;
                        guard_last <= guard_last_next;
                        frame_q    <= tx_frame;
                        shift_q    <= tx_frame;
                        bit_idx    <= BIT_ZERO;
                        err_seen   <= 1'b0;
                        rep_left   <= REP_MAX;
                        wt_armed   <= 1'b0;
                    end else if (etu_cnt == etu_last) begin
                        // An etu of idle, measured at the rate the **last
                        // character** ran at and not at whatever
                        // `etu_div` says now. That is deliberate twice
                        // over: the guard interval belongs to the
                        // character that ended, which is the etu the far
                        // end is measuring it in too; and refreshing the
                        // limit here as well would make the per-character
                        // latch above redundant, so that removing either
                        // one would leave the block working and no test
                        // could tell. README.md §5 records that this is
                        // the shape of a defect that hides, and
                        // `iso7816_uart_tb.v`'s §2 catches the loss of
                        // either half now that there is only one of them.
                        etu_cnt  <= DIV_ZERO;
                        if (hold_q != G_ZERO) hold_q <= hold_q - G_ONE;
                        if (wt_armed & wt_en) begin
                            if (wt_cnt == wt_last) begin
                                rx_timeout <= 1'b1;
                                timed_out  <= 1'b1;
                                wt_armed   <= 1'b0;
                            end else begin
                                wt_cnt <= wt_cnt + WT_ONE;
                            end
                        end
                    end else begin
                        etu_cnt <= etu_cnt + DIV_ONE;
                    end
                end

                S_TX_BIT: begin
                    if (etu_cnt == etu_last) begin
                        etu_cnt <= DIV_ZERO;
                        // A one is shifted in, so the line returns to
                        // idle by itself when the frame runs out.
                        shift_q <= {1'b1, shift_q[9:1]};
                        if (bit_idx == TX_LAST) begin
                            state <= S_TX_GUARD;
                            g_cnt <= G_ZERO;
                        end else begin
                            bit_idx <= bit_idx + BIT_ONE;
                        end
                    end else begin
                        etu_cnt <= etu_cnt + DIV_ONE;
                    end
                end

                S_TX_GUARD: begin
                    // The parity bit ended at 10.0 etu, so `g_cnt` 1 is
                    // the twelfth etu and its middle is 11.5 etu after
                    // the start bit's falling edge: the sample point
                    // QUOTED for the error pulse.
                    if (RETRY && (g_cnt == G_ONE) && (etu_cnt == etu_half)
                        && !io_sync_q) begin
                        err_seen <= 1'b1;
                    end
                    if (etu_cnt == etu_last) begin
                        etu_cnt <= DIV_ZERO;
                        if (g_cnt == guard_limit) begin
                            if (err_seen && (rep_left != REP_ZERO)) begin
                                // Send it again. The guard just spent is
                                // the interval the standard asks for, so
                                // the repeat starts here.
                                state    <= S_TX_BIT;
                                shift_q  <= frame_q;
                                bit_idx  <= BIT_ZERO;
                                err_seen <= 1'b0;
                                rep_left <= rep_left - REP_ONE;
                                repeated <= 1'b1;
                            end else begin
                                state    <= S_IDLE;
                                busy_q   <= 1'b0;
                                tx_done  <= 1'b1;
                                tx_abort <= err_seen;
                                // The guard is spent, so nothing holds
                                // the next transmission; what starts now
                                // is the window the card has to answer.
                                hold_q   <= G_ZERO;
                                wt_cnt   <= WT_ZERO;
                                wt_last  <= wt_etu - WT_ONE;
                                wt_en    <= (wt_etu != WT_ZERO);
                                wt_armed <= 1'b1;
                            end
                        end else begin
                            g_cnt <= g_cnt + G_ONE;
                        end
                    end else begin
                        etu_cnt <= etu_cnt + DIV_ONE;
                    end
                end

                S_RX_START: begin
                    if (etu_cnt == etu_half) begin
                        etu_cnt <= DIV_ZERO;
                        // Still low at the middle of the start bit: a
                        // character. Otherwise it was a glitch, and the
                        // block goes back to watching without having
                        // disturbed the wire.
                        if (io_sync_q) begin
                            state  <= S_IDLE;
                            busy_q <= 1'b0;
                        end else begin
                            state   <= S_RX_BIT;
                            bit_idx <= BIT_ZERO;
                        end
                    end else begin
                        etu_cnt <= etu_cnt + DIV_ONE;
                    end
                end

                S_RX_BIT: begin
                    if (etu_cnt == etu_last) begin
                        etu_cnt <= DIV_ZERO;
                        rx_nine <= nine_next;
                        if (bit_idx == RX_LAST) begin
                            // The ninth sample is the parity bit, so the
                            // character is complete and is delivered in
                            // this cycle, decoded from `nine_next`.
                            bit_idx         <= BIT_ZERO;
                            state           <= (RETRY && !rx_par_ok)
                                                 ? S_RX_ERRW : S_RX_END;
                            rx_data         <= rx_byte;
                            rx_valid        <= 1'b1;
                            rx_parity_error <= ~rx_par_ok;
                            // The one the consumer never took.
                            rx_overrun      <= rx_valid & ~rx_ready;
                            deliver         <= 1'b1;
                            deliver_bad     <= ~rx_par_ok;
                            // The interval before this block may
                            // transmit, and the window the card has to
                            // send another character.
                            hold_q          <= guard_last_next;
                            wt_cnt          <= WT_ZERO;
                            wt_last         <= wt_etu - WT_ONE;
                            wt_en           <= (wt_etu != WT_ZERO);
                            wt_armed        <= 1'b1;
                        end else begin
                            bit_idx <= bit_idx + BIT_ONE;
                        end
                    end else begin
                        etu_cnt <= etu_cnt + DIV_ONE;
                    end
                end

                S_RX_ERRW: begin
                    // One etu from the parity sample at 9.5 etu brings
                    // the pulse's leading edge to 10.5 etu.
                    if (etu_cnt == etu_last) begin
                        etu_cnt <= DIV_ZERO;
                        state   <= S_RX_PULSE;
                        g_cnt   <= G_ZERO;
                    end else begin
                        etu_cnt <= etu_cnt + DIV_ONE;
                    end
                end

                S_RX_PULSE: begin
                    if (etu_cnt == etu_last) begin
                        etu_cnt <= DIV_ZERO;
                        if (g_cnt == G_ONE) begin
                            // Two etu of pulse: 10.5 to 12.5 etu.
                            state  <= S_IDLE;
                            busy_q <= 1'b0;
                            // This block held the wire past the end of
                            // the character, so the interval starts
                            // again from here.
                            hold_q <= guard_last_next;
                        end else begin
                            g_cnt <= g_cnt + G_ONE;
                        end
                    end else begin
                        etu_cnt <= etu_cnt + DIV_ONE;
                    end
                end

                // S_RX_END, which is 3'd7 and the last of eight: half an
                // etu from the parity sample to the end of the character,
                // so a parity bit that was electrically low cannot be
                // read as the next start edge. Written as `default` so
                // that all eight encodings of `state` are covered and
                // none can wedge the block — an unrouted slice input on
                // an ECP5 reads as a one.
                default: begin
                    if (etu_cnt == etu_half) begin
                        etu_cnt <= DIV_ZERO;
                        state   <= S_IDLE;
                        busy_q  <= 1'b0;
                    end else begin
                        etu_cnt <= etu_cnt + DIV_ONE;
                    end
                end
            endcase
        end
    end

    // -------------------------------------------------------------------
    // The counters, each saturating
    //
    // An instrument that wraps reports a small number for a large fault.
    // -------------------------------------------------------------------
    reg [COUNT_WIDTH-1:0] tx_chars_q;
    reg [COUNT_WIDTH-1:0] rx_chars_q;
    reg [COUNT_WIDTH-1:0] par_errs_q;
    reg [COUNT_WIDTH-1:0] repeats_q;
    reg [COUNT_WIDTH-1:0] timeouts_q;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_chars_q <= CNT_ZERO;
            rx_chars_q <= CNT_ZERO;
            par_errs_q <= CNT_ZERO;
            repeats_q  <= CNT_ZERO;
            timeouts_q <= CNT_ZERO;
        end else begin
            if (tx_done && tx_chars_q != CNT_MAX) begin
                tx_chars_q <= tx_chars_q + CNT_ONE;
            end
            if (deliver && rx_chars_q != CNT_MAX) begin
                rx_chars_q <= rx_chars_q + CNT_ONE;
            end
            if (deliver_bad && par_errs_q != CNT_MAX) begin
                par_errs_q <= par_errs_q + CNT_ONE;
            end
            if (repeated && repeats_q != CNT_MAX) begin
                repeats_q <= repeats_q + CNT_ONE;
            end
            if (timed_out && timeouts_q != CNT_MAX) begin
                timeouts_q <= timeouts_q + CNT_ONE;
            end
        end
    end

    assign tx_char_count      = tx_chars_q;
    assign rx_char_count      = rx_chars_q;
    assign parity_error_count = par_errs_q;
    assign repeat_count       = repeats_q;
    assign timeout_count      = timeouts_q;
endmodule
