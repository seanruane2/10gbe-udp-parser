// udp_rx_parser.v
// =============================================================================
// Ethernet / IPv4 / UDP receive parser, all fused into one module, running on a
// 64-bit XGMII datapath at 156.25 MHz (which works out to 10 Gbit/s).
//
// This is my second go at the problem. The first attempt is the four-stage
// xgmii_rx -> eth_rx -> ipv4_rx -> udp_rx chain that's still sitting in rtl/.
// It works fine, but it takes 9 cycles to get from "first frame byte on the
// wire" to "first payload byte at the output". This module does the same job in
// 6, which as far as I can tell is the hardware floor for a 64-bit bus - my
// reasoning for that is below.
//
// =============================================================================
// WHY MY LAYERED VERSION WAS SLOW
// =============================================================================
// Every stage in the old design (a) waited for its own header, (b) re-aligned
// the byte stream into a residue register, and (c) registered its output. It's
// step (c) that really hurts: registering re-serialises the stream, so the next
// stage can't even start looking at its own header until a cycle later. Adding
// the 9 cycles up:
//
//     1  xgmii_rx output register
//     2  eth_rx  header cycles
//     1  eth_rx  output register   (2-byte realign)
//     3  ipv4_rx header cycles
//     1  ipv4_rx output register   (4-byte realign)
//     1  udp_rx  header cycle
//     -
//     9
//
// What took me a while to notice is that the header *decode* never needed any of
// those cycles. Ethernet (14) + IPv4 (20, with IHL=5) + UDP (8) = 42 bytes, and
// every one of them sits at a fixed, known byte offset. So every field lives at
// a fixed (word, lane) position and I can pull it out with a plain wire slice
// off a single registered input word. No FSM header states, no per-stage
// handshaking, no waiting around.
//
// =============================================================================
// THE BIT THAT MAKES IT WORK: 42 mod 8 == 2
// =============================================================================
// Number the 64-bit words coming off the wire, word 0 = frame bytes 0..7:
//
//   word 0  bytes  0.. 7   Ethernet: dst MAC, src MAC[0:1]
//   word 1  bytes  8..15   src MAC[2:5], EtherType,  IP[0..1]
//   word 2  bytes 16..23   IP[2..9]     (total length, flags/frag, protocol)
//   word 3  bytes 24..31   IP[10..17]   (checksum, src IP, dst IP hi)
//   word 4  bytes 32..39   IP[18..19], UDP[0..5]  (ports, UDP length)
//   word 5  bytes 40..47   UDP[6..7],  PAYLOAD BYTES 0..5   <-- first payload
//   word 6  bytes 48..55   PAYLOAD BYTES 6..13
//   word 7  bytes 56..63   PAYLOAD BYTES 14..21
//
// Payload byte 0 is at offset 42 = 5*8 + 2, so it physically arrives in word 5.
// Nothing can possibly come out before word 5 exists. I have to register the
// XGMII bus once (there's no way round that - you can't hang combinational parse
// logic straight off the PHY pins), and that puts word 5 in front of me in cycle
// 6. So 6 cycles is the floor, and that's what this module hits.
//
// The second half of that observation is the fun one. Because the headers are 42
// bytes and 42 mod 8 == 2, once I've emitted the 6 leftover payload bytes stuck
// in word 5, EVERY LATER WORD IS ALREADY PAYLOAD-ALIGNED. Payload bytes 6..13
// are exactly word 6; bytes 14..21 are exactly word 7. So:
//
//   * beat 0  = the top 6 bytes of word 5, shifted down 2 lanes (fixed wiring)
//   * beat N  = word 5+N, passed straight through, untouched
//
// Which means there's no barrel shifter, no residue register and no realignment
// pipeline anywhere in this design - the output datapath is one 2:1 mux on 48
// bits. What it costs me is that the FIRST beat carries 6 valid bytes instead of
// 8, which is a bit unusual - see the tkeep note in the port list.
//
// =============================================================================
// FRAMING COMES FROM THE UDP LENGTH, NOT FROM XGMII TERM
// =============================================================================
// My old design worked out where the packet ended by watching for the XGMII TERM
// character. Turns out that's wrong, for two reasons:
//
//  1. Ethernet pads every frame up to a 60-byte minimum. A 10-byte UDP payload
//     goes onto the wire with 8 bytes of zero padding glued on the end, and TERM
//     lands after the padding. So the old parser happily forwarded the padding
//     as if it were payload. My old testbench never caught it because it built a
//     52-byte frame by hand and never padded it - real NICs and switches do.
//
//  2. TERM in lane 0 means "the previous word was the last one", so you need a
//     cycle of lookahead on the data path to go back and fix up tkeep/tlast.
//
// But I already know the exact payload length from the UDP length field in word
// 4, a whole cycle before the first payload byte turns up. So I just count the
// bytes out and stop on time. Padding gets ignored for free, and the
// TERM-in-lane-0 lookahead problem goes away completely. TERM is now only a
// sanity check: if the frame ends before I've delivered the bytes the header
// promised, I terminate the beat with tuser=1 (more on that below).
//
// =============================================================================
// VALIDATION - all of it finishes before the first payload byte goes out
// =============================================================================
//   EtherType == 0x0800              known in cycle 2
//   Version/IHL == 0x45              known in cycle 2
//   not fragmented (MF=0, offset=0)  known in cycle 3
//   Protocol == 17 (UDP)             known in cycle 3
//   IPv4 header checksum == 0xFFFF   known in cycle 5
//   UDP length sane, matches IP      known in cycle 5
//   dst port matches filter          known in cycle 5
//
// So this is cut-through with *no speculation*: a packet never gets partly
// emitted and then retracted. That only works because the checksum covers the IP
// header alone, and the IP header is complete in word 4. If I ever needed to
// check something that spans the payload (a UDP checksum, say) I'd have to
// forward speculatively and raise tuser late - which is the other reason the
// tuser plumbing is already in here.
//
// =============================================================================
// BACKPRESSURE
// =============================================================================
// There isn't any, and there can't be - the wire doesn't stop for us.
// m_axis_tready is accepted as an input but then ignored. If it's ever low while
// I'm driving tvalid, stat_backpressure counts it, so at least you find out
// something's wrong. If your consumer can stall, stick an axis_fifo downstream.
// That costs latency, which is why I haven't put one in here.
// =============================================================================

`default_nettype none
`timescale 1ns / 1ps

module udp_rx_parser #(
    // 0 = combinational output  -> 6-cycle latency, output comes through a small
    //     mux off the input register. This is the default.
    // 1 = registered output     -> 7-cycle latency, but a clean registered
    //     module boundary. It costs one cycle, so only turn it on if your
    //     consumer is a long way across the die and you need the timing margin.
    parameter OUTPUT_REG = 0
)(
    input  wire        clk,
    input  wire        rst,

    // ---- Raw XGMII from the PHY ----
    input  wire [63:0] xgmii_rxd,
    input  wire [7:0]  xgmii_rxc,

    // ---- Runtime configuration ----
    // Multicast feed selection. Exchanges put each feed on its own UDP port, so
    // filtering right here saves the downstream decoder from ever seeing traffic
    // it would just throw away. It's free in latency terms - the port is already
    // known in cycle 5.
    input  wire        cfg_port_filter_en,
    input  wire [15:0] cfg_dst_port,

    // ---- Payload AXI-Stream ----
    // NOTE ON tkeep: the first beat of a packet carries 6 bytes (tkeep=0x3F),
    // every beat after that carries 8, and the final beat carries whatever's
    // left over. Payload byte 0 is always in lane 0. This is the one place I
    // bend the usual "only the last beat is allowed to be partial" convention,
    // and it's what buys me the cycle - see the 42 mod 8 stuff above.
    //
    // NOTE ON tuser: goes high with tlast to mean "this packet is damaged, throw
    // it away". I raise it when the frame ends before the UDP length said it
    // would, or when an XGMII error character shows up mid-frame. On an abort
    // the final beat can have tkeep=0 (nothing survived) - it's only there to
    // release the sink downstream.
    output wire [63:0] m_axis_tdata,
    output wire [7:0]  m_axis_tkeep,
    output wire        m_axis_tlast,
    output wire        m_axis_tuser,
    output wire        m_axis_tvalid,
    input  wire        m_axis_tready,

    // ---- Parsed header fields ----
    // These hold steady from the hdr_valid strobe until the next packet's
    // strobe, so downstream can just register them alongside the first payload
    // beat and not worry about timing.
    output reg  [47:0] dst_mac,
    output reg  [47:0] src_mac,
    output reg  [15:0] ethertype,
    output reg  [31:0] src_ip,
    output reg  [31:0] dst_ip,
    output reg  [15:0] total_length,
    output reg  [15:0] src_port,
    output reg  [15:0] dst_port,
    output reg  [15:0] udp_length,
    output wire        hdr_valid,      // 1 cycle, aligned with the first beat

    // ---- Counters ----
    output reg  [31:0] stat_frames,          // frames seen on the wire
    output reg  [31:0] stat_accepted,        // frames delivered as payload
    output reg  [31:0] stat_drop_ethertype,  // not IPv4
    output reg  [31:0] stat_drop_vlan,       // VLAN-tagged (see note below)
    output reg  [31:0] stat_drop_ihl,        // not version 4 / IHL != 5
    output reg  [31:0] stat_drop_frag,       // fragmented
    output reg  [31:0] stat_drop_proto,      // not UDP
    output reg  [31:0] stat_drop_cksum,      // bad IPv4 header checksum
    output reg  [31:0] stat_drop_len,        // UDP length insane or != IP total
    output reg  [31:0] stat_drop_port,       // rejected by the port filter
    output reg  [31:0] stat_runt,            // frame ended inside the headers
    output reg  [31:0] stat_abort,           // frame ended inside the payload
    output reg  [31:0] stat_xgmii_err,       // XGMII error character seen
    output reg  [31:0] stat_backpressure     // tready low while we drove tvalid
);

// ---- XGMII control characters ----
localparam [7:0] XGMII_START = 8'hFB;
localparam [7:0] XGMII_TERM  = 8'hFD;
localparam [7:0] XGMII_ERR   = 8'hFE;

localparam [15:0] ETHERTYPE_IPV4  = 16'h0800;
localparam [15:0] ETHERTYPE_VLAN  = 16'h8100;
localparam [15:0] ETHERTYPE_QINQ  = 16'h88A8;
localparam [7:0]  PROTO_UDP       = 8'd17;
localparam [7:0]  IPV4_VER_IHL    = 8'h45;   // version 4, 20-byte header

// =============================================================================
// Front end. Decode the XGMII control lanes combinationally straight off the raw
// bus, then register the answer alongside the data. Doing it on this side of the
// pipeline register keeps the control decode off the parse path entirely.
// =============================================================================

wire raw_start = xgmii_rxc[0] && (xgmii_rxd[7:0] == XGMII_START);

wire [7:0] raw_term_lane;
wire [7:0] raw_err_lane;
genvar gi;
generate
    for (gi = 0; gi < 8; gi = gi + 1) begin : g_ctrl
        assign raw_term_lane[gi] = xgmii_rxc[gi] &&
                                   (xgmii_rxd[gi*8 +: 8] == XGMII_TERM);
        assign raw_err_lane[gi]  = xgmii_rxc[gi] &&
                                   (xgmii_rxd[gi*8 +: 8] == XGMII_ERR);
    end
endgenerate

wire raw_term = |raw_term_lane;
wire raw_err  = |raw_err_lane;

// How many bytes in this word are real data? Everything below the TERM lane.
// casez is priority ordered, so this finds the LOWEST set bit for me.
reg [3:0] raw_nbytes;
always @(*) begin
    casez (raw_term_lane)
        8'b???????1: raw_nbytes = 4'd0;
        8'b??????1?: raw_nbytes = 4'd1;
        8'b?????1??: raw_nbytes = 4'd2;
        8'b????1???: raw_nbytes = 4'd3;
        8'b???1????: raw_nbytes = 4'd4;
        8'b??1?????: raw_nbytes = 4'd5;
        8'b?1??????: raw_nbytes = 4'd6;
        8'b1???????: raw_nbytes = 4'd7;
        default:     raw_nbytes = 4'd8;   // no TERM: all eight bytes are data
    endcase
end

// ---- The one and only pipeline register in this module ----
// d1 holds wire word W during cycle W+1. Everything below reads d1.
reg [63:0] d1;
reg        d1_start;
reg        d1_term;
reg        d1_err;
reg [3:0]  d1_nb;

always @(posedge clk) begin
    if (rst) begin
        d1_start <= 1'b0;
        d1_term  <= 1'b0;
        d1_err   <= 1'b0;
        d1_nb    <= 4'd0;
        d1       <= 64'h0;
    end else begin
        d1       <= xgmii_rxd;
        d1_start <= raw_start;
        d1_term  <= raw_term;
        d1_err   <= raw_err;
        d1_nb    <= raw_nbytes;
    end
end

// ---- Byte lanes of d1. Lane 0 is the EARLIEST byte in time (easy to forget). ----
wire [7:0] b0 = d1[ 7: 0];
wire [7:0] b1 = d1[15: 8];
wire [7:0] b2 = d1[23:16];
wire [7:0] b3 = d1[31:24];
wire [7:0] b4 = d1[39:32];
wire [7:0] b5 = d1[47:40];
wire [7:0] b6 = d1[55:48];
wire [7:0] b7 = d1[63:56];

// Protocol fields are big-endian, so a 16-bit field is {earlier, later}.
wire [15:0] p01 = {b0, b1};
wire [15:0] p23 = {b2, b3};
wire [15:0] p45 = {b4, b5};
wire [15:0] p67 = {b6, b7};

// =============================================================================
// Frame tracking
// =============================================================================
reg        rx;        // a frame is in flight and d1 holds one of its words
reg [2:0]  wcnt;      // which word of the frame is in d1 (saturates at 7)

// Header check results, each latched in the cycle its field arrives.
reg f_eth;            // EtherType is IPv4
reg f_vlan;           // EtherType is a VLAN tag (counted separately)
reg f_ihl;            // version 4, IHL 5
reg f_frag;           // not a fragment
reg f_proto;          // protocol is UDP
reg f_err;            // an XGMII error character landed in this frame

// IPv4 header checksum accumulator. There are only ten 16-bit words to add, so
// the running sum can never get past 10 * 0xFFFF = 0x9FFF6. Twenty bits is
// therefore always enough and I don't need to fold anything as I go.
reg [19:0] ck_acc;

// total_length - 20, worked out early in cycle 3 so that cycle 5 only has to do
// an equality compare instead of an add.
reg [15:0] ip_pay_len;
reg        f_iplen;   // total_length was at least 28 (20 IP + 8 UDP)

// ---- Payload beat schedule ----
//
// My first version of this kept a 16-bit "bytes remaining" counter and compared
// it against 6 and 8 in the same cycle the beat went out. That puts three 16-bit
// comparators between the counter and tkeep/tlast, and since the counter also
// gets updated from those same comparisons it closes a 16-bit combinational loop
// straight back into its own reset pin. Post-route it turned up as the critical
// path, which is how I found it.
//
// The fix is that the entire schedule is knowable the moment I read the UDP
// length: beat 0 carries the 6 payload bytes sharing word 5 with the UDP header,
// then every beat is 8 bytes until whatever remainder is left at the end. So
// work it all out once in cycle 5 and keep nothing but a beat counter after
// that - a decrement and an equality test instead of a subtract and three
// magnitude compares.
reg [3:0]  beat_size_r;      // bytes the beat now on the bus carries
reg        beat_is_last_r;   // ...and whether it is the final one
reg [3:0]  last_size_r;      // size of the final beat, precomputed
reg [12:0] beats_left;       // beats still to send, including the current one
reg        emit_en;   // this frame passed every check; payload may flow
reg        hdr_valid_i;

// Verdict pipeline. I latch the accept/reject reasons in cycle 5 and only turn
// them into counter increments in cycle 6, which keeps the priority encoder off
// the checksum path. A diagnostic counter lagging by one cycle doesn't matter to
// anyone.
reg v_stb, v_ck_ok, v_len_ok, v_port_ok, v_term;

// =============================================================================
// The accept decision. Everything here happens in cycle 5 (wcnt == 4), one cycle
// before the first payload byte is available, and gets registered into emit_en.
// This turned out to be the critical path of the whole design, so it's worth
// writing down why it ended up looking so odd.
//
// The textbook way to finish an IPv4 checksum is: add the last word, fold the
// carries down twice, then compare against 0xFFFF. That's four adders back to
// back, each one waiting on the one before it. Built that way, this path came
// out at 9.35 ns post-route on a -1 speed grade 7-series part (13 CARRY4s in
// series) against a 6.4 ns budget, so it clearly had to change.
//
// The way out is that my accumulator is only 20 bits wide, so the "high half"
// I'd be folding back in is just 4 bits. Write S = ck_acc + last_word and fold:
//
//   f1 = S[19:16] + S[15:0]           (17 bits)
//   f2 = f1[15:0] + f1[16]            (16 bits)   valid iff f2 == 0xFFFF
//
// If f1 overflows then f1 <= 0xF + 0xFFFF = 0x1000E, so f1[15:0] <= 0x000E and
// f2 <= 0x000F, which can never be 0xFFFF. So the second fold can't ever produce
// a pass, and the whole test collapses down to  S[19:16] + S[15:0] == 0xFFFF.
//
// Then, since S[19:16] is only four bits, 0xFFFF - S[19:16] == {12'hFFF, ~S[19:16]},
// so that add-and-compare collapses a second time into a plain pattern match
// with no carry propagation left in it at all:
//
//   valid  <=>  S[15:4] == 0xFFF  and  S[3:0] == ~S[19:16]
//
// One 20-bit add feeding two levels of LUT, instead of four dependent adders. I
// convinced myself on paper that it's exactly equivalent, and then made the
// testbench flip every single bit of the checksum field to be sure.
// =============================================================================
wire [19:0] ck_sum = ck_acc + {4'd0, p01};          // + IP words 18..19
wire        ck_ok  = (ck_sum[15:4] == 12'hFFF) && (ck_sum[3:0] == ~ck_sum[19:16]);

// p67 is the UDP length this cycle. Since ip_pay_len was already worked out back
// in cycle 3, this is just a compare instead of "total_length == p67 + 20".
wire len_ok  = f_iplen && (p67 >= 16'd8) && (ip_pay_len == p67);
wire port_ok = !cfg_port_filter_en || (p45 == cfg_dst_port);

// d1_term here means the frame stopped dead at word 4, i.e. before the UDP
// header was even finished - that's a runt, and it's never acceptable.
wire accept = f_eth & f_ihl & f_frag & f_proto & ~f_err &
              ck_ok & len_ok & port_ok & ~d1_term;

// =============================================================================
// Payload beat generation
//
//   cycle 6      wcnt == 5   beat 0: the 6 payload bytes living in word 5,
//                            shifted down two lanes by fixed wiring
//   cycle 7..N   wcnt >= 6   beat k: word 5+k, completely untouched
// =============================================================================
wire first_beat = rx && emit_en && (wcnt == 3'd5);
wire cont_beat  = rx && emit_en && (wcnt >= 3'd6);
wire any_beat   = first_beat || cont_beat;

// How many bytes I WANT to send this cycle, and whether this is the final beat.
// Both of these are registers, worked out a cycle ahead - see beat_size_r.
wire [3:0] n_want = beat_size_r;

// ...and how many the wire actually gave me (these only differ if the frame got
// truncated).
wire [3:0] n_avail = first_beat ? ((d1_nb > 4'd2) ? (d1_nb - 4'd2) : 4'd0)
                                : d1_nb;
wire [3:0] n_emit  = ((d1_term || d1_err) && (n_avail < n_want)) ? n_avail : n_want;

// The frame died before it handed over everything the UDP length promised. This
// is really just "bytes emitted so far < payload length", rewritten against the
// schedule I precomputed: if this was supposed to be the final beat then I only
// fall short when the wire ran out early, and if it wasn't the final beat then
// more was promised anyway.
wire beat_abort = any_beat && (d1_term || d1_err) &&
                  ((n_emit < n_want) || !beat_is_last_r);

wire beat      = any_beat && (n_want != 4'd0) &&
                 ((n_emit != 4'd0) || beat_abort);
wire beat_last = beat && (beat_is_last_r || beat_abort);

// ---- Cycle-5 schedule arithmetic (p67 is the UDP length in that cycle) ----
wire [15:0] pay_len  = p67 - 16'd8;    // payload bytes in this datagram
wire [15:0] pay_rem  = p67 - 16'd14;   // ...after beat 0 has taken its 6
wire        pay_tiny = (pay_len <= 16'd6);
// Whole 8-byte beats after beat 0, plus one more if there's a remainder.
// 13 bits is always enough here: beats_left only ever gets loaded when the
// length check passed, and that forces udp_length == total_length - 20 <= 65515,
// which works out to at most 1 + 8187 + 1 = 8189 beats.
wire [12:0] beats_init = 13'd1 + pay_rem[15:3] + {12'd0, |pay_rem[2:0]};
wire [3:0]  last_init  = (pay_rem[2:0] == 3'd0) ? 4'd8 : {1'b0, pay_rem[2:0]};

// Fixed 2-lane shift on the first beat, straight through for every other one.
wire [63:0] beat_data = first_beat ? {16'h0, d1[63:16]} : d1;

reg [7:0] beat_keep;
always @(*) begin
    case (n_emit)
        4'd0:    beat_keep = 8'b00000000;
        4'd1:    beat_keep = 8'b00000001;
        4'd2:    beat_keep = 8'b00000011;
        4'd3:    beat_keep = 8'b00000111;
        4'd4:    beat_keep = 8'b00001111;
        4'd5:    beat_keep = 8'b00011111;
        4'd6:    beat_keep = 8'b00111111;
        4'd7:    beat_keep = 8'b01111111;
        default: beat_keep = 8'b11111111;
    endcase
end

// =============================================================================
// Main sequential block
// =============================================================================
always @(posedge clk) begin
    if (rst) begin
        rx                  <= 1'b0;
        wcnt                <= 3'd0;
        f_eth               <= 1'b0;
        f_vlan              <= 1'b0;
        f_ihl               <= 1'b0;
        f_frag              <= 1'b0;
        f_proto             <= 1'b0;
        f_err               <= 1'b0;
        f_iplen             <= 1'b0;
        ck_acc              <= 20'd0;
        ip_pay_len          <= 16'd0;
        beats_left          <= 13'd0;
        beat_size_r         <= 4'd0;
        beat_is_last_r      <= 1'b1;
        last_size_r         <= 4'd0;
        emit_en             <= 1'b0;
        hdr_valid_i         <= 1'b0;
        v_stb               <= 1'b0;
        v_ck_ok             <= 1'b0;
        v_len_ok            <= 1'b0;
        v_port_ok           <= 1'b0;
        v_term              <= 1'b0;
        dst_mac             <= 48'h0;
        src_mac             <= 48'h0;
        ethertype           <= 16'h0;
        src_ip              <= 32'h0;
        dst_ip              <= 32'h0;
        total_length        <= 16'h0;
        src_port            <= 16'h0;
        dst_port            <= 16'h0;
        udp_length          <= 16'h0;
        stat_frames         <= 32'h0;
        stat_accepted       <= 32'h0;
        stat_drop_ethertype <= 32'h0;
        stat_drop_vlan      <= 32'h0;
        stat_drop_ihl       <= 32'h0;
        stat_drop_frag      <= 32'h0;
        stat_drop_proto     <= 32'h0;
        stat_drop_cksum     <= 32'h0;
        stat_drop_len       <= 32'h0;
        stat_drop_port      <= 32'h0;
        stat_runt           <= 32'h0;
        stat_abort          <= 32'h0;
        stat_xgmii_err      <= 32'h0;
        stat_backpressure   <= 32'h0;
    end else begin
        hdr_valid_i <= 1'b0;
        v_stb       <= 1'b0;

        // ------------------------------------------------------------------
        // A new frame always wins. Giving START priority over everything else
        // is what makes back-to-back frames work, i.e. when the next START
        // turns up in the word straight after the previous TERM.
        // ------------------------------------------------------------------
        if (d1_start) begin
            rx          <= 1'b1;
            wcnt        <= 3'd0;
            f_eth       <= 1'b0;
            f_vlan      <= 1'b0;
            f_ihl       <= 1'b0;
            f_frag      <= 1'b0;
            f_proto     <= 1'b0;
            f_err       <= 1'b0;
            f_iplen     <= 1'b0;
            ck_acc      <= 20'd0;
            emit_en     <= 1'b0;
            beats_left  <= 13'd0;
            beat_size_r <= 4'd0;
            stat_frames <= stat_frames + 32'd1;

        end else if (rx) begin
            if (wcnt != 3'd7)
                wcnt <= wcnt + 3'd1;

            if (d1_err) begin
                f_err          <= 1'b1;
                stat_xgmii_err <= stat_xgmii_err + 32'd1;
            end

            case (wcnt)

            // ---- word 0: bytes 0..7 -------------------------------------
            3'd0: begin
                dst_mac        <= {b0, b1, b2, b3, b4, b5};
                src_mac[47:32] <= {b6, b7};
            end

            // ---- word 1: bytes 8..15 ------------------------------------
            3'd1: begin
                src_mac[31:0] <= {b0, b1, b2, b3};
                ethertype     <= p45;
                f_eth         <= (p45 == ETHERTYPE_IPV4);
                // I only detect VLAN so the counter can tell you why the frame
                // disappeared. Actually supporting it would move every offset
                // below by 4 (and 46 mod 8 == 6, so the first beat would carry
                // 2 bytes instead of 6) - that's a second set of fixed slices
                // and no extra cycles, but I've left it out of scope for now.
                f_vlan        <= (p45 == ETHERTYPE_VLAN) || (p45 == ETHERTYPE_QINQ);
                // IPv4 options (IHL > 5) would make the payload offset vary,
                // which needs a real barrel shifter and costs a cycle. Exchange
                // feeds never use them, so I just drop those frames.
                f_ihl         <= (b6 == IPV4_VER_IHL);
                ck_acc        <= {4'd0, p67};              // IP words 0..1
            end

            // ---- word 2: bytes 16..23 = IP[2..9] ------------------------
            3'd2: begin
                total_length <= p01;
                // Chew through the length check early so cycle 5 has no adder.
                ip_pay_len   <= p01 - 16'd20;
                f_iplen      <= (p01 >= 16'd28);
                // p45 = {flags, fragment offset}: bit 13 is MF, bits 12:0 are
                // the offset. If either is set this is a fragment, which means
                // the bytes after the IP header are NOT a UDP header.
                f_frag       <= ~p45[13] && (p45[12:0] == 13'd0);
                f_proto      <= (b7 == PROTO_UDP);
                ck_acc       <= ck_acc + {4'd0, p01} + {4'd0, p23}
                                       + {4'd0, p45} + {4'd0, p67};
            end

            // ---- word 3: bytes 24..31 = IP[10..17] ----------------------
            3'd3: begin
                src_ip        <= {p23, p45};
                dst_ip[31:16] <= p67;
                ck_acc        <= ck_acc + {4'd0, p01} + {4'd0, p23}
                                        + {4'd0, p45} + {4'd0, p67};
            end

            // ---- word 4: bytes 32..39 = IP[18..19] + UDP[0..5] ----------
            // By now I know everything I need to accept or reject the packet.
            3'd4: begin
                dst_ip[15:0] <= p01;
                src_port     <= p23;
                dst_port     <= p45;
                udp_length   <= p67;

                emit_en      <= accept;
                hdr_valid_i  <= accept;
                // Beat 0 carries the 6 payload bytes sharing word 5 with the
                // tail end of the UDP header - or the whole payload, if the
                // datagram is short enough that it all fits.
                beat_size_r     <= (!accept) ? 4'd0 :
                                   pay_tiny  ? pay_len[3:0] : 4'd6;
                beat_is_last_r  <= pay_tiny;
                last_size_r     <= pay_tiny ? pay_len[3:0] : last_init;
                beats_left      <= (!accept) ? 13'd0 :
                                   pay_tiny  ? 13'd1 : beats_init;

                // The counters are diagnostics, not datapath, so the priority
                // encoding that decides WHICH one to bump can wait until next
                // cycle. When I left it here it hung a priority chain plus the
                // clock-enable of fourteen 32-bit counters off the back of the
                // checksum compare, which is how I ended up with a stats
                // register as the endpoint of my critical path. So all I do
                // here is flop the three late verdicts.
                v_stb    <= 1'b1;
                v_ck_ok  <= ck_ok;
                v_len_ok <= len_ok;
                v_port_ok<= port_ok;
                v_term   <= d1_term;
            end

            default: ; // words 5+ are payload, the beat logic deals with them

            endcase

            // The frame ended somewhere inside the headers. I route this
            // through the same verdict strobe as everything else so there's
            // only one place in the design that ever touches stat_runt.
            if (d1_term && (wcnt < 3'd4)) begin
                v_stb  <= 1'b1;
                v_term <= 1'b1;
            end

            // Tick payload bytes off the schedule as they go out.
            if (beat) begin
                if (beat_last) begin
                    beats_left     <= 13'd0;
                    beat_size_r    <= 4'd0;
                    beat_is_last_r <= 1'b1;
                    emit_en        <= 1'b0;
                    if (beat_abort)
                        stat_abort <= stat_abort + 32'd1;
                end else begin
                    // Every beat except the last one carries a full 8 bytes.
                    beats_left     <= beats_left - 13'd1;
                    beat_size_r    <= (beats_left == 13'd2) ? last_size_r : 4'd8;
                    beat_is_last_r <= (beats_left == 13'd2);
                end
            end

            if (d1_term)
                rx <= 1'b0;
        end

        // ------------------------------------------------------------------
        // Verdict retirement, running one cycle behind the decision itself.
        // Exactly one counter moves per frame, which makes the numbers easy to
        // reason about when you're staring at them after a sim run.
        // ------------------------------------------------------------------
        if (v_stb) begin
            if (v_term)          stat_runt           <= stat_runt           + 32'd1;
            else if (f_err)      ; // already counted by stat_xgmii_err
            else if (!f_eth) begin
                if (f_vlan)      stat_drop_vlan      <= stat_drop_vlan      + 32'd1;
                else             stat_drop_ethertype <= stat_drop_ethertype + 32'd1;
            end
            else if (!f_ihl)     stat_drop_ihl       <= stat_drop_ihl       + 32'd1;
            else if (!f_frag)    stat_drop_frag      <= stat_drop_frag      + 32'd1;
            else if (!f_proto)   stat_drop_proto     <= stat_drop_proto     + 32'd1;
            else if (!v_ck_ok)   stat_drop_cksum     <= stat_drop_cksum     + 32'd1;
            else if (!v_len_ok)  stat_drop_len       <= stat_drop_len       + 32'd1;
            else if (!v_port_ok) stat_drop_port      <= stat_drop_port      + 32'd1;
            else                 stat_accepted       <= stat_accepted       + 32'd1;
        end

        // If the consumer can't keep up at line rate then something is wrong
        // with the design, not with this particular packet - so I count it
        // loudly instead of pretending I can stall.
        if (m_axis_tvalid && !m_axis_tready)
            stat_backpressure <= stat_backpressure + 32'd1;
    end
end

// =============================================================================
// Output stage
// =============================================================================
generate
if (OUTPUT_REG == 0) begin : g_comb_out
    // Straight out of the input register through the beat mux - 6-cycle path.
    assign m_axis_tdata  = beat_data;
    assign m_axis_tkeep  = beat_keep;
    assign m_axis_tlast  = beat_last;
    assign m_axis_tuser  = beat_abort;
    assign m_axis_tvalid = beat;
    assign hdr_valid     = hdr_valid_i;
end else begin : g_reg_out
    // One more flop on everything - 7-cycle path, but a clean boundary.
    reg [63:0] o_tdata;
    reg [7:0]  o_tkeep;
    reg        o_tlast, o_tuser, o_tvalid, o_hdr;
    always @(posedge clk) begin
        if (rst) begin
            o_tvalid <= 1'b0;
            o_tlast  <= 1'b0;
            o_tuser  <= 1'b0;
            o_tkeep  <= 8'h0;
            o_tdata  <= 64'h0;
            o_hdr    <= 1'b0;
        end else begin
            o_tdata  <= beat_data;
            o_tkeep  <= beat_keep;
            o_tlast  <= beat_last;
            o_tuser  <= beat_abort;
            o_tvalid <= beat;
            o_hdr    <= hdr_valid_i;
        end
    end
    assign m_axis_tdata  = o_tdata;
    assign m_axis_tkeep  = o_tkeep;
    assign m_axis_tlast  = o_tlast;
    assign m_axis_tuser  = o_tuser;
    assign m_axis_tvalid = o_tvalid;
    assign hdr_valid     = o_hdr;
end
endgenerate

endmodule

`default_nettype wire
