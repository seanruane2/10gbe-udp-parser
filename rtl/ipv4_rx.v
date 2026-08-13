// ipv4_rx.v
// =============================================================================
// Third stage of the pipeline. Reads the IPv4 header off the front of an
// incoming IP packet, validates the checksum, and only forwards UDP packets
// to the next stage (anything else gets dropped).
//
// The IPv4 header layout (20 bytes, assuming no IP options):
//
//   byte  0    : version (4) and IHL (5)  -> always 0x45 for us
//   byte  1    : DSCP / ECN               (QoS stuff we ignore)
//   bytes 2..3 : total length             (whole IP packet, header + payload)
//   bytes 4..5 : identification           (ignored - no fragments)
//   bytes 6..7 : flags + fragment offset  (ignored)
//   byte  8    : TTL                      (ignored)
//   byte  9    : protocol                 (17 = UDP)         <- we check this
//   bytes 10..11: header checksum         <- we validate this
//   bytes 12..15: source IP address
//   bytes 16..19: destination IP address
//
// =============================================================================
// IPv4 checksum in plain English
// =============================================================================
// 1. Treat the 20-byte header as ten 16-bit numbers.
// 2. Add them all up using ordinary unsigned addition.
// 3. Whenever the sum overflows past 16 bits, take the high half and add it
//    back into the low half. Repeat until the high half is zero. This is
//    called "one's complement addition with end-around carry".
// 4. Flip all 16 bits (one's complement).
// 5. That number gets written into the checksum field by the sender.
//
// On our side, we redo the sum (which already includes the checksum field
// from the wire) and fold it. If the result is 0xFFFF, the header is valid.
//
// =============================================================================
// Alignment, again
// =============================================================================
// The header is 20 bytes - that's 2 full bus cycles + 4 bytes spilling into
// a third cycle. Those 4 leftover bytes are the first 4 bytes of the UDP
// header! We park them in a 4-byte residue register and prepend them onto
// the next cycle's data so udp_rx sees a nicely-aligned UDP stream.
// Same trick as eth_rx, just shifting by 4 instead of 2 bytes.
// =============================================================================

`default_nettype none
`timescale 1ns / 1ps

module ipv4_rx (
    input  wire        clk,
    input  wire        rst,

    // Input: IP packet bytes (from eth_rx, already realigned)
    input  wire [63:0] s_axis_tdata,
    input  wire [7:0]  s_axis_tkeep,
    input  wire        s_axis_tlast,
    input  wire        s_axis_tvalid,
    output wire        s_axis_tready,

    // Output: UDP segment bytes (IP header stripped, realigned)
    output reg  [63:0] m_axis_tdata,
    output reg  [7:0]  m_axis_tkeep,
    output reg         m_axis_tlast,
    output reg         m_axis_tvalid,
    input  wire        m_axis_tready,    // not used

    // Sideband: parsed IP fields, valid for one cycle after the IP header arrives
    output reg  [31:0] src_ip,
    output reg  [31:0] dst_ip,
    output reg  [15:0] total_length,
    output reg  [7:0]  protocol,
    output reg         sideband_valid,

    // Stats
    output reg  [31:0] stat_udp_rx,
    output reg  [31:0] stat_dropped_proto,
    output reg  [31:0] stat_dropped_cksum
);

// ---- States ----
localparam S_HEADER1 = 3'd0;  // IP bytes 0..7
localparam S_HEADER2 = 3'd1;  // IP bytes 8..15
localparam S_HEADER3 = 3'd2;  // IP bytes 16..19 + UDP bytes 0..3
localparam S_DATA    = 3'd3;  // realigned UDP stream
localparam S_FLUSH   = 3'd4;  // flush leftover residue at end of packet
localparam S_DROP    = 3'd5;  // bad checksum or non-UDP - eat the rest

reg [2:0] state = S_HEADER1;

localparam PROTO_UDP = 8'd17;

assign s_axis_tready = 1'b1;

// Pieces of header we capture while parsing - they're needed later cycles.
reg [15:0] total_length_r;
reg [7:0]  proto_r;
reg [31:0] src_ip_r;

// Rolling 32-bit checksum accumulator.
reg [31:0] cksum_acc;

// Residue: the 4 UDP bytes that arrived in cycle 3 of the IP header.
reg [31:0] residue;

// Bytes left to flush after tlast - could be 1, 2, 3, or 4.
reg [2:0] flush_count;

// Local temporaries that have to live at module scope (Verilog-2001 rule).
reg [31:0] final_acc;
reg [15:0] cksum_result;
reg [3:0]  in_bytes;

// ---- Helpers ----

function [3:0] count_keep;
    input [7:0] keep;
    begin
        case (keep)
            8'b00000001: count_keep = 4'd1;
            8'b00000011: count_keep = 4'd2;
            8'b00000111: count_keep = 4'd3;
            8'b00001111: count_keep = 4'd4;
            8'b00011111: count_keep = 4'd5;
            8'b00111111: count_keep = 4'd6;
            8'b01111111: count_keep = 4'd7;
            8'b11111111: count_keep = 4'd8;
            default:     count_keep = 4'd0;
        endcase
    end
endfunction

function [7:0] keep_mask;
    input [3:0] n;
    begin
        case (n)
            4'd0: keep_mask = 8'b00000000;
            4'd1: keep_mask = 8'b00000001;
            4'd2: keep_mask = 8'b00000011;
            4'd3: keep_mask = 8'b00000111;
            4'd4: keep_mask = 8'b00001111;
            4'd5: keep_mask = 8'b00011111;
            4'd6: keep_mask = 8'b00111111;
            4'd7: keep_mask = 8'b01111111;
            default: keep_mask = 8'b11111111;
        endcase
    end
endfunction

// Fold a 32-bit one's-complement sum down to a 16-bit value.
// Add the high half to the low half. If THAT overflows too, add the carry
// back in one more time. Two folds is always enough for a 32-bit sum.
function [15:0] fold32;
    input  [31:0] sum;
    reg    [16:0] tmp;
    begin
        tmp    = sum[31:16] + sum[15:0];
        fold32 = tmp[15:0]  + {15'd0, tmp[16]};
    end
endfunction

always @(posedge clk) begin
    if (rst) begin
        state              <= S_HEADER1;
        m_axis_tvalid      <= 1'b0;
        m_axis_tlast       <= 1'b0;
        m_axis_tkeep       <= 8'h0;
        m_axis_tdata       <= 64'h0;
        sideband_valid     <= 1'b0;
        stat_udp_rx        <= 32'h0;
        stat_dropped_proto <= 32'h0;
        stat_dropped_cksum <= 32'h0;
        cksum_acc          <= 32'h0;
        flush_count        <= 3'h0;
        residue            <= 32'h0;
    end else begin
        m_axis_tvalid  <= 1'b0;
        m_axis_tlast   <= 1'b0;
        sideband_valid <= 1'b0;

        case (state)

            // -----------------------------------------------------------------
            // HEADER1: IP bytes 0..7
            //   byte 0:  Version+IHL (0x45)
            //   byte 1:  DSCP/ECN
            //   bytes 2..3: total length
            //   bytes 4..5: identification
            //   bytes 6..7: flags + fragment offset
            //
            // Start the checksum by summing words 0..3.
            // -----------------------------------------------------------------
            S_HEADER1: if (s_axis_tvalid) begin 
                // Stash total length for the sideband later.
                // Big-endian: byte 2 is the high byte of the 16-bit value.
                total_length_r <= {s_axis_tdata[23:16], s_axis_tdata[31:24]};

                // Sum of words 0..3 (each word is 2 bytes, MSB first on wire).
                cksum_acc <= {s_axis_tdata[ 7: 0], s_axis_tdata[15: 8]}
                           + {s_axis_tdata[23:16], s_axis_tdata[31:24]}
                           + {s_axis_tdata[39:32], s_axis_tdata[47:40]}
                           + {s_axis_tdata[55:48], s_axis_tdata[63:56]};

                if (s_axis_tlast) begin
                    stat_dropped_proto <= stat_dropped_proto + 1;
                    state <= S_HEADER1;
                end else
                    state <= S_HEADER2;
            end

            // -----------------------------------------------------------------
            // HEADER2: IP bytes 8..15
            //   byte 8:  TTL
            //   byte 9:  Protocol (17 = UDP)
            //   bytes 10..11: header checksum
            //   bytes 12..15: source IP address
            //
            // Add words 4..7 to the checksum.
            // -----------------------------------------------------------------
            S_HEADER2: if (s_axis_tvalid) begin
                // Save protocol byte for later (we check it in HEADER3).
                proto_r <= s_axis_tdata[15:8];

                // Save source IP - it's 4 bytes from bytes 12..15.
                // Stored big-endian: src_ip_r[31:24] = byte 12 = first byte.
                src_ip_r <= {s_axis_tdata[39:32],
                             s_axis_tdata[47:40],
                             s_axis_tdata[55:48],
                             s_axis_tdata[63:56]};

                // Continue the checksum sum.
                cksum_acc <= cksum_acc
                           + {s_axis_tdata[ 7: 0], s_axis_tdata[15: 8]}
                           + {s_axis_tdata[23:16], s_axis_tdata[31:24]}
                           + {s_axis_tdata[39:32], s_axis_tdata[47:40]}
                           + {s_axis_tdata[55:48], s_axis_tdata[63:56]};

                if (s_axis_tlast) begin
                    stat_dropped_proto <= stat_dropped_proto + 1;
                    state <= S_HEADER1;
                end else
                    state <= S_HEADER3;
            end

            // -----------------------------------------------------------------
            // HEADER3: bytes 16..23 of the stream
            //   bytes 16..19 are still IP header (destination IP).
            //   bytes 20..23 are the first 4 bytes of the UDP header.
            //
            // Finalize the checksum, decide pass / drop, and stash the 4
            // UDP bytes in the residue register so they can lead the
            // realigned output stream next cycle.
            // -----------------------------------------------------------------
            S_HEADER3: if (s_axis_tvalid) begin
                // The 4 leftover bytes (bytes 20..23 of the stream) are the
                // first 4 bytes of the UDP header. Save them.
                residue <= s_axis_tdata[63:32];

                // Capture sideband fields.
                src_ip       <= src_ip_r;
                total_length <= total_length_r;
                protocol     <= proto_r;
                dst_ip       <= {s_axis_tdata[ 7: 0],
                                 s_axis_tdata[15: 8],
                                 s_axis_tdata[23:16],
                                 s_axis_tdata[31:24]};
                sideband_valid <= 1'b1;

                // Finalize the checksum: add words 8..9 (DST IP) and fold.
                // Blocking '=' is fine here - we use these values immediately
                // and they're not visible outside the always block.
                final_acc    = cksum_acc
                             + {s_axis_tdata[ 7: 0], s_axis_tdata[15: 8]}
                             + {s_axis_tdata[23:16], s_axis_tdata[31:24]};
                cksum_result = fold32(final_acc);

                // Decide what to do with this packet.
                if (cksum_result != 16'hFFFF) begin
                    // Checksum mismatch - corrupted header, drop.
                    stat_dropped_cksum <= stat_dropped_cksum + 1;
                    state <= s_axis_tlast ? S_HEADER1 : S_DROP;
                end else if (proto_r != PROTO_UDP) begin
                    // It's a different layer-4 protocol (TCP, ICMP, ...) - drop.
                    stat_dropped_proto <= stat_dropped_proto + 1;
                    state <= s_axis_tlast ? S_HEADER1 : S_DROP;
                end else begin
                    // It's a valid UDP packet. Start forwarding.
                    stat_udp_rx <= stat_udp_rx + 1;
                    if (s_axis_tlast) begin
                        // Whole packet ended right here - emit the 4 UDP bytes
                        // we have as a tiny tlast=1 word.
                        m_axis_tdata  <= {32'h0, s_axis_tdata[63:32]};
                        m_axis_tkeep  <= 8'h0F;
                        m_axis_tlast  <= 1'b1;
                        m_axis_tvalid <= 1'b1;
                        state         <= S_HEADER1;
                    end else begin
                        state <= S_DATA;
                    end
                end
            end

            // -----------------------------------------------------------------
            // DATA: every cycle, output one word made of
            //   [4 fresh bytes from input] glued onto [4 residue bytes]
            // Then update residue with the top 4 bytes of the current input.
            // -----------------------------------------------------------------
            S_DATA: if (s_axis_tvalid) begin
                m_axis_tdata <= {s_axis_tdata[31:0], residue};
                residue      <= s_axis_tdata[63:32];

                if (s_axis_tlast) begin
                    in_bytes = count_keep(s_axis_tkeep);
                    if (in_bytes <= 4'd4) begin
                        // input + 4 residue bytes <= 8: fits this cycle.
                        m_axis_tkeep  <= keep_mask(in_bytes + 4'd4);
                        m_axis_tlast  <= 1'b1;
                        m_axis_tvalid <= 1'b1;
                        state         <= S_HEADER1;
                    end else begin
                        // Doesn't fit - need a flush cycle for the leftover.
                        m_axis_tkeep  <= 8'hFF;
                        m_axis_tlast  <= 1'b0;
                        m_axis_tvalid <= 1'b1;
                        flush_count   <= in_bytes - 4'd4;  // = in_bytes+4 - 8
                        state         <= S_FLUSH;
                    end
                end else begin
                    m_axis_tkeep  <= 8'hFF;
                    m_axis_tlast  <= 1'b0;
                    m_axis_tvalid <= 1'b1;
                end
            end

            // -----------------------------------------------------------------
            // FLUSH: push the trailing 1..4 bytes still living in the
            // residue register out the door with tlast=1.
            // -----------------------------------------------------------------
            S_FLUSH: begin
                m_axis_tdata  <= {32'h0, residue};
                m_axis_tkeep  <= keep_mask({1'b0, flush_count});
                m_axis_tlast  <= 1'b1;
                m_axis_tvalid <= 1'b1;
                state         <= S_HEADER1;
            end

            // -----------------------------------------------------------------
            // DROP: swallow the rest of this packet without forwarding.
            // -----------------------------------------------------------------
            S_DROP: if (s_axis_tvalid && s_axis_tlast) begin
                state <= S_HEADER1;
            end

            default: state <= S_HEADER1;
        endcase
    end
end

endmodule
