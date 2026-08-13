// eth_rx.v
// =============================================================================
// Second stage of the pipeline. Reads the Ethernet header off the front of
// an incoming frame, and either passes the rest of the packet onwards (if
// it's an IPv4 packet) or quietly drops the whole thing.
//
// The Ethernet header is 14 bytes:
//   bytes  0..5 : destination MAC address (who is this for?)
//   bytes  6..11: source MAC address      (who sent it?)
//   bytes 12..13: EtherType                (what's the next layer?)
//
// EtherType tells us what's inside:
//   0x0800 = IPv4   <- this is the one we care about
//   0x86DD = IPv6   <- drop
//   0x0806 = ARP    <- drop
//   anything else   <- drop
//
// =============================================================================
// THE TRICKY BIT: byte alignment
// =============================================================================
// The bus is 8 bytes wide. But the Ethernet header is 14 bytes - NOT a
// multiple of 8. So when the header finishes, the bus cycle isn't finished;
// there are still 2 bytes of IP data sitting in the high half of the cycle.
//
// Cycle 1 (bytes  0..7): all Ethernet header.
// Cycle 2 (bytes  8..15): bytes 8..13 = end of header, bytes 14..15 = start of IP.
//
// If we just throw those 2 IP bytes away, the IP parser sees the header
// shifted by 2 bytes and reads complete garbage. So we save them in a small
// register called "residue", and on the next cycle we glue them onto the
// front of the bytes coming in. That way the IP parser sees a fresh,
// nicely-aligned stream starting with byte 0 of the IP header.
//
// Think of it like cutting an envelope open: there's always a tiny strip of
// envelope you have to peel off before you get to the letter. The residue
// register is that strip.
// =============================================================================

`default_nettype none
`timescale 1ns / 1ps

module eth_rx (
    input  wire        clk,
    input  wire        rst,

    // Input: raw Ethernet frame bytes (from xgmii_rx)
    input  wire [63:0] s_axis_tdata,
    input  wire [7:0]  s_axis_tkeep,
    input  wire        s_axis_tlast,
    input  wire        s_axis_tvalid,
    output wire        s_axis_tready,

    // Output: IP packet bytes (header stripped, properly realigned)
    output reg  [63:0] m_axis_tdata,
    output reg  [7:0]  m_axis_tkeep,
    output reg         m_axis_tlast,
    output reg         m_axis_tvalid,
    input  wire        m_axis_tready,    // not used - we assume always ready

    // Sideband: parsed header fields (valid for one cycle after header arrives)
    output reg  [47:0] dst_mac,
    output reg  [47:0] src_mac,
    output reg  [15:0] ethertype,
    output reg         sideband_valid,

    // Stats
    output reg  [31:0] stat_ipv4_rx,
    output reg  [31:0] stat_dropped
);

// ---- States ----
localparam S_HEADER1 = 3'd0;  // first 8 frame bytes: DST MAC + start of SRC MAC
localparam S_HEADER2 = 3'd1;  // next 8 bytes: rest of SRC MAC + EtherType + 2 IP bytes
localparam S_DATA    = 3'd2;  // shift residue + new input out as realigned IP stream
localparam S_FLUSH   = 3'd3;  // empty out the last residue bytes after tlast
localparam S_DROP    = 3'd4;  // wrong EtherType - eat the rest of the packet

reg [2:0] state = S_HEADER1;

localparam ETHERTYPE_IPV4 = 16'h0800;

// We never push back - just always say yes.
assign s_axis_tready = 1'b1;

// Holders for header fields we collect across two cycles.
reg [47:0] dst_mac_r;
reg [47:0] src_mac_r;

// The 2 leftover IP bytes that arrive in cycle 2 - they need to lead off
// the realigned output stream on the next cycle.
reg [15:0] residue;

// During the very last cycle of a packet there may be leftover residue
// bytes that didn't fit in the final output word. We park them here and
// flush them out on the next clock.
reg [1:0] flush_count;  // 1 or 2 bytes to flush

// ---- Helper: how many bytes are real in this tkeep?  ----
// AXI-Stream uses contiguous keep from the low side, so 1111 means 4 bytes,
// 0111 means 3, etc.
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

// ---- Helper: build a tkeep with the low N bits set ----
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

// ---- Local temporary used in S_DATA - declared at module scope because in
// Verilog-2001 you can't put `reg` declarations inside begin/end blocks. ----
reg [3:0] in_bytes;

always @(posedge clk) begin
    if (rst) begin
        state          <= S_HEADER1;
        m_axis_tvalid  <= 1'b0;
        m_axis_tlast   <= 1'b0;
        m_axis_tkeep   <= 8'h00;
        m_axis_tdata   <= 64'h0;
        sideband_valid <= 1'b0;
        stat_ipv4_rx   <= 32'h0;
        stat_dropped   <= 32'h0;
        flush_count    <= 2'h0;
        residue        <= 16'h0;
    end else begin
        // Defaults: nothing on the output, no sideband.
        m_axis_tvalid  <= 1'b0;
        m_axis_tlast   <= 1'b0;
        sideband_valid <= 1'b0;

        case (state)

            // -----------------------------------------------------------------
            // HEADER1: bytes 0..7 of the frame.
            // = full destination MAC (6 bytes) + first 2 bytes of source MAC.
            // We just stash them for now - nothing comes out the other side yet.
            // -----------------------------------------------------------------
            S_HEADER1: if (s_axis_tvalid) begin
                // Bytes on the wire are big-endian: byte 0 is the most
                // significant byte of the MAC address. Pack them so dst_mac[47:40]
                // holds byte 0, dst_mac[7:0] holds byte 5.
                dst_mac_r <= {s_axis_tdata[ 7: 0],   // byte 0
                              s_axis_tdata[15: 8],   // byte 1
                              s_axis_tdata[23:16],   // byte 2
                              s_axis_tdata[31:24],   // byte 3
                              s_axis_tdata[39:32],   // byte 4
                              s_axis_tdata[47:40]};  // byte 5

                // First 2 source MAC bytes - rest comes next cycle.
                src_mac_r[47:32] <= {s_axis_tdata[55:48],   // byte 6
                                     s_axis_tdata[63:56]};  // byte 7

                if (s_axis_tlast) begin
                    // Whoops, the packet ended inside the header. Definitely junk.
                    stat_dropped <= stat_dropped + 1;
                    state        <= S_HEADER1;
                end else begin
                    state <= S_HEADER2;
                end
            end

            // -----------------------------------------------------------------
            // HEADER2: bytes 8..15 of the frame.
            // = rest of source MAC (4 bytes) + EtherType (2 bytes) + first
            //   2 bytes of IP header.
            // Save those 2 IP bytes into residue for the alignment shuffle.
            // -----------------------------------------------------------------
            S_HEADER2: if (s_axis_tvalid) begin
                // Rest of the source MAC: bytes 8..11
                src_mac_r[31:0] <= {s_axis_tdata[ 7: 0],
                                    s_axis_tdata[15: 8],
                                    s_axis_tdata[23:16],
                                    s_axis_tdata[31:24]};

                // EtherType lives in bytes 12..13 (big-endian on the wire).
                ethertype <= {s_axis_tdata[39:32], s_axis_tdata[47:40]};

                // Publish what we've learned on the sideband.
                // dst_mac was fully captured last cycle, so it's available now.
                dst_mac <= dst_mac_r;
                // src_mac needs the top 2 from last cycle + the 4 we just got.
                src_mac <= {src_mac_r[47:32],
                            s_axis_tdata[ 7: 0],
                            s_axis_tdata[15: 8],
                            s_axis_tdata[23:16],
                            s_axis_tdata[31:24]};
                sideband_valid <= 1'b1;

                // Save bytes 14..15: they are IP header bytes 0..1.
                // Put byte 14 into residue[7:0] so it ends up as the
                // FIRST byte of the output stream next cycle.
                residue <= {s_axis_tdata[63:56],   // byte 15 -> residue[15:8]
                            s_axis_tdata[55:48]};  // byte 14 -> residue[7:0]

                // Is this an IPv4 packet?  If yes, get ready to forward it.
                if ({s_axis_tdata[39:32], s_axis_tdata[47:40]} == ETHERTYPE_IPV4) begin
                    stat_ipv4_rx <= stat_ipv4_rx + 1;
                    if (s_axis_tlast) begin
                        // Packet ended right after the header - too short.
                        stat_dropped <= stat_dropped + 1;
                        state <= S_HEADER1;
                    end else begin
                        state <= S_DATA;
                    end
                end else begin
                    // Not IPv4 - drop.
                    stat_dropped <= stat_dropped + 1;
                    state <= s_axis_tlast ? S_HEADER1 : S_DROP;
                end
            end

            // -----------------------------------------------------------------
            // DATA: every cycle, produce one output word made of
            // [6 new bytes from input] glued onto [2 saved residue bytes].
            // Update the residue with the top 2 bytes of the current input,
            // ready for next cycle.
            // -----------------------------------------------------------------
            S_DATA: if (s_axis_tvalid) begin
                // The output word: residue in the low 2 bytes, then 6 fresh
                // bytes from the input. The result is the IP stream shifted
                // back into 8-byte alignment.
                m_axis_tdata <= {s_axis_tdata[47:0], residue};

                // Update the residue: the top 2 bytes of THIS input will
                // be the first 2 bytes of next cycle's output.
                residue <= s_axis_tdata[63:48];

                if (s_axis_tlast) begin
                    // End of packet. How many input bytes did we have?
                    in_bytes = count_keep(s_axis_tkeep);
                    if (in_bytes <= 4'd6) begin
                        // input bytes + 2 residue bytes all fit in this word.
                        m_axis_tkeep  <= keep_mask(in_bytes + 4'd2);
                        m_axis_tlast  <= 1'b1;
                        m_axis_tvalid <= 1'b1;
                        state         <= S_HEADER1;
                    end else begin
                        // Too many to fit - need one extra output cycle to flush.
                        m_axis_tkeep  <= 8'hFF;
                        m_axis_tlast  <= 1'b0;
                        m_axis_tvalid <= 1'b1;
                        flush_count   <= in_bytes - 4'd6;  // = in_bytes+2 - 8
                        state         <= S_FLUSH;
                    end
                end else begin
                    // Middle of a packet - 8 valid bytes out.
                    m_axis_tkeep  <= 8'hFF;
                    m_axis_tlast  <= 1'b0;
                    m_axis_tvalid <= 1'b1;
                end
            end

            // -----------------------------------------------------------------
            // FLUSH: only reached when the input packet ended with so many
            // bytes that residue + input was more than one output word's
            // worth. We push the last 1 or 2 bytes out with tlast=1.
            // -----------------------------------------------------------------
            S_FLUSH: begin
                m_axis_tdata  <= {48'h0, residue};
                m_axis_tkeep  <= keep_mask({2'b0, flush_count});
                m_axis_tlast  <= 1'b1;
                m_axis_tvalid <= 1'b1;
                state         <= S_HEADER1;
            end

            // -----------------------------------------------------------------
            // DROP: this isn't IPv4. Eat every cycle quietly until tlast.
            // -----------------------------------------------------------------
            S_DROP: if (s_axis_tvalid && s_axis_tlast) begin
                state <= S_HEADER1;
            end

            default: state <= S_HEADER1;
        endcase
    end
end

endmodule
