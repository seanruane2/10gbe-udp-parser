// udp_rx.v
// =============================================================================
// Last stage of the pipeline. The UDP header is exactly 8 bytes - which
// happens to be exactly one bus cycle on our 64-bit datapath. So this is
// the easiest module of the lot. No byte shuffling, no fancy alignment.
//
// UDP header (8 bytes):
//   bytes 0..1 : source port       (who is sending)
//   bytes 2..3 : destination port  (which "channel" this stream is)
//   bytes 4..5 : UDP length        (header + payload in bytes)
//   bytes 6..7 : checksum          (optional in IPv4 - we don't validate)
//
// After byte 7: pure application payload. For an HFT feed like Nasdaq's
// ITCH protocol, those bytes carry market data messages.
//
// All multi-byte fields are big-endian on the wire.
//
// Latency note: the payload data path is *combinational* in DATA state.
// Once we've consumed the UDP header cycle, every payload word entering
// our input shows up at our output in the SAME cycle, not the next one.
// This saves one cycle of end-to-end parse latency vs. a fully-registered
// pass-through. The combinational path it adds is one mux/AND gated on
// the state bit, which is easy slack at 156.25 MHz.
// =============================================================================

`default_nettype none
`timescale 1ns / 1ps

module udp_rx (
    input  wire        clk,
    input  wire        rst,

    // Input: UDP segment bytes (from ipv4_rx) - first cycle is the UDP header.
    input  wire [63:0] s_axis_tdata,
    input  wire [7:0]  s_axis_tkeep,
    input  wire        s_axis_tlast,
    input  wire        s_axis_tvalid,
    output wire        s_axis_tready,

    // Output: raw application payload.
    output wire [63:0] m_axis_tdata,
    output wire [7:0]  m_axis_tkeep,
    output wire        m_axis_tlast,
    output wire        m_axis_tvalid,
    input  wire        m_axis_tready,   // not used

    // Sideband: parsed UDP header (valid one cycle after the header arrives).
    output reg  [15:0] src_port,
    output reg  [15:0] dst_port,
    output reg  [15:0] udp_length,
    output reg         sideband_valid,

    // Latency probe: cycle number when the first payload word goes out.
    output reg  [63:0] payload_start_time,

    // Stats: number of payload words forwarded (handy for sanity-checking).
    output reg  [31:0] stat_rx
);

localparam S_HEADER = 1'b0;
localparam S_DATA   = 1'b1;
reg state = S_HEADER;

assign s_axis_tready = 1'b1;

// ---- Combinational payload pass-through ----
// In DATA state, the payload bytes don't need any transformation, so we
// forward them straight out of the module on the same cycle they arrive.
// Gating on `state == S_DATA` keeps the UDP-header cycle from leaking out.
wire in_data = (state == S_DATA) && s_axis_tvalid;
assign m_axis_tdata  = s_axis_tdata;
assign m_axis_tkeep  = s_axis_tkeep;
assign m_axis_tlast  = s_axis_tlast & in_data;
assign m_axis_tvalid = in_data;

// Free-running cycle counter so we can timestamp the first payload word.
reg [63:0] cycle_counter;
reg        first_payload_seen;

always @(posedge clk) begin
    if (rst)
        cycle_counter <= 64'h0;
    else
        cycle_counter <= cycle_counter + 1;
end

always @(posedge clk) begin
    if (rst) begin
        state              <= S_HEADER;
        sideband_valid     <= 1'b0;
        stat_rx            <= 32'h0;
        first_payload_seen <= 1'b0;
        payload_start_time <= 64'h0;
    end else begin
        sideband_valid <= 1'b0;

        if (s_axis_tvalid) begin
            case (state)

                // -------------------------------------------------------------
                // HEADER: the whole UDP header arrives in this one cycle.
                // Pick out the three fields we care about and announce them
                // on the sideband. The rest of the packet is payload.
                //
                // Each 16-bit field is high-byte-first on the wire, so we
                // reconstruct it as {HIGH, LOW}.
                // -------------------------------------------------------------
                S_HEADER: begin
                    src_port   <= {s_axis_tdata[ 7: 0], s_axis_tdata[15: 8]};
                    dst_port   <= {s_axis_tdata[23:16], s_axis_tdata[31:24]};
                    udp_length <= {s_axis_tdata[39:32], s_axis_tdata[47:40]};
                    // Bytes 6..7 are the UDP checksum - in IPv4 it's optional
                    // and most senders leave it 0. We don't validate.

                    sideband_valid <= 1'b1;

                    // If the packet ended right after the header (no payload),
                    // just go back to waiting. Otherwise stream the payload.
                    state <= s_axis_tlast ? S_HEADER : S_DATA;
                end

                // -------------------------------------------------------------
                // DATA: the combinational assigns above are already pushing
                // payload bytes onto the output. Here we just bookkeep:
                // count words, timestamp the first one, and watch for tlast
                // so we know when the packet ends.
                // -------------------------------------------------------------
                S_DATA: begin
                    stat_rx <= stat_rx + 1;

                    if (!first_payload_seen) begin
                        payload_start_time <= cycle_counter;
                        first_payload_seen <= 1'b1;
                    end

                    if (s_axis_tlast)
                        state <= S_HEADER;
                end

                default: state <= S_HEADER;
            endcase
        end
    end
end

endmodule
