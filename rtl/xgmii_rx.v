// xgmii_rx.v

// First stage of the pipeline. Takes raw XGMII signals coming off the
// 10G Ethernet transceiver and turns them into a clean stream of frame bytes
// for the next stage to chew on.
//
// What is XGMII?
//   - 64 wires of data + 8 wires of "control" status, sampled every clock.
//   - At 156.25 MHz that's 8 bytes per cycle = 10 Gbit/s.
//   - Each of the 8 bytes can either be a real data byte or a special
//     "control character" telling us about the packet boundary.
//
// The three control characters we care about:
//   0xFB  START - a packet is just starting
//   0xFD  TERM  - a packet has just ended
//   0x07  IDLE  - nothing is being sent right now
//
// The job here is small:
//   1. Sit and wait until we see a START in lane 0.
//   2. Skip the rest of that cycle (it's just preamble bytes that help the
//      receiver lock onto the signal - useless to us).
//   3. From the next cycle onwards, copy data bytes onto an AXI-Stream bus.
//   4. When we see a TERM, mark the previous valid byte as the last byte.
//
// Byte order convention (this trips everyone up at least once):
//   lane 0 = first byte in time = xgmii_rxd[ 7:0]
//   lane 1                       = xgmii_rxd[15:8]
//   ...
//   lane 7 = last byte in cycle  = xgmii_rxd[63:56]


`default_nettype none
`timescale 1ns / 1ps

module xgmii_rx (
    input  wire        clk,
    input  wire        rst,

    // Raw XGMII signals coming from the PHY (or, in our case, the testbench)
    input  wire [63:0] xgmii_rxd,    // 8 data bytes packed into 64 bits
    input  wire [7:0]  xgmii_rxc,    // 1 = control char, 0 = real data byte

    // AXI-Stream output: pure frame bytes, preamble already stripped off
    output reg  [63:0] m_axis_tdata,
    output reg  [7:0]  m_axis_tkeep, // which of the 8 bytes are real
    output reg         m_axis_tlast, // 1 on the final cycle of the frame
    output reg         m_axis_tvalid,// 1 means this cycle has real bytes

    // Just some counters to help with debugging
    output reg  [31:0] stat_frames_rx,
    output reg  [31:0] stat_errors
);

// ---- Special XGMII byte values ----
localparam XGMII_START = 8'hFB;
localparam XGMII_TERM  = 8'hFD;

// ---- States: we only really have two, "waiting" and "streaming" ----
localparam S_IDLE = 1'b0;
localparam S_DATA = 1'b1;
reg state = S_IDLE;

// "Did lane 0 just receive a START character?"
// Lane 0 is bits [7:0] of the bus, and xgmii_rxc[0] tells us if it's a control byte.
wire start_detected = xgmii_rxc[0] && (xgmii_rxd[7:0] == XGMII_START);

// ---- Look for TERM in any of the 8 lanes ----
// term_lane is a one-hot mask: bit N is 1 if lane N holds a TERM character.
wire [7:0] term_lane;
genvar i;
generate
    for (i = 0; i < 8; i = i + 1) begin : gen_term
        assign term_lane[i] = xgmii_rxc[i] && (xgmii_rxd[i*8 +: 8] == XGMII_TERM);
    end
endgenerate

wire term_detected = |term_lane;

// Which lane number holds the TERM? Find the lowest set bit in term_lane.
// Anything BEFORE this lane is real data; the TERM itself and everything after
// is junk. So if TERM is in lane 4, bytes 0-3 of this cycle are good.
reg [2:0] term_index;
always @(*) begin
    casez (term_lane)
        8'b???????1: term_index = 3'd0;
        8'b??????1?: term_index = 3'd1;
        8'b?????1??: term_index = 3'd2;
        8'b????1???: term_index = 3'd3;
        8'b???1????: term_index = 3'd4;
        8'b??1?????: term_index = 3'd5;
        8'b?1??????: term_index = 3'd6;
        8'b1???????: term_index = 3'd7;
        default:     term_index = 3'd0;
    endcase
end

// Turn term_index into a tkeep mask. If TERM is in lane 3 then bytes 0,1,2
// were the last real data bytes - so tkeep = 0000_0111.
//   term_index 0 -> 0000_0000  (no good bytes this cycle, edge case)
//   term_index 1 -> 0000_0001
//   term_index 4 -> 0000_1111
//   term_index 7 -> 0111_1111
wire [7:0] term_keep = (term_index == 0) ? 8'h00 : (8'hFF >> (8 - term_index));

// =============================================================================
// State machine - dead simple: idle until START, stream until TERM.
// =============================================================================
always @(posedge clk) begin
    if (rst) begin
        state          <= S_IDLE;
        m_axis_tvalid  <= 1'b0;
        m_axis_tlast   <= 1'b0;
        m_axis_tkeep   <= 8'h00;
        m_axis_tdata   <= 64'h0;
        stat_frames_rx <= 32'h0;
        stat_errors    <= 32'h0;
    end else begin
        // Default each cycle: not outputting anything.
        // The case below will override these when a real cycle comes in.
        m_axis_tvalid <= 1'b0;
        m_axis_tlast  <= 1'b0;

        case (state)

            // ---------------------------------------------------------------
            // IDLE: just watching the wire. Until we see START, do nothing.
            // The START character lives in the same cycle as the preamble,
            // so on the NEXT cycle the first real frame byte will arrive.
            // ---------------------------------------------------------------
            S_IDLE: begin
                if (start_detected)
                    state <= S_DATA;
            end

            // ---------------------------------------------------------------
            // DATA: every cycle, push the 8 bytes onto the AXI-Stream bus.
            // Keep doing this until we see TERM, then mark that cycle as
            // the last and go back to IDLE.
            // ---------------------------------------------------------------
            S_DATA: begin
                m_axis_tdata  <= xgmii_rxd;
                m_axis_tvalid <= 1'b1;

                if (term_detected) begin
                    // End of the frame this cycle.
                    m_axis_tkeep   <= term_keep;
                    m_axis_tlast   <= 1'b1;
                    stat_frames_rx <= stat_frames_rx + 1;
                    state          <= S_IDLE;
                end else begin
                    // Middle of frame - every byte is good.
                    m_axis_tkeep <= 8'hFF;
                end
            end

            default: state <= S_IDLE;
        endcase
    end
end

endmodule
