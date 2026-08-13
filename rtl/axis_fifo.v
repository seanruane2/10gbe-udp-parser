// axis_fifo.v
// A small synchronous AXI-Stream FIFO.
//
// WHAT IT'S FOR:
// You can't backpressure udp_rx_parser - the wire doesn't stop, so there's
// nowhere for the data to go back to. If the thing consuming the payload can
// stall (a slow decoder, or a DMA engine waiting on a bus grant), drop one of
// these in between to soak up the burst. It's deliberately NOT in the parser's
// latency path by default, and it shouldn't be, because every entry costs you a
// cycle.
//
// HOW IT WORKS:
// It's a circular buffer with a write pointer and a read pointer, and each one
// carries an extra bit above the address so you can tell full and empty apart:
//   empty : the two pointers are identical
//   full  : the addresses match but the wrap bits are different
//
// DEPTH has to be a power of two, because the pointer wrap arithmetic relies on
// it. There's an elaboration-time check below to catch that.
//
// SIGNALS:
// tdata  the bytes (64 bits = 8 bytes per beat)
// tkeep  which of those bytes are real
// tlast  final beat of the packet
// tuser  comes along with tlast, meaning "this packet is damaged, bin it"
// tvalid the sender has real data this cycle
// tready the receiver can accept data this cycle
// A beat only moves when tvalid and tready are both high.

`default_nettype none
`timescale 1ns / 1ps

module axis_fifo #(
    parameter DATA_WIDTH = 64,
    parameter KEEP_WIDTH = DATA_WIDTH/8,
    parameter DEPTH      = 16            // must be a power of two
)(
    input  wire                   clk,
    input  wire                   rst,

    // Input side
    input  wire [DATA_WIDTH-1:0]  s_axis_tdata,
    input  wire [KEEP_WIDTH-1:0]  s_axis_tkeep,
    input  wire                   s_axis_tlast,
    input  wire                   s_axis_tuser,
    input  wire                   s_axis_tvalid,
    output wire                   s_axis_tready,

    // Output side
    output reg  [DATA_WIDTH-1:0]  m_axis_tdata,
    output reg  [KEEP_WIDTH-1:0]  m_axis_tkeep,
    output reg                    m_axis_tlast,
    output reg                    m_axis_tuser,
    output reg                    m_axis_tvalid,
    input  wire                   m_axis_tready,

    // Goes high for one cycle any time a beat gets offered while we're full.
    // Dropping a beat corrupts the packet, so in a healthy system this should
    // never fire - wire it up to a counter and keep an eye on it.
    output reg                    overflow
);

localparam PTR_BITS = $clog2(DEPTH);

// synthesis translate_off
initial begin
    if ((DEPTH & (DEPTH-1)) != 0) begin
        $display("ERROR: axis_fifo DEPTH=%0d is not a power of two", DEPTH);
        $finish;
    end
end
// synthesis translate_on

reg [DATA_WIDTH-1:0]  mem_data [0:DEPTH-1];
reg [KEEP_WIDTH-1:0]  mem_keep [0:DEPTH-1];
reg                   mem_last [0:DEPTH-1];
reg                   mem_user [0:DEPTH-1];

reg [PTR_BITS:0] wr_ptr = 0;
reg [PTR_BITS:0] rd_ptr = 0;

wire full  = (wr_ptr[PTR_BITS] != rd_ptr[PTR_BITS]) &&
             (wr_ptr[PTR_BITS-1:0] == rd_ptr[PTR_BITS-1:0]);
wire empty = (wr_ptr == rd_ptr);

assign s_axis_tready = !full;

wire do_write = s_axis_tvalid && s_axis_tready;

// -----------------------------------------------------------------------
// One always block owns both pointers and the output register.
//
// My first version of this incremented wr_ptr in one always block and reset it
// in a different one. That's two procedural blocks driving the same reg, i.e. a
// multiply-driven net, which races in simulation and gets rejected outright by
// synthesis. It's also why I'd never actually instantiated this module anywhere
// - it couldn't have been built in the first place.
// -----------------------------------------------------------------------
always @(posedge clk) begin
    if (rst) begin
        wr_ptr        <= 0;
        rd_ptr        <= 0;
        m_axis_tvalid <= 1'b0;
        m_axis_tlast  <= 1'b0;
        m_axis_tuser  <= 1'b0;
        m_axis_tkeep  <= {KEEP_WIDTH{1'b0}};
        m_axis_tdata  <= {DATA_WIDTH{1'b0}};
        overflow      <= 1'b0;
    end else begin
        overflow <= s_axis_tvalid && full;

        if (do_write)
            wr_ptr <= wr_ptr + 1'b1;

        // Load the output register when it's free (or is getting drained this
        // cycle anyway) and there's actually something waiting to load.
        if (!empty && (!m_axis_tvalid || m_axis_tready)) begin
            m_axis_tdata  <= mem_data[rd_ptr[PTR_BITS-1:0]];
            m_axis_tkeep  <= mem_keep[rd_ptr[PTR_BITS-1:0]];
            m_axis_tlast  <= mem_last[rd_ptr[PTR_BITS-1:0]];
            m_axis_tuser  <= mem_user[rd_ptr[PTR_BITS-1:0]];
            m_axis_tvalid <= 1'b1;
            rd_ptr        <= rd_ptr + 1'b1;
        end else if (m_axis_tready) begin
            // Nothing left to load and the receiver took what we had.
            // (If tready is low we just hold the beat - that's the handshake.)
            m_axis_tvalid <= 1'b0;
        end
    end
end

// I keep the memory writes out of the reset branch so the array still gets
// inferred as distributed/block RAM instead of a big pile of resettable flops.
always @(posedge clk) begin
    if (do_write) begin
        mem_data[wr_ptr[PTR_BITS-1:0]] <= s_axis_tdata;
        mem_keep[wr_ptr[PTR_BITS-1:0]] <= s_axis_tkeep;
        mem_last[wr_ptr[PTR_BITS-1:0]] <= s_axis_tlast;
        mem_user[wr_ptr[PTR_BITS-1:0]] <= s_axis_tuser;
    end
end

endmodule

`default_nettype wire
