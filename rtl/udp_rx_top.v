// udp_rx_top.v
// =============================================================================
// Top level of the low-latency receive path.
//
//     XGMII  -->  udp_rx_parser  -->  UDP payload (AXI-Stream)
//
// There's nothing in between the PHY and the parser, and nothing between the
// parser and the payload port, and that's on purpose. Every module boundary you
// stick on this path costs you a cycle, and the whole idea of the design is that
// there's only ever one register sitting between the wire and the output.
//
// If whatever consumes the payload can apply backpressure, hang an axis_fifo off
// the payload port OUTSIDE this module. I haven't put one in here because it
// would add latency to a path that doesn't need it - and if the consumer really
// can't keep up with a 10 Gbit/s wire, stat_backpressure will let you know.
//
// This replaces udp_stack_top.v, which wires up my original four-stage
// xgmii_rx / eth_rx / ipv4_rx / udp_rx chain. That chain still builds and still
// works - I've kept it around for the latency comparison in the README (9 cycles
// against the 6 here) - but this is the one to build on.
// =============================================================================

`default_nettype none
`timescale 1ns / 1ps

module udp_rx_top #(
    parameter OUTPUT_REG = 0    // 0 = 6-cycle latency, 1 = 7-cycle. See parser.
)(
    input  wire        clk,
    input  wire        rst,

    // ---- XGMII from the PHY / transceiver ----
    input  wire [63:0] xgmii_rxd,
    input  wire [7:0]  xgmii_rxc,

    // ---- Feed selection: only accept one UDP destination port ----
    input  wire        cfg_port_filter_en,
    input  wire [15:0] cfg_dst_port,

    // ---- UDP payload out ----
    output wire [63:0] payload_tdata,
    output wire [7:0]  payload_tkeep,
    output wire        payload_tlast,
    output wire        payload_tuser,   // high with tlast = packet's damaged, bin it
    output wire        payload_tvalid,
    input  wire        payload_tready,

    // ---- Parsed headers. These hold from hdr_valid until the next packet. ----
    output wire [47:0] eth_dst_mac,
    output wire [47:0] eth_src_mac,
    output wire [15:0] eth_type,
    output wire [31:0] ip_src,
    output wire [31:0] ip_dst,
    output wire [15:0] ip_total_length,
    output wire [15:0] udp_src_port,
    output wire [15:0] udp_dst_port,
    output wire [15:0] udp_length,
    output wire        hdr_valid,

    // ---- Counters ----
    output wire [31:0] stat_frames,
    output wire [31:0] stat_accepted,
    output wire [31:0] stat_drop_ethertype,
    output wire [31:0] stat_drop_vlan,
    output wire [31:0] stat_drop_ihl,
    output wire [31:0] stat_drop_frag,
    output wire [31:0] stat_drop_proto,
    output wire [31:0] stat_drop_cksum,
    output wire [31:0] stat_drop_len,
    output wire [31:0] stat_drop_port,
    output wire [31:0] stat_runt,
    output wire [31:0] stat_abort,
    output wire [31:0] stat_xgmii_err,
    output wire [31:0] stat_backpressure
);

udp_rx_parser #(
    .OUTPUT_REG          (OUTPUT_REG)
) u_parser (
    .clk                 (clk),
    .rst                 (rst),

    .xgmii_rxd           (xgmii_rxd),
    .xgmii_rxc           (xgmii_rxc),

    .cfg_port_filter_en  (cfg_port_filter_en),
    .cfg_dst_port        (cfg_dst_port),

    .m_axis_tdata        (payload_tdata),
    .m_axis_tkeep        (payload_tkeep),
    .m_axis_tlast        (payload_tlast),
    .m_axis_tuser        (payload_tuser),
    .m_axis_tvalid       (payload_tvalid),
    .m_axis_tready       (payload_tready),

    .dst_mac             (eth_dst_mac),
    .src_mac             (eth_src_mac),
    .ethertype           (eth_type),
    .src_ip              (ip_src),
    .dst_ip              (ip_dst),
    .total_length        (ip_total_length),
    .src_port            (udp_src_port),
    .dst_port            (udp_dst_port),
    .udp_length          (udp_length),
    .hdr_valid           (hdr_valid),

    .stat_frames         (stat_frames),
    .stat_accepted       (stat_accepted),
    .stat_drop_ethertype (stat_drop_ethertype),
    .stat_drop_vlan      (stat_drop_vlan),
    .stat_drop_ihl       (stat_drop_ihl),
    .stat_drop_frag      (stat_drop_frag),
    .stat_drop_proto     (stat_drop_proto),
    .stat_drop_cksum     (stat_drop_cksum),
    .stat_drop_len       (stat_drop_len),
    .stat_drop_port      (stat_drop_port),
    .stat_runt           (stat_runt),
    .stat_abort          (stat_abort),
    .stat_xgmii_err      (stat_xgmii_err),
    .stat_backpressure   (stat_backpressure)
);

endmodule

`default_nettype wire
