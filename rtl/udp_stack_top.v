// udp_stack_top.v
// =============================================================================
// Glue module. Just wires the four pipeline stages together and exposes
// the inputs / outputs you'd connect to in a real design.
//
// Data flow:
//
//     XGMII  -->  xgmii_rx  -->  eth_rx  -->  ipv4_rx  -->  udp_rx  -->  payload
//
// Each arrow between stages is an AXI-Stream bus (tdata + tkeep + tlast +
// tvalid + tready). Header fields parsed along the way come out as
// "sideband" outputs at the top level so downstream logic can see who sent
// what, on which port, etc.
//
// Stat counters from every stage are exposed for debugging.
// =============================================================================

`default_nettype none
`timescale 1ns / 1ps

module udp_stack_top (
    input  wire        clk,
    input  wire        rst,

    // ---- XGMII input from the PHY / transceiver ----
    input  wire [63:0] xgmii_rxd,
    input  wire [7:0]  xgmii_rxc,

    // ---- Final payload output (AXI-Stream of raw UDP payload bytes) ----
    output wire [63:0] payload_tdata,
    output wire [7:0]  payload_tkeep,
    output wire        payload_tlast,
    output wire        payload_tvalid,
    input  wire        payload_tready,

    // ---- Parsed header fields (sideband) ----
    output wire [47:0] eth_dst_mac,
    output wire [47:0] eth_src_mac,
    output wire [31:0] ip_src,
    output wire [31:0] ip_dst,
    output wire [15:0] udp_src_port,
    output wire [15:0] udp_dst_port,
    output wire [15:0] udp_length,

    // ---- Stats from each stage ----
    output wire [31:0] stat_eth_ipv4,
    output wire [31:0] stat_eth_dropped,
    output wire [31:0] stat_ip_udp,
    output wire [31:0] stat_ip_dropped_proto,
    output wire [31:0] stat_ip_dropped_cksum,
    output wire [31:0] stat_udp_rx
);

// ---- Internal AXI-Stream wires between stages ----

// xgmii_rx -> eth_rx
wire [63:0] x2e_tdata;
wire [7:0]  x2e_tkeep;
wire        x2e_tlast;
wire        x2e_tvalid;
wire        x2e_tready;

// eth_rx -> ipv4_rx
wire [63:0] e2i_tdata;
wire [7:0]  e2i_tkeep;
wire        e2i_tlast;
wire        e2i_tvalid;
wire        e2i_tready;

// ipv4_rx -> udp_rx
wire [63:0] i2u_tdata;
wire [7:0]  i2u_tkeep;
wire        i2u_tlast;
wire        i2u_tvalid;
wire        i2u_tready;

// xgmii_rx doesn't accept backpressure - drive its (unused) ready high.
assign x2e_tready = 1'b1;

// Sideband wires we don't expose at the top level.
wire        eth_sideband_valid;
wire        ip_sideband_valid;
wire        udp_sideband_valid;
wire [7:0]  ip_protocol;
wire [63:0] udp_payload_start_time;
wire [31:0] xgmii_frames;
wire [31:0] xgmii_errors;

// =============================================================================
// Stage 1: XGMII receiver - strips preamble, gives clean frame bytes.
// =============================================================================
xgmii_rx u_xgmii_rx (
    .clk            (clk),
    .rst            (rst),
    .xgmii_rxd      (xgmii_rxd),
    .xgmii_rxc      (xgmii_rxc),
    .m_axis_tdata   (x2e_tdata),
    .m_axis_tkeep   (x2e_tkeep),
    .m_axis_tlast   (x2e_tlast),
    .m_axis_tvalid  (x2e_tvalid),
    .stat_frames_rx (xgmii_frames),
    .stat_errors    (xgmii_errors)
);

// =============================================================================
// Stage 2: Ethernet header parser - keeps IPv4, drops the rest.
// =============================================================================
eth_rx u_eth_rx (
    .clk            (clk),
    .rst            (rst),
    .s_axis_tdata   (x2e_tdata),
    .s_axis_tkeep   (x2e_tkeep),
    .s_axis_tlast   (x2e_tlast),
    .s_axis_tvalid  (x2e_tvalid),
    .s_axis_tready  (x2e_tready),
    .m_axis_tdata   (e2i_tdata),
    .m_axis_tkeep   (e2i_tkeep),
    .m_axis_tlast   (e2i_tlast),
    .m_axis_tvalid  (e2i_tvalid),
    .m_axis_tready  (e2i_tready),
    .dst_mac        (eth_dst_mac),
    .src_mac        (eth_src_mac),
    .ethertype      (),
    .sideband_valid (eth_sideband_valid),
    .stat_ipv4_rx   (stat_eth_ipv4),
    .stat_dropped   (stat_eth_dropped)
);

// =============================================================================
// Stage 3: IPv4 header parser - validates checksum, keeps UDP only.
// =============================================================================
ipv4_rx u_ipv4_rx (
    .clk                 (clk),
    .rst                 (rst),
    .s_axis_tdata        (e2i_tdata),
    .s_axis_tkeep        (e2i_tkeep),
    .s_axis_tlast        (e2i_tlast),
    .s_axis_tvalid       (e2i_tvalid),
    .s_axis_tready       (e2i_tready),
    .m_axis_tdata        (i2u_tdata),
    .m_axis_tkeep        (i2u_tkeep),
    .m_axis_tlast        (i2u_tlast),
    .m_axis_tvalid       (i2u_tvalid),
    .m_axis_tready       (i2u_tready),
    .src_ip              (ip_src),
    .dst_ip              (ip_dst),
    .total_length        (),
    .protocol            (ip_protocol),
    .sideband_valid      (ip_sideband_valid),
    .stat_udp_rx         (stat_ip_udp),
    .stat_dropped_proto  (stat_ip_dropped_proto),
    .stat_dropped_cksum  (stat_ip_dropped_cksum)
);

// =============================================================================
// Stage 4: UDP header parser - exposes ports + length, passes payload through.
// =============================================================================
udp_rx u_udp_rx (
    .clk                (clk),
    .rst                (rst),
    .s_axis_tdata       (i2u_tdata),
    .s_axis_tkeep       (i2u_tkeep),
    .s_axis_tlast       (i2u_tlast),
    .s_axis_tvalid      (i2u_tvalid),
    .s_axis_tready      (i2u_tready),
    .m_axis_tdata       (payload_tdata),
    .m_axis_tkeep       (payload_tkeep),
    .m_axis_tlast       (payload_tlast),
    .m_axis_tvalid      (payload_tvalid),
    .m_axis_tready      (payload_tready),
    .src_port           (udp_src_port),
    .dst_port           (udp_dst_port),
    .udp_length         (udp_length),
    .sideband_valid     (udp_sideband_valid),
    .payload_start_time (udp_payload_start_time),
    .stat_rx            (stat_udp_rx)
);

endmodule
