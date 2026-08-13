// tb_udp_stack.v
// =============================================================================
// Testbench for the whole UDP receive stack.
//
// What it does, step by step:
//   1. Generates a 156.25 MHz clock and holds reset for a few cycles.
//   2. Hand-builds a complete Ethernet / IPv4 / UDP frame in a byte array.
//   3. Wraps it in XGMII (preamble, START, TERM, idles) and drives it onto
//      the DUT's input one 8-byte cycle at a time.
//   4. Watches for payload to start coming out the back of the pipeline.
//   5. Prints what arrived and how long it took.
//
// How to run in Vivado:
//   - Add all .v files under rtl/ as design sources.
//   - Add this file as a simulation source.
//   - Make sure tb_udp_stack is the simulation top.
//   - Run Behavioral Simulation - the Tcl console prints the results.
// =============================================================================

`timescale 1ns / 1ps
`default_nettype none

module tb_udp_stack;

// ----- Clock: 156.25 MHz = 6.4 ns period -----
reg clk = 0;
always #3.2 clk = ~clk;

reg rst = 1;

// ----- XGMII input signals we drive -----
reg [63:0] xgmii_rxd = 64'h0707070707070707; // start out in IDLE
reg [7:0]  xgmii_rxc = 8'hFF;

// ----- DUT outputs we watch -----
wire [63:0] payload_tdata;
wire [7:0]  payload_tkeep;
wire        payload_tlast;
wire        payload_tvalid;
wire [15:0] udp_dst_port_out;
wire [15:0] udp_src_port_out;
wire [31:0] ip_src_out;
wire [31:0] ip_dst_out;

// ----- Device under test -----
udp_stack_top dut (
    .clk                   (clk),
    .rst                   (rst),
    .xgmii_rxd             (xgmii_rxd),
    .xgmii_rxc             (xgmii_rxc),
    .payload_tdata         (payload_tdata),
    .payload_tkeep         (payload_tkeep),
    .payload_tlast         (payload_tlast),
    .payload_tvalid        (payload_tvalid),
    .payload_tready        (1'b1),
    .eth_dst_mac           (),
    .eth_src_mac           (),
    .ip_src                (ip_src_out),
    .ip_dst                (ip_dst_out),
    .udp_src_port          (udp_src_port_out),
    .udp_dst_port          (udp_dst_port_out),
    .udp_length            (),
    .stat_eth_ipv4         (),
    .stat_eth_dropped      (),
    .stat_ip_udp           (),
    .stat_ip_dropped_proto (),
    .stat_ip_dropped_cksum (),
    .stat_udp_rx           ()
);

// =============================================================================
// Cycle counter + latency probes
// =============================================================================
integer cycle_count         = 0;
integer first_xgmii_cycle   = 0;
integer first_payload_cycle = 0;

always @(posedge clk)
    cycle_count <= cycle_count + 1;

always @(posedge clk) begin
    if (payload_tvalid && (first_payload_cycle == 0))
        first_payload_cycle = cycle_count;
end

// =============================================================================
// The packet we send.
// pkt[] holds the raw bytes - byte 0 is the first byte on the wire
// (the highest-order byte of the destination MAC).
// =============================================================================
reg [7:0] pkt [0:63];
integer   pkt_len;

// Compute the IPv4 header checksum directly from pkt[14..33] and store it
// back into pkt[24..25]. Same algorithm the real network does:
//   sum all ten 16-bit words, fold the carries back in, then flip every bit.
task compute_ip_checksum;
    integer k;
    reg [31:0] acc;
    begin
        acc = 0;
        for (k = 14; k < 34; k = k + 2)
            acc = acc + {pkt[k], pkt[k+1]};
        acc = acc[31:16] + acc[15:0];   // fold any carry once
        acc = acc[31:16] + acc[15:0];   // and again, in case the fold itself carried
        pkt[24] = ~acc[15:8];
        pkt[25] = ~acc[7:0];
    end
endtask

// Fill pkt[] with a hand-crafted Ethernet / IPv4 / UDP frame.
task build_packet;
    begin
        // ---- Ethernet header (bytes 0..13) ----
        pkt[ 0] = 8'hAA; pkt[ 1] = 8'hBB; pkt[ 2] = 8'hCC;   // dst MAC
        pkt[ 3] = 8'hDD; pkt[ 4] = 8'hEE; pkt[ 5] = 8'hFF;
        pkt[ 6] = 8'h11; pkt[ 7] = 8'h22; pkt[ 8] = 8'h33;   // src MAC
        pkt[ 9] = 8'h44; pkt[10] = 8'h55; pkt[11] = 8'h66;
        pkt[12] = 8'h08; pkt[13] = 8'h00;                     // EtherType = IPv4

        // ---- IPv4 header (bytes 14..33) ----
        pkt[14] = 8'h45; pkt[15] = 8'h00;            // Version=4, IHL=5
        pkt[16] = 8'h00; pkt[17] = 8'h26;            // total length = 38
        pkt[18] = 8'h00; pkt[19] = 8'h00;            // identification
        pkt[20] = 8'h40; pkt[21] = 8'h00;            // flags = DF, frag offset = 0
        pkt[22] = 8'h40; pkt[23] = 8'h11;            // TTL = 64, Protocol = 17 (UDP)
        pkt[24] = 8'h00; pkt[25] = 8'h00;            // header checksum (computed below)
        pkt[26] = 8'hC0; pkt[27] = 8'hA8;            // src IP = 192.168.1.100
        pkt[28] = 8'h01; pkt[29] = 8'h64;
        pkt[30] = 8'hEF; pkt[31] = 8'hFF;            // dst IP = 239.255.43.1 (multicast)
        pkt[32] = 8'h2B; pkt[33] = 8'h01;

        compute_ip_checksum;

        // ---- UDP header (bytes 34..41) ----
        pkt[34] = 8'h30; pkt[35] = 8'h39;            // src port  = 12345
        pkt[36] = 8'h67; pkt[37] = 8'h6D;            // dst port  = 26477 (ITCH)
        pkt[38] = 8'h00; pkt[39] = 8'h12;            // length    = 18
        pkt[40] = 8'h00; pkt[41] = 8'h00;            // checksum  = 0 (optional)

        // ---- Application payload (10 bytes): "HELLO_HFT!" ----
        pkt[42] = "H"; pkt[43] = "E"; pkt[44] = "L"; pkt[45] = "L";
        pkt[46] = "O"; pkt[47] = "_"; pkt[48] = "H"; pkt[49] = "F";
        pkt[50] = "T"; pkt[51] = "!";

        pkt_len = 52;
    end
endtask

// Drive pkt[] onto the XGMII interface, wrapped in the standard preamble,
// START, and TERM characters. Idle codes (0x07) fill any unused lanes.
task drive_packet;
    integer b;
    integer rem;
    integer lane;
    integer term_lane;
    reg [63:0] w;
    reg [7:0]  c;
    begin
        // Two idle cycles, just to be tidy.
        @(posedge clk); xgmii_rxd <= 64'h0707070707070707; xgmii_rxc <= 8'hFF;
        @(posedge clk); xgmii_rxd <= 64'h0707070707070707; xgmii_rxc <= 8'hFF;

        // START + preamble + SFD all share ONE 64-bit cycle:
        //   lane 0   = 0xFB (START, control)
        //   lanes 1..6 = 0x55 (preamble, data)
        //   lane 7   = 0xD5 (SFD, data)
        @(posedge clk);
        xgmii_rxd <= {8'hD5,
                      8'h55, 8'h55, 8'h55, 8'h55, 8'h55, 8'h55,
                      8'hFB};
        xgmii_rxc <= 8'b00000001;

        // Anchor the latency measurement on the cycle the FIRST FRAME BYTE is
        // actually on the bus. I originally had this line above the START drive,
        // which anchored it on an idle cycle and then counted the preamble cycle
        // as parse latency - two cycles of pure accounting error, so I was
        // reporting 11 for what is really a 9-cycle design. The preamble isn't
        // parsing, it's just the receiver locking on.
        //   cycle_count+1 = the START/preamble word
        //   cycle_count+2 = frame byte 0
        first_xgmii_cycle = cycle_count + 2;

        // Stream the frame bytes, 8 at a time.
        b = 0;
        while (b < pkt_len) begin
            @(posedge clk);
            rem = pkt_len - b;

            if (rem >= 8) begin
                // Whole cycle is data.
                xgmii_rxd <= {pkt[b+7], pkt[b+6], pkt[b+5], pkt[b+4],
                              pkt[b+3], pkt[b+2], pkt[b+1], pkt[b+0]};
                xgmii_rxc <= 8'h00;
                b = b + 8;
            end else begin
                // Last cycle: real bytes in the low lanes, TERM right after,
                // then IDLE in any remaining lanes.
                w = 64'h0707070707070707;
                c = 8'hFF;
                for (lane = 0; lane < rem; lane = lane + 1) begin
                    w[lane*8 +: 8] = pkt[b + lane];
                    c[lane]        = 1'b0;
                end
                term_lane          = rem;
                w[term_lane*8 +: 8] = 8'hFD;
                c[term_lane]        = 1'b1;
                xgmii_rxd <= w;
                xgmii_rxc <= c;
                b = pkt_len;
            end
        end

        // If the packet was an exact multiple of 8 bytes we need a
        // dedicated TERM cycle. (Our test packet isn't, so this path
        // doesn't fire here - it's just for completeness.)
        if (pkt_len % 8 == 0) begin
            @(posedge clk);
            xgmii_rxd <= {56'h07070707070707, 8'hFD};
            xgmii_rxc <= 8'b00000001;
        end

        // Trailing idles.
        @(posedge clk); xgmii_rxd <= 64'h0707070707070707; xgmii_rxc <= 8'hFF;
        @(posedge clk); xgmii_rxd <= 64'h0707070707070707; xgmii_rxc <= 8'hFF;
    end
endtask

// =============================================================================
// Main simulation flow
// =============================================================================
integer lat_cycles;
real    lat_ns;

initial begin
    $display("");
    $display("================================================");
    $display("  10G Ethernet -> IPv4 -> UDP parse pipeline");
    $display("  clock 156.25 MHz, 64-bit datapath");
    $display("================================================");

    repeat (10) @(posedge clk);
    rst = 0;
    $display("[t=%0t ns] reset released", $time);
    repeat (3) @(posedge clk);

    build_packet;
    $display("[t=%0t ns] sending packet:", $time);
    $display("  Ethernet : AA:BB:CC:DD:EE:FF <- 11:22:33:44:55:66  type=0x0800");
    $display("  IPv4     : 192.168.1.100 -> 239.255.43.1   protocol=UDP");
    $display("  UDP      : sport=12345  dport=26477 (Nasdaq ITCH)");
    $display("  Payload  : \"HELLO_HFT!\" (10 bytes)");
    $display("");

    drive_packet;

    // Give the pipeline time to drain.
    repeat (200) @(posedge clk);

    // ---- Report ----
    $display("");
    $display("================================================");
    if (first_payload_cycle == 0) begin
        $display("  FAIL: no payload was ever seen at the output");
    end else begin
        lat_cycles = first_payload_cycle - first_xgmii_cycle;
        lat_ns     = lat_cycles * 6.4;
        $display("  first XGMII byte    : cycle %0d", first_xgmii_cycle);
        $display("  first payload word  : cycle %0d", first_payload_cycle);
        $display("  parse latency       : %0d cycles = %0.1f ns", lat_cycles, lat_ns);
        $display("  parsed src IP       : %0d.%0d.%0d.%0d",
                 ip_src_out[31:24], ip_src_out[23:16], ip_src_out[15:8], ip_src_out[7:0]);
        $display("  parsed dst IP       : %0d.%0d.%0d.%0d",
                 ip_dst_out[31:24], ip_dst_out[23:16], ip_dst_out[15:8], ip_dst_out[7:0]);
        $display("  parsed src port     : %0d", udp_src_port_out);
        $display("  parsed dst port     : %0d", udp_dst_port_out);
        if (lat_ns <= 200.0)
            $display("  result: PASS (under 200 ns target)");
        else
            $display("  result: NOTE - latency %0.1f ns exceeds 200 ns", lat_ns);
    end
    $display("================================================");
    $finish;
end

// =============================================================================
// Monitor: print each payload word as it comes out of the pipeline.
// =============================================================================
always @(posedge clk) begin
    if (payload_tvalid) begin
        $display("  [cycle %0d] payload: 0x%016h  keep=%08b  last=%b",
                 cycle_count, payload_tdata, payload_tkeep, payload_tlast);
    end
end

// =============================================================================
// Watchdog: if something hangs, kill the sim instead of running forever.
// =============================================================================
initial begin
    #50000;
    $display("WATCHDOG: simulation timed out");
    $finish;
end

endmodule
