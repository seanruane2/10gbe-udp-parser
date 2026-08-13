// tb_udp_rx.v
// =============================================================================
// Self-checking regression for udp_rx_top / udp_rx_parser.
//
// My old testbench sent one hand-built happy-path packet and printed whatever
// came out the other end. Looking back that's a demo rather than verification -
// there's no way it could have caught the padding bug, and it never touched a
// single error path. This one builds packets from a parameterised generator,
// drives them onto XGMII, and scores every payload byte against a reference
// model.
//
// What it covers:
//   payload sizes 0,1,5,6,7,8,13,14,15,46,1472    (the 6/8-byte beat boundaries)
//   Ethernet padding to the 60-byte minimum        (the bug my old TB missed)
//   frame length an exact multiple of 8            (needs its own TERM cycle)
//   bad IPv4 header checksum
//   non-UDP protocol, non-IPv4 EtherType, VLAN tag
//   IPv4 options (IHL != 5), fragmented packet
//   UDP length that doesn't agree with the IP total length
//   destination-port filter, both accepting and rejecting
//   two frames back to back with the tightest gap I can drive
//   runt frame that ends inside the headers
//   truncated frame that ends inside the payload  (should abort with tuser)
//
// Every accepted packet also gets its wire-to-payload latency checked against
// EXP_LATENCY. I anchor the measurement on the cycle the FIRST FRAME BYTE is on
// the bus, not on an idle cycle before the preamble - that's what my old
// testbench did, and it's why it kept reporting 11 cycles for a 9-cycle design.
// =============================================================================

`timescale 1ns / 1ps
`default_nettype none

module tb_udp_rx;

// 156.25 MHz = 6.4 ns period
localparam real CLK_HALF = 3.2;

// Build with -d OUT_REG to test the registered-output version of the parser,
// which trades one cycle of latency for a clean registered module boundary.
`ifdef OUT_REG
localparam integer OUTREG = 1;
`else
localparam integer OUTREG = 0;
`endif

localparam integer EXP_LATENCY = 6 + OUTREG;  // wire byte 0 -> first payload beat

reg clk = 0;
always #CLK_HALF clk = ~clk;

reg rst = 1;

reg [63:0] xgmii_rxd = 64'h0707070707070707;
reg [7:0]  xgmii_rxc = 8'hFF;

reg        cfg_port_filter_en = 0;
reg [15:0] cfg_dst_port       = 16'd26477;

wire [63:0] payload_tdata;
wire [7:0]  payload_tkeep;
wire        payload_tlast;
wire        payload_tuser;
wire        payload_tvalid;

wire [47:0] eth_dst_mac, eth_src_mac;
wire [15:0] eth_type;
wire [31:0] ip_src, ip_dst;
wire [15:0] ip_total_length, udp_src_port, udp_dst_port, udp_length;
wire        hdr_valid;

wire [31:0] stat_frames, stat_accepted, stat_drop_ethertype, stat_drop_vlan;
wire [31:0] stat_drop_ihl, stat_drop_frag, stat_drop_proto, stat_drop_cksum;
wire [31:0] stat_drop_len, stat_drop_port, stat_runt, stat_abort;
wire [31:0] stat_xgmii_err, stat_backpressure;

udp_rx_top #(.OUTPUT_REG(OUTREG)) dut (
    .clk (clk), .rst (rst),
    .xgmii_rxd (xgmii_rxd), .xgmii_rxc (xgmii_rxc),
    .cfg_port_filter_en (cfg_port_filter_en), .cfg_dst_port (cfg_dst_port),
    .payload_tdata (payload_tdata), .payload_tkeep (payload_tkeep),
    .payload_tlast (payload_tlast), .payload_tuser (payload_tuser),
    .payload_tvalid (payload_tvalid), .payload_tready (1'b1),
    .eth_dst_mac (eth_dst_mac), .eth_src_mac (eth_src_mac), .eth_type (eth_type),
    .ip_src (ip_src), .ip_dst (ip_dst), .ip_total_length (ip_total_length),
    .udp_src_port (udp_src_port), .udp_dst_port (udp_dst_port),
    .udp_length (udp_length), .hdr_valid (hdr_valid),
    .stat_frames (stat_frames), .stat_accepted (stat_accepted),
    .stat_drop_ethertype (stat_drop_ethertype), .stat_drop_vlan (stat_drop_vlan),
    .stat_drop_ihl (stat_drop_ihl), .stat_drop_frag (stat_drop_frag),
    .stat_drop_proto (stat_drop_proto), .stat_drop_cksum (stat_drop_cksum),
    .stat_drop_len (stat_drop_len), .stat_drop_port (stat_drop_port),
    .stat_runt (stat_runt), .stat_abort (stat_abort),
    .stat_xgmii_err (stat_xgmii_err), .stat_backpressure (stat_backpressure)
);

// =============================================================================
// Bookkeeping
// =============================================================================
integer cycle_count = 0;
always @(posedge clk) cycle_count <= cycle_count + 1;

integer errors    = 0;
integer tests_run = 0;

reg [7:0] pkt [0:2047];      // frame as it goes on the wire
integer   pkt_len;
reg [7:0] exp_pl [0:2047];   // reference payload
integer   exp_len;

reg [7:0] got_pl [0:2047];   // payload as received
integer   got_len;
integer   got_pkts;
reg       got_tuser;
integer   first_byte_cycle;
integer   first_pl_cycle;
integer   measured_lat;
reg       exp_tuser;

// =============================================================================
// Scoreboard. Unpack every valid beat as it goes past, then score the whole
// packet once tlast lands.
// =============================================================================
integer i;
always @(posedge clk) begin
    if (payload_tvalid) begin
        if (first_pl_cycle < 0) begin
            first_pl_cycle = cycle_count;
            measured_lat   = first_pl_cycle - first_byte_cycle;
        end
        for (i = 0; i < 8; i = i + 1)
            if (payload_tkeep[i]) begin
                got_pl[got_len] = payload_tdata[i*8 +: 8];
                got_len = got_len + 1;
            end
        if (payload_tlast) begin
            got_tuser = payload_tuser;
            got_pkts  = got_pkts + 1;
            if (!payload_tuser) score_packet;
            got_len   = 0;
        end
    end
end

task score_packet;
    integer k;
    begin
        if (got_len != exp_len) begin
            $display("    FAIL payload length: got %0d bytes, expected %0d",
                     got_len, exp_len);
            errors = errors + 1;
        end else begin
            for (k = 0; k < exp_len; k = k + 1)
                if (got_pl[k] !== exp_pl[k]) begin
                    $display("    FAIL payload byte %0d: got 0x%02h, expected 0x%02h",
                             k, got_pl[k], exp_pl[k]);
                    errors = errors + 1;
                end
        end
    end
endtask

task clear_sb;
    begin
        got_len        = 0;
        got_pkts       = 0;
        got_tuser      = 0;
        first_pl_cycle = -1;
        measured_lat   = -1;
        exp_tuser      = 0;
    end
endtask

// =============================================================================
// Packet builder
// =============================================================================
task build;
    input integer payload_len;
    input [15:0]  etype;
    input [7:0]   ver_ihl;
    input [7:0]   proto;
    input [15:0]  frag_field;   // IPv4 flags + fragment offset
    input [15:0]  dport;
    input integer pad_to;       // 0 = no Ethernet padding
    input         corrupt_ck;
    input integer force_udp_len;// 0 = correct length, else override
    integer k;
    integer udp_len;
    integer tot_len;
    reg [31:0] acc;
    begin
        udp_len = (force_udp_len != 0) ? force_udp_len : (8 + payload_len);
        tot_len = 20 + 8 + payload_len;

        // ---- Ethernet ----
        pkt[ 0]=8'hAA; pkt[ 1]=8'hBB; pkt[ 2]=8'hCC;
        pkt[ 3]=8'hDD; pkt[ 4]=8'hEE; pkt[ 5]=8'hFF;
        pkt[ 6]=8'h11; pkt[ 7]=8'h22; pkt[ 8]=8'h33;
        pkt[ 9]=8'h44; pkt[10]=8'h55; pkt[11]=8'h66;
        pkt[12]=etype[15:8]; pkt[13]=etype[7:0];

        // ---- IPv4 ----
        pkt[14]=ver_ihl;      pkt[15]=8'h00;
        pkt[16]=tot_len[15:8];pkt[17]=tot_len[7:0];
        pkt[18]=8'h00;        pkt[19]=8'h00;
        pkt[20]=frag_field[15:8]; pkt[21]=frag_field[7:0];
        pkt[22]=8'h40;        pkt[23]=proto;
        pkt[24]=8'h00;        pkt[25]=8'h00;          // checksum, filled below
        pkt[26]=8'hC0; pkt[27]=8'hA8; pkt[28]=8'h01; pkt[29]=8'h64; // 192.168.1.100
        pkt[30]=8'hEF; pkt[31]=8'hFF; pkt[32]=8'h2B; pkt[33]=8'h01; // 239.255.43.1

        acc = 0;
        for (k = 14; k < 34; k = k + 2) acc = acc + {pkt[k], pkt[k+1]};
        acc = acc[31:16] + acc[15:0];
        acc = acc[31:16] + acc[15:0];
        pkt[24] = ~acc[15:8];
        pkt[25] = ~acc[7:0];
        if (corrupt_ck) pkt[25] = pkt[25] ^ 8'h01;

        // ---- UDP ----
        pkt[34]=8'h30;         pkt[35]=8'h39;          // src port 12345
        pkt[36]=dport[15:8];   pkt[37]=dport[7:0];
        pkt[38]=udp_len[15:8]; pkt[39]=udp_len[7:0];
        pkt[40]=8'h00;         pkt[41]=8'h00;          // UDP checksum unused

        // ---- Payload. Deterministic, but not all the same value, so that any
        // lane swapping actually shows up instead of hiding. ----
        for (k = 0; k < payload_len; k = k + 1) begin
            pkt[42+k]  = (k * 7 + 8'h11) & 8'hFF;
            exp_pl[k]  = (k * 7 + 8'h11) & 8'hFF;
        end
        exp_len = payload_len;
        pkt_len = 42 + payload_len;

        // ---- Ethernet padding, which must never reach the payload port ----
        if (pad_to > pkt_len) begin
            for (k = pkt_len; k < pad_to; k = k + 1) pkt[k] = 8'h00;
            pkt_len = pad_to;
        end
    end
endtask

// Flip one bit of the 16-bit IPv4 header checksum field (bytes 24..25).
task flip_ck_bit;
    input integer b;
    begin
        if (b < 8) pkt[25] = pkt[25] ^ (8'h01 << b);
        else       pkt[24] = pkt[24] ^ (8'h01 << (b - 8));
    end
endtask

// =============================================================================
// XGMII driver. Sends START/preamble, then the frame, then TERM. I don't put any
// leading or trailing idles in here on purpose - the caller decides the gap, so
// you can drive back-to-back frames just by calling this twice in a row.
// =============================================================================
task drive_frame;
    input integer nbytes;      // how many bytes to actually put on the wire
    integer b, lane, rem;
    reg [63:0] w;
    reg [7:0]  c;
    begin
        @(posedge clk);
        xgmii_rxd <= {8'hD5, 8'h55,8'h55,8'h55,8'h55,8'h55,8'h55, 8'hFB};
        xgmii_rxc <= 8'b00000001;
        // START sits in cycle_count+1, so frame byte 0 is on the bus at +2.
        first_byte_cycle = cycle_count + 2;

        b = 0;
        while (b < nbytes) begin
            @(posedge clk);
            rem = nbytes - b;
            if (rem >= 8) begin
                xgmii_rxd <= {pkt[b+7],pkt[b+6],pkt[b+5],pkt[b+4],
                              pkt[b+3],pkt[b+2],pkt[b+1],pkt[b+0]};
                xgmii_rxc <= 8'h00;
                b = b + 8;
            end else begin
                w = 64'h0707070707070707;
                c = 8'hFF;
                for (lane = 0; lane < rem; lane = lane + 1) begin
                    w[lane*8 +: 8] = pkt[b+lane];
                    c[lane]        = 1'b0;
                end
                w[rem*8 +: 8] = 8'hFD;   // TERM straight after the last byte
                c[rem]        = 1'b1;
                xgmii_rxd <= w;
                xgmii_rxc <= c;
                b = nbytes;
            end
        end

        // Frame came out as an exact multiple of 8 bytes, so TERM needs a
        // cycle all to itself.
        if (nbytes % 8 == 0) begin
            @(posedge clk);
            xgmii_rxd <= {56'h07070707070707, 8'hFD};
            xgmii_rxc <= 8'b00000001;
        end
    end
endtask

task idle;
    input integer n;
    integer k;
    begin
        for (k = 0; k < n; k = k + 1) begin
            @(posedge clk);
            xgmii_rxd <= 64'h0707070707070707;
            xgmii_rxc <= 8'hFF;
        end
    end
endtask

// =============================================================================
// Result checkers
// =============================================================================
task expect_accept;
    input [511:0] name;         // 64 chars. Any shorter and the names get clipped.
    input integer want_pkts;
    begin
        tests_run = tests_run + 1;
        if (got_pkts != want_pkts) begin
            $display("    FAIL %0s: %0d packets delivered, expected %0d",
                     name, got_pkts, want_pkts);
            errors = errors + 1;
        end else if (got_tuser) begin
            $display("    FAIL %0s: packet arrived flagged bad (tuser)", name);
            errors = errors + 1;
        end else if (measured_lat != EXP_LATENCY) begin
            $display("    FAIL %0s: latency %0d cycles, expected %0d",
                     name, measured_lat, EXP_LATENCY);
            errors = errors + 1;
        end else begin
            $display("    pass %0s  (%0d B payload, %0d cycles = %0.1f ns)",
                     name, exp_len, measured_lat, measured_lat * 6.4);
        end
    end
endtask

task expect_drop;
    input [511:0] name;
    input [31:0]  counter;
    input [31:0]  want_counter;
    begin
        tests_run = tests_run + 1;
        if (got_pkts != 0) begin
            $display("    FAIL %0s: %0d packets leaked through", name, got_pkts);
            errors = errors + 1;
        end else if (counter !== want_counter) begin
            $display("    FAIL %0s: drop counter is %0d, expected %0d",
                     name, counter, want_counter);
            errors = errors + 1;
        end else begin
            $display("    pass %0s  (dropped, counted)", name);
        end
    end
endtask

// =============================================================================
// Tests
// =============================================================================
integer sizes [0:10];
integer si;
integer ck_drops;

initial begin
    $display("");
    $display("==========================================================");
    $display("  udp_rx_parser regression - 10GbE UDP receive path");
    $display("  156.25 MHz, 64-bit datapath, expecting %0d-cycle latency",
             EXP_LATENCY);
    $display("==========================================================");

    repeat (8) @(posedge clk);
    rst = 0;
    repeat (4) @(posedge clk);

    // ---------------------------------------------------------------------
    $display("\n-- payload sizes (exercises the 6-byte first beat) --");
    sizes[0]=1;  sizes[1]=5;  sizes[2]=6;  sizes[3]=7;   sizes[4]=8;
    sizes[5]=13; sizes[6]=14; sizes[7]=15; sizes[8]=46;  sizes[9]=1472;
    sizes[10]=22;   // frame = 64 bytes, an exact multiple of 8
    for (si = 0; si <= 10; si = si + 1) begin
        clear_sb;
        build(sizes[si], 16'h0800, 8'h45, 8'd17, 16'h4000, 16'd26477, 0, 0, 0);
        drive_frame(pkt_len);
        idle(24);
        expect_accept(sizes[si] == 22 ? "size 22 (64B frame, own TERM cycle)"
                                      : "payload size", 1);
    end

    // ---------------------------------------------------------------------
    $display("\n-- Ethernet padding has to be stripped (the v1 bug) --");
    clear_sb;
    build(10, 16'h0800, 8'h45, 8'd17, 16'h4000, 16'd26477, 60, 0, 0);
    drive_frame(pkt_len);
    idle(24);
    expect_accept("10 B payload in a 60 B padded frame", 1);

    clear_sb;
    build(0, 16'h0800, 8'h45, 8'd17, 16'h4000, 16'd26477, 60, 0, 0);
    drive_frame(pkt_len);
    idle(24);
    tests_run = tests_run + 1;
    if (got_pkts != 0) begin
        $display("    FAIL zero-length payload emitted %0d beats", got_pkts);
        errors = errors + 1;
    end else
        $display("    pass zero-length UDP payload  (header only, no beats)");

    // ---------------------------------------------------------------------
    $display("\n-- header field extraction --");
    clear_sb;
    build(10, 16'h0800, 8'h45, 8'd17, 16'h4000, 16'd26477, 60, 0, 0);
    drive_frame(pkt_len);
    idle(24);
    tests_run = tests_run + 1;
    if (eth_dst_mac !== 48'hAABBCCDDEEFF || eth_src_mac !== 48'h112233445566 ||
        ip_src !== 32'hC0A80164 || ip_dst !== 32'hEFFF2B01 ||
        udp_src_port !== 16'd12345 || udp_dst_port !== 16'd26477 ||
        udp_length !== 16'd18 || ip_total_length !== 16'd38) begin
        $display("    FAIL sideband: mac %012h/%012h ip %08h/%08h port %0d/%0d len %0d/%0d",
                 eth_dst_mac, eth_src_mac, ip_src, ip_dst,
                 udp_src_port, udp_dst_port, udp_length, ip_total_length);
        errors = errors + 1;
    end else
        $display("    pass parsed MACs, IPs, ports and lengths all correct");

    // ---------------------------------------------------------------------
    $display("\n-- IPv4 checksum: every single-bit corruption must be caught --");
    // The parser turns the usual add-fold-fold-compare into a carry-free
    // pattern match. That's only a good idea if it really is identical, so I
    // sweep every bit of the checksum field and corrupt an address byte too.
    ck_drops = 0;
    for (si = 0; si < 16; si = si + 1) begin
        clear_sb;
        build(10, 16'h0800, 8'h45, 8'd17, 16'h4000, 16'd26477, 60, 0, 0);
        flip_ck_bit(si);
        drive_frame(pkt_len); idle(24);
        ck_drops = ck_drops + 1;
        expect_drop("checksum field bit flip", stat_drop_cksum, ck_drops);
    end
    clear_sb;
    build(10, 16'h0800, 8'h45, 8'd17, 16'h4000, 16'd26477, 60, 0, 0);
    pkt[26] = pkt[26] ^ 8'h20;      // corrupt the source IP instead
    drive_frame(pkt_len); idle(24);
    ck_drops = ck_drops + 1;
    expect_drop("corrupted source IP caught by checksum", stat_drop_cksum, ck_drops);

    $display("\n-- packets that must be dropped --");
    clear_sb;
    build(10, 16'h0800, 8'h45, 8'd6, 16'h4000, 16'd26477, 60, 0, 0);
    drive_frame(pkt_len); idle(24);
    expect_drop("protocol 6 (TCP)", stat_drop_proto, 32'd1);

    clear_sb;
    build(10, 16'h0806, 8'h45, 8'd17, 16'h4000, 16'd26477, 60, 0, 0);
    drive_frame(pkt_len); idle(24);
    expect_drop("EtherType 0x0806 (ARP)", stat_drop_ethertype, 32'd1);

    clear_sb;
    build(10, 16'h8100, 8'h45, 8'd17, 16'h4000, 16'd26477, 60, 0, 0);
    drive_frame(pkt_len); idle(24);
    expect_drop("VLAN-tagged frame", stat_drop_vlan, 32'd1);

    clear_sb;
    build(10, 16'h0800, 8'h46, 8'd17, 16'h4000, 16'd26477, 60, 0, 0);
    drive_frame(pkt_len); idle(24);
    expect_drop("IPv4 options (IHL=6)", stat_drop_ihl, 32'd1);

    clear_sb;
    build(10, 16'h0800, 8'h45, 8'd17, 16'h2000, 16'd26477, 60, 0, 0);
    drive_frame(pkt_len); idle(24);
    expect_drop("fragmented (MF set)", stat_drop_frag, 32'd1);

    clear_sb;
    build(10, 16'h0800, 8'h45, 8'd17, 16'h0001, 16'd26477, 60, 0, 0);
    drive_frame(pkt_len); idle(24);
    expect_drop("fragmented (offset != 0)", stat_drop_frag, 32'd2);

    clear_sb;
    build(10, 16'h0800, 8'h45, 8'd17, 16'h4000, 16'd26477, 60, 0, 99);
    drive_frame(pkt_len); idle(24);
    expect_drop("UDP length inconsistent with IP total", stat_drop_len, 32'd1);

    // ---------------------------------------------------------------------
    $display("\n-- destination-port filter --");
    cfg_port_filter_en = 1;
    clear_sb;
    build(10, 16'h0800, 8'h45, 8'd17, 16'h4000, 16'd26477, 60, 0, 0);
    drive_frame(pkt_len); idle(24);
    expect_accept("wanted port 26477 accepted", 1);

    clear_sb;
    build(10, 16'h0800, 8'h45, 8'd17, 16'h4000, 16'd9999, 60, 0, 0);
    drive_frame(pkt_len); idle(24);
    expect_drop("unwanted port 9999 rejected", stat_drop_port, 32'd1);
    cfg_port_filter_en = 0;

    // ---------------------------------------------------------------------
    $display("\n-- malformed framing --");
    clear_sb;
    build(10, 16'h0800, 8'h45, 8'd17, 16'h4000, 16'd26477, 0, 0, 0);
    drive_frame(20);            // frame dies partway through the IPv4 header
    idle(24);
    expect_drop("runt: frame ends inside the headers", stat_runt, 32'd1);

    clear_sb;
    build(40, 16'h0800, 8'h45, 8'd17, 16'h4000, 16'd26477, 0, 0, 0);
    drive_frame(52);            // headers promise 40 B of payload, only 10 turn up
    idle(24);
    tests_run = tests_run + 1;
    if (got_pkts != 1 || !got_tuser || stat_abort !== 32'd1) begin
        $display("    FAIL truncated frame: pkts=%0d tuser=%0b stat_abort=%0d",
                 got_pkts, got_tuser, stat_abort);
        errors = errors + 1;
    end else
        $display("    pass truncated payload aborted with tuser=1");

    // ---------------------------------------------------------------------
    $display("\n-- two frames back to back, tightest gap --");
    clear_sb;
    build(24, 16'h0800, 8'h45, 8'd17, 16'h4000, 16'd26477, 0, 0, 0);
    drive_frame(pkt_len);
    drive_frame(pkt_len);       // the next START lands in the word right after TERM
    idle(24);
    expect_accept("back-to-back frames", 2);

    // ---------------------------------------------------------------------
    $display("\n-- we never claimed backpressure we couldn't honour --");
    tests_run = tests_run + 1;
    if (stat_backpressure !== 32'd0) begin
        $display("    FAIL stat_backpressure = %0d", stat_backpressure);
        errors = errors + 1;
    end else
        $display("    pass stat_backpressure = 0");

    // ---------------------------------------------------------------------
    $display("");
    $display("==========================================================");
    $display("  frames seen %0d, accepted %0d", stat_frames, stat_accepted);
    $display("  checks run  %0d", tests_run);
    if (errors == 0)
        $display("  RESULT: PASS - all checks clean, latency %0d cycles = %0.1f ns",
                 EXP_LATENCY, EXP_LATENCY * 6.4);
    else
        $display("  RESULT: FAIL - %0d error(s)", errors);
    $display("==========================================================");
    $finish;
end

initial begin
    #4000000;
    $display("WATCHDOG: simulation timed out");
    $finish;
end

endmodule

`default_nettype wire
