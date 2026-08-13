# How the v1 layered UDP receive chain works

> **This describes the superseded four-stage design** (`xgmii_rx → eth_rx →
> ipv4_rx → udp_rx`), kept because the layered version is the clearest way to
> explain byte realignment, and because it is the baseline for the latency
> comparison. The design actually in use is the fused single-stage parser in
> `rtl/udp_rx_parser.v` — see [README.md](README.md). v1 takes 9 cycles from wire
> to payload; v2 takes 6. v1 also forwards Ethernet padding as payload, which is
> the bug that motivated the rewrite.

A walkthrough of what each piece of this design does, why it does it that way,
and what was broken before. Aimed at someone who's just learning network
hardware and Verilog at the same time.

---

## 1. The big picture

The job: a 10G Ethernet link is delivering packets. Most of those packets carry
market data wrapped in three layers of header (Ethernet, IPv4, UDP). We want
to strip all three headers off in hardware and hand the application's actual
payload to the next stage as fast as possible.

```
                   raw 10G wire signals (XGMII)
                              |
        +---------------------|---------------------+
        |                     v                     |
        |   +------------+   bytes   +-----------+  |
        |   |  xgmii_rx  |---------->|  eth_rx   |  |
        |   +------------+           +-----------+  |
        |                                  |        |
        |                                  v        |
        |                            +-----------+  |
        |                            |  ipv4_rx  |  |
        |                            +-----------+  |
        |                                  |        |
        |                                  v        |
        |                            +-----------+  |
        |                            |  udp_rx   |  |
        |                            +-----------+  |
        |                                  |        |
        +----------------------------------|--------+
                                           v
                              UDP application payload
                              (AXI-Stream of raw bytes)
```

Every arrow between stages is an **AXI-Stream** bus. AXI-Stream is just a few
wires that move chunks of data from a sender to a receiver one cycle at a time:

| Signal   | Meaning                                                        |
| -------- | -------------------------------------------------------------- |
| `tdata`  | The actual bytes. Here it's 64 bits = 8 bytes per cycle.       |
| `tkeep`  | 1 bit per byte saying "this byte is real" (not padding).       |
| `tlast`  | High on the very last cycle of a packet.                       |
| `tvalid` | "The sender has real data this cycle."                         |
| `tready` | "The receiver is ready to accept data."                        |

Bytes only move when `tvalid` AND `tready` are both high in the same cycle.
In this design we never assert `tready` low (no backpressure) - simpler that
way, and it's fine for the simulation.

---

## 2. The clock and the datapath

- **Clock:** 156.25 MHz. That's the standard 10G Ethernet clock.
  Period = 6.4 ns.
- **Datapath:** 64 bits = 8 bytes per cycle.
  At 156.25 MHz x 64 bits = 10 Gbit/s. Exactly fast enough.

Every register in the design clocks at 156.25 MHz. The whole pipeline runs
in lockstep, one byte-octet per cycle.

---

## 3. Stage 1 - xgmii_rx (the front door)

### What XGMII is

XGMII (10G Media Independent Interface) is the interface between the physical
transceiver (the optical chip on the board) and the digital logic. It has:

- **64 data wires** (8 bytes worth)
- **8 control wires** (1 bit per byte, telling us "this byte is special")

Three special bytes matter:

| Byte | Name  | Meaning                                                         |
| ---- | ----- | --------------------------------------------------------------- |
| 0xFB | START | "A packet starts here."                                         |
| 0xFD | TERM  | "A packet ends here."                                           |
| 0x07 | IDLE  | "Nothing's being sent right now - filler."                      |

A frame on the wire looks like:

```
idle  idle  idle  [START + 7 preamble bytes]  [8 bytes data]  [8 bytes data]
                                              ...    [last data + TERM + idle]   idle  idle
```

The preamble bytes (0x55 0x55 ... 0xD5) are sync bytes - they help the receiver
lock onto the signal. They're not part of the actual frame, so we drop them.

### What xgmii_rx does

1. Sit in `S_IDLE`, watching `xgmii_rxd` for the START byte (0xFB in lane 0).
2. When START is seen, that whole cycle is preamble - we ignore it. Move to
   `S_DATA` on the next clock.
3. In `S_DATA`, every cycle the 8 bytes of `xgmii_rxd` are real frame bytes.
   Pump them onto the output AXI-Stream with `tkeep=0xFF`, `tlast=0`.
4. If TERM appears, the bytes BEFORE the TERM byte are the last real bytes.
   So we set `tkeep` to keep just those, `tlast=1`, and go back to `S_IDLE`.

For our test packet (52 bytes), the TERM ends up in lane 4 of the final cycle,
so `tkeep = 0000_1111` (bytes 0..3 are valid, byte 4 is TERM, bytes 5..7 are idle).

---

## 4. Stage 2 - eth_rx (the post office sorter)

### The Ethernet header (14 bytes)

| Bytes  | Field                                               |
| ------ | --------------------------------------------------- |
| 0..5   | Destination MAC                                     |
| 6..11  | Source MAC                                          |
| 12..13 | EtherType (0x0800 = IPv4)                           |

### The job

- Capture DST MAC, SRC MAC, EtherType.
- If EtherType is IPv4, forward bytes 14+ to the next stage.
- Otherwise, drop the rest of the packet.

### The alignment problem (this is where the original code was broken)

The bus is 8 bytes wide. The header is 14 bytes - **not** a multiple of 8.
So:

```
cycle 1:  [ 0  1  2  3  4  5  6  7 ]   <- all header
cycle 2:  [ 8  9 10 11 12 13 14 15 ]   <- bytes 14, 15 belong to IP, not Ethernet
```

When we finish reading the Ethernet header in cycle 2, two bytes of IP header
data are sitting in the top half of the same bus cycle. If we throw them away,
the IPv4 parser will see the IP header shifted by 2 bytes and read total nonsense.

**The fix:** keep a small "residue" register that holds those 2 bytes. On the
next cycle, glue the residue onto the low end of the new input. Now the output
stream looks 8-byte-aligned to the next stage:

```
input cycle 2:  bytes  8  9 10 11 12 13 14 15
                                       \____/
                                        |
                                       save into residue

input cycle 3:  bytes 16 17 18 19 20 21 22 23   ___________
                |________________________|     |           |
                            |                  |           |
output cycle 1:  bytes 14 15 16 17 18 19 20 21 (with residue 22 23
                                                 saved for next cycle)

input cycle 4:  bytes 24 25 26 27 28 29 30 31
                |________________________|
                            |
output cycle 2:  bytes 22 23 24 25 26 27 28 29 (residue saved: 30 31)
```

And so on. Each output cycle is 6 fresh input bytes + 2 saved residue bytes.

### The state machine

| State    | What it does                                                |
| -------- | ----------------------------------------------------------- |
| HEADER1  | Capture bytes 0..7 (DST MAC + start of SRC MAC).           |
| HEADER2  | Capture bytes 8..15. Save EtherType + residue.             |
| DATA     | Stream out realigned data, 6 new + 2 residue per cycle.    |
| FLUSH    | If a packet ended with leftover residue, push it out alone. |
| DROP     | Wrong EtherType - swallow the rest until tlast.            |

### What was broken before

The previous code had a comment that promised to do the residue trick, but the
code itself didn't. Instead it produced no output during HEADER2 and then in
DATA just forwarded the raw input cycle unchanged. So the IPv4 parser got the
IP stream shifted by 2 bytes - it thought "DSCP" (the second byte of the IP
header) was "Version+IHL", and it thought the second byte of the IP checksum
was the protocol field. Of course the protocol mismatch then caused the packet
to be dropped, and the testbench reported "FAIL: no payload was ever seen."

---

## 5. Stage 3 - ipv4_rx (the customs inspector)

### The IPv4 header (20 bytes, no options)

| Bytes  | Field                                               |
| ------ | --------------------------------------------------- |
| 0      | Version (4) + IHL (5) = 0x45                        |
| 1      | DSCP / ECN (QoS - we ignore)                        |
| 2..3   | Total Length                                        |
| 4..5   | Identification                                      |
| 6..7   | Flags + Fragment Offset                             |
| 8      | TTL                                                 |
| 9      | Protocol (17 = UDP)                                 |
| 10..11 | Header Checksum                                     |
| 12..15 | Source IP                                           |
| 16..19 | Destination IP                                      |

### The IPv4 checksum, in plain English

1. Treat the 20-byte header as ten 16-bit numbers.
2. Add them all up using ordinary unsigned addition.
3. If the sum overflows past 16 bits, fold the high half back into the low half.
   Repeat until the high half is zero. This is **one's-complement addition**.
4. Flip every bit.
5. The sender writes that 16-bit result into bytes 10..11.

On the receive side, you redo the sum (the checksum field is now non-zero,
since it's part of the data). After folding, a good header gives you
**0xFFFF** - that's the indicator of a valid checksum.

### The alignment trick, take 2

The IP header is 20 bytes - again not a multiple of 8. That means after the
two full header cycles, 4 IP bytes (the destination IP) live in the **low** half
of the third cycle, and **4 UDP bytes** live in the **high** half.

So this time the residue is **4 bytes** instead of 2, and the alignment shifts
by 4 bytes per cycle from there on. Exactly the same trick, just shifted by a
different amount.

### The state machine

| State    | What it does                                              |
| -------- | --------------------------------------------------------- |
| HEADER1  | IP bytes 0..7. Start checksum sum.                       |
| HEADER2  | IP bytes 8..15. Add to checksum, save SRC IP + Protocol. |
| HEADER3  | IP bytes 16..19 + first 4 UDP bytes. Finish checksum.    |
| DATA     | Shift 4-byte residue into output stream.                  |
| FLUSH    | Drain leftover residue at packet end.                     |
| DROP     | Bad checksum or non-UDP - swallow the rest.              |

---

## 6. Stage 4 - udp_rx (the final unwrap)

### The UDP header (8 bytes)

| Bytes | Field             |
| ----- | ----------------- |
| 0..1  | Source Port       |
| 2..3  | Destination Port  |
| 4..5  | UDP Length        |
| 6..7  | Checksum (we ignore - optional in IPv4) |

8 bytes = 1 bus cycle. **No alignment shenanigans needed.** This is the easiest
stage:

1. First cycle: extract the three fields, publish them on the sideband, move to
   the DATA state.
2. Every cycle after: pass the data straight through to the payload output.

This is the actual application payload. In a real HFT system this is where an
ITCH parser, OUCH order encoder, or whatever your trading logic is would
connect in.

---

## 7. Why "residue" and not something fancier?

In a real high-performance design you might use a **shift register** or **barrel
shifter** to do the same job, and you might handle every possible header
alignment in parallel. We didn't, because:

- The protocols we parse have fixed, known header sizes. Ethernet is always
  14 bytes, IPv4 is 20 bytes (no options), UDP is 8 bytes. So the shift
  amount is hard-coded, not dynamic.
- A 2-byte or 4-byte residue is dirt cheap in hardware (just a small register).
- Keeping it simple makes the code easier to read and debug.

If you ever need to handle IPv4 options (variable-length header), TCP (also
variable-length), or VLAN-tagged Ethernet, the residue idea still works - you
just compute the shift amount at runtime instead of hard-coding it.

---

## 8. The testbench, step by step

`tb/tb_udp_stack.v` does five things:

1. **Generate clock and reset.** 156.25 MHz, ten cycles of reset.
2. **Build a packet in memory** (`pkt[]`). All 52 bytes of an Ethernet/IPv4/UDP
   frame addressed to a fake destination, carrying the string "HELLO_HFT!" as
   payload. The IP checksum is computed by software so it'll pass.
3. **Drive the packet onto XGMII** (`drive_packet`). Adds the START/preamble
   on the front and the TERM on the back. Each frame byte goes into one of
   the eight lanes per cycle.
4. **Watch for payload** (`always @(posedge clk) if (payload_tvalid) ...`).
   When the first payload word arrives, remember the cycle number.
5. **Report.** Show parsed IPs, ports, payload bytes, and total latency.

Run output (Vivado xsim, after the alignment fix and the testbench anchor fix
described below):

```
[cycle 26] payload: 0x46485f4f4c4c4548  keep=11111111  last=0   <- "HELLO_HF"
[cycle 27] payload: 0x0000000007fd2154  keep=00000011  last=1   <- "T!" (only 2 bytes valid)
parse latency       : 9 cycles = 57.6 ns
parsed src IP       : 192.168.1.100
parsed dst IP       : 239.255.43.1
parsed src port     : 12345
parsed dst port     : 26477
result: PASS (under 200 ns target)
```

An earlier version of this document claimed 12 cycles. That number was wrong
twice over: the testbench used to anchor its measurement on an idle cycle two
cycles before the first frame byte reached the bus (counting the preamble as
parse latency), and the figure quoted did not match even that. The real,
measured, wire-byte-0-to-payload figure for this chain is 9.

The second payload word has `tkeep=0000_0011`, meaning only bytes 0 and 1
(`0x54 = 'T'`, `0x21 = '!'`) are real. The high bits of that word
(`0x07fd`...) are XGMII filler bytes (IDLE and TERM) that propagated through
the pipeline because we didn't bother masking invalid bytes to zero. That's
harmless because tkeep tells downstream logic to ignore them. If you want a
prettier waveform, you could mask them in xgmii_rx - but functionally it makes
no difference.

---

## 9. Latency, end-to-end

Each pipeline stage costs its own header cycles plus, for most of them, one
cycle of output registering:

| Stage     | Cycles added                        |
| --------- | ----------------------------------- |
| xgmii_rx  | 1 register stage                    |
| eth_rx    | 2 header cycles + 1 register stage  |
| ipv4_rx   | 3 header cycles + 1 register stage  |
| udp_rx    | 1 header cycle + 0 (its payload path is combinational) |

That sums to **9 cycles = 57.6 ns**, which is what the simulation measures.

The output registers are the interesting entry. They are not just three cycles
of delay — they re-serialise the byte stream, so each stage cannot begin looking
at its own header until the stage before it has finished registering. The header
*decode* never needed those cycles, because Ethernet, IPv4 and UDP headers sit at
fixed, known byte offsets. Removing the inter-stage boundaries entirely is what
`udp_rx_parser.v` does, and it gets to 6 cycles — the floor, since the first
payload byte does not physically arrive on the wire until the sixth word.

---

## 10. The files

```
rtl/
  xgmii_rx.v        Strips XGMII framing.
  eth_rx.v          Strips Ethernet header, realigns by 2 bytes.
  ipv4_rx.v         Strips IPv4 header, validates checksum, realigns by 4 bytes.
  udp_rx.v          Strips UDP header (no realignment needed).
  udp_stack_top.v   Connects all four stages together.
  axis_fifo.v       Optional buffer FIFO (not currently wired in).
tb/
  tb_udp_stack.v    Testbench - hand-builds a packet and drives it in.
sim/
  run_sim.tcl       Vivado xsim script for the command-line.
tb_udp_stack_behav.wcfg   Vivado waveform configuration (signal layout).
```

---

## 11. What the bug actually was

To summarise:

- The original `eth_rx` claimed in a comment to handle the 2-byte alignment
  problem, but the actual code didn't. The 2 IP-header bytes that arrived in
  the same cycle as the EtherType were silently dropped, and from there on the
  IP stream was shifted by 2 bytes.
- That made `ipv4_rx` read the Protocol field from a position where the IP
  checksum's low byte actually lived. The "wrong protocol" check fired and
  the packet went into the drop state.
- No data ever reached `udp_rx`, so no payload ever came out, and the
  testbench printed "FAIL".

The fix was to give `eth_rx` a proper 2-byte residue register and shift its
output stream correctly, and to give `ipv4_rx` the same treatment with a
4-byte residue at its IP-to-UDP boundary. With both alignments correct, the
packet flows end-to-end and the testbench passes.
