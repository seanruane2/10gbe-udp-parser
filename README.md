# 10 GbE UDP receive parser

A 64-bit XGMII → Ethernet → IPv4 → UDP header-stripping pipeline in Verilog,
built for minimum wire-to-payload latency. At 156.25 MHz with a 64-bit datapath
it sustains 10 Gbit/s line rate, and delivers the first payload byte **6 clock
cycles (38.4 ns)** after the first frame byte appears on the wire — which is the
hardware floor for a 64-bit bus (see [Why 6 is the floor](#why-6-is-the-floor)).

Everything below is measured, not estimated: latency and functional results come
from Vivado xsim, timing and area from a full synthesise-place-route run. Both
are reproducible from `sim/`.

---

## Results

### Latency

| Design | Cycles, wire byte 0 → first payload byte | Time |
| ------ | ---------------------------------------- | ---- |
| v1, four-stage layered chain | 9 | 57.6 ns |
| **v2, fused parser** (`OUTPUT_REG=0`, default) | **6** | **38.4 ns** |
| v2 with a registered output boundary (`OUTPUT_REG=1`) | 7 | 44.8 ns |

A 33% reduction, and v2 sits exactly on the theoretical floor.

### Timing and area

Out-of-context synthesis, place and route, `sim/synth_check.tcl`. The clock
constraint is 6.4 ns (156.25 MHz). Slack is the internal reg-to-reg path.

| Part | Slack | Implied Fmax | LUTs | FFs |
| ---- | ----- | ------------ | ---- | --- |
| `xc7k160tfbg484-2` Kintex-7 −2 (a realistic 10G part) | **+2.540 ns** | 259 MHz | 332 | 834 |
| `xc7z020clg400-1` Zynq-7020 −1 (Pynq-Z2, slowest grade) | **+0.511 ns** | 170 MHz | 333 | 834 |

Roughly 0.3% of a Kintex-160T. Over half the flip-flops are the fourteen 32-bit
diagnostic counters; drop the ones you do not need and the design is smaller
still.

### Verification

`tb/tb_udp_rx.v` — 44 self-checking assertions, all passing. Every payload byte
is scored against a reference model, and every accepted packet has its latency
checked.

```
payload sizes 0,1,5,6,7,8,13,14,15,22,46,1472    beat-boundary sweep
Ethernet padding to the 60-byte minimum          padding must not reach payload
frame length an exact multiple of 8              dedicated TERM cycle
IPv4 checksum, all 16 single-bit corruptions     plus a corrupted address byte
non-UDP protocol / non-IPv4 EtherType / VLAN     dropped and counted
IPv4 options (IHL != 5) / fragmented packet      dropped and counted
UDP length inconsistent with IP total length     dropped and counted
destination-port filter, accept and reject
two frames back to back, tightest gap
runt frame ending inside the headers
truncated frame ending inside the payload        aborts with tuser=1
header field extraction                          MACs, IPs, ports, lengths
```

---

## How it works

```
        XGMII (64b data + 8b control) @ 156.25 MHz
                        |
                        v
              +-------------------+
              |   udp_rx_parser   |   one input register, no internal stages
              +-------------------+
                        |
                        v
        UDP payload, AXI-Stream (tdata/tkeep/tlast/tuser/tvalid)
        + parsed headers on the sideband
```

### Why 6 is the floor

Number the 64-bit words coming off the wire, word 0 = frame bytes 0..7:

| Word | Bytes | Contents |
| ---- | ----- | -------- |
| 0 | 0..7 | dst MAC, src MAC[0:1] |
| 1 | 8..15 | src MAC[2:5], EtherType, IP[0..1] |
| 2 | 16..23 | IP[2..9] — total length, flags/frag, protocol |
| 3 | 24..31 | IP[10..17] — checksum, src IP, dst IP high |
| 4 | 32..39 | IP[18..19], UDP[0..5] — ports, UDP length |
| 5 | 40..47 | UDP[6..7], **payload bytes 0..5** |
| 6 | 48..55 | payload bytes 6..13 |

Ethernet (14) + IPv4 (20, IHL=5) + UDP (8) = 42 bytes, so payload byte 0 sits at
offset 42 and physically arrives in word 5. Nothing can come out before word 5
exists. You must register the XGMII bus once — you cannot run parse logic
combinationally off the PHY pins — so word 5 is usable in cycle 6. That is the
floor, and that is where this lands.

### 42 mod 8 == 2

The nice part. Because the headers are 42 bytes and 42 mod 8 == 2, once the six
leftover payload bytes in word 5 are emitted, **every later word is already
payload-aligned**: bytes 6..13 are exactly word 6, bytes 14..21 are exactly
word 7.

* beat 0 = top 6 bytes of word 5, shifted down two lanes by fixed wiring
* beat N = word 5+N, passed through completely untouched

There is no barrel shifter, no residue register, and no realignment pipeline in
the design. The output datapath is a single 2:1 mux on 48 bits. The price is
that the first beat carries 6 valid bytes instead of 8 — `tkeep` qualifies every
beat, and payload byte 0 is always in lane 0.

### Framing comes from UDP length, not from XGMII TERM

The parser reads the UDP length in word 4 — one full cycle before the first
payload byte arrives — and counts exactly that many bytes out. Two things fall
out of this for free:

* **Ethernet padding is ignored.** Frames are padded to a 60-byte minimum on the
  wire; a length-driven parser simply stops before the padding.
* **The TERM-in-lane-0 problem disappears.** When the XGMII TERM character lands
  in lane 0 it means "the *previous* word was the last one", which normally
  forces a cycle of lookahead to fix up `tkeep`/`tlast` retroactively. If the
  length already told you where the packet ends, there is nothing to fix up.

TERM is then only a consistency check: if the frame ends before the promised
bytes are delivered, the beat is terminated with `tuser=1`.

### Validation, all of it before the first payload byte

| Check | Known by |
| ----- | -------- |
| EtherType == 0x0800 | cycle 2 |
| Version/IHL == 0x45 | cycle 2 |
| not fragmented (MF=0, offset=0) | cycle 3 |
| Protocol == 17 (UDP) | cycle 3 |
| IPv4 header checksum valid | cycle 5 |
| UDP length sane and consistent with IP total length | cycle 5 |
| destination port matches the filter | cycle 5 |

So this is cut-through with **no speculation** — a packet is never partially
emitted and then retracted. That works because the IPv4 checksum covers only the
header, which is complete in word 4. Anything that had to be validated across
the payload (a UDP checksum, or the Ethernet FCS) would have to be forwarded
speculatively and retracted late; the `tuser` plumbing is already there for it.

### The two paths that had to be restructured

The first build of this design missed 6.4 ns by 3.2 ns. Both offenders were in
the accept decision in cycle 5, and both are worth knowing about.

**1. The checksum fold.** The textbook ending to an IPv4 checksum is: add the
last word, fold the carries twice, compare against 0xFFFF. That is four
dependent adders — 13 CARRY4s in series, 9.35 ns post-route. But the accumulator
is only 20 bits, so the half being folded back is just 4 bits, and:

* the second fold can never produce a pass (if the first fold overflows, the
  result is at most 0x000F), so the test collapses to `S[19:16] + S[15:0] == 0xFFFF`
* and since `S[19:16]` is 4 bits, `0xFFFF - S[19:16] == {12'hFFF, ~S[19:16]}`, so
  the add-and-compare collapses again into a pattern match with **no carry
  propagation at all**: `S[15:4] == 0xFFF && S[3:0] == ~S[19:16]`

One 20-bit add feeding two LUT levels. The regression sweeps every bit of the
checksum field to confirm the two forms are identical.

**2. The byte counter.** Driving `tkeep`/`tlast` from a 16-bit "bytes remaining"
counter puts three 16-bit comparators on the output path, and — because the
counter is updated from those same comparisons — closes a 16-bit combinational
loop back into its own reset pin. Instead the entire beat schedule (6, then 8s,
then a remainder) is computed once in cycle 5, leaving a beat counter that only
needs a decrement and an equality test.

**3. The statistics counters.** The priority encoder choosing which counter to
bump, plus the clock enables of fourteen 32-bit counters, sat directly behind the
checksum compare — a stats register was literally the endpoint of the critical
path. Counters are diagnostics, so the verdict is now retired one cycle later.

Progression on the Zynq-7020 −1: 103.7 → 114.6 → 140.6 → **169.8 MHz**.

---

## Interfaces

`udp_rx_top` takes raw XGMII in and produces payload out.

**`tkeep`** — the first beat of a packet carries 6 bytes (`0x3F`), every beat
after it carries 8, and the final beat carries the remainder. This bends the
usual "only the last beat is partial" convention, and it is what buys the cycle.

**`tuser`** — asserted with `tlast` to mean "this packet is damaged, discard it".
Raised when the frame ends before the UDP length said it would, or when an XGMII
error character lands mid-frame. On an abort the final beat may have `tkeep=0`;
it exists purely to release the downstream sink.

**`tready`** — accepted and ignored. There is no backpressure and there cannot
be: the wire does not stop. If it is ever low while the parser drives `tvalid`,
`stat_backpressure` counts it, so a consumer that cannot keep up shows up as a
number rather than as silent corruption. Put an `axis_fifo` downstream if the
consumer can stall — outside the parser, because it costs latency.

**`cfg_port_filter_en` / `cfg_dst_port`** — feed selection. Exchanges put each
multicast feed on its own UDP port, and the port is known in cycle 5, so
filtering here costs zero cycles.

### Counters

`stat_frames`, `stat_accepted`, and one counter per rejection reason:
`ethertype`, `vlan`, `ihl`, `frag`, `proto`, `cksum`, `len`, `port`, plus
`runt` (frame ended inside the headers), `abort` (frame ended inside the
payload), `xgmii_err`, and `backpressure`. Exactly one moves per frame.

---

## Deliberately out of scope

* **VLAN tags** are detected and counted, not parsed. Supporting single-tag VLAN
  moves every offset by 4 — and since 46 mod 8 == 6, the first beat would carry
  2 bytes instead of 6. It is a second set of fixed slices and a mux, no extra
  cycles, but it doubles the header wiring.
* **IPv4 options** (IHL > 5) are dropped. They make the payload offset variable,
  which needs a real barrel shifter and costs a cycle. Exchange feeds do not use
  them.
* **Fragments** are dropped. The bytes after the IP header of a fragment are not
  a UDP header.
* **The Ethernet FCS is not checked.** A CRC-32 cannot be validated until the end
  of the frame, so any cut-through receiver must forward first and retract via
  `tuser` — that is what the abort path is for, and wiring an FCS check into it
  is the obvious next step.
* **START is only detected in lane 0.** Real XGMII also allows lane 4.

---

## Files

```
rtl/
  udp_rx_parser.v   the fused parser - all of the design is here
  udp_rx_top.v      thin top level, sideband and counters
  axis_fifo.v       optional elasticity buffer, not in the latency path

  xgmii_rx.v        \
  eth_rx.v           |  v1, the original four-stage chain. Superseded, kept
  ipv4_rx.v          |  so the 9-cycle vs 6-cycle comparison is reproducible.
  udp_rx.v           |  See HOW_IT_WORKS.md and "What changed" below.
  udp_stack_top.v   /

tb/
  tb_udp_rx.v       self-checking regression for v2 (44 checks)
  tb_udp_stack.v    the original single-packet demo for v1

sim/
  run_sim.tcl       xsim build and run
  synth_check.tcl   out-of-context synth/place/route + timing report
```

Running it:

```bash
xvlog ../rtl/udp_rx_parser.v ../rtl/udp_rx_top.v ../tb/tb_udp_rx.v && xelab tb_udp_rx -s sim && xsim sim -runall
```

---

## What changed from v1

The original design was a four-stage chain: `xgmii_rx → eth_rx → ipv4_rx →
udp_rx`. It worked, but each stage waited for its own header, realigned the byte
stream through a residue register, and then registered its output — and that
last step re-serialised the stream so the next stage could not start on its own
header until a cycle later. The 9 cycles broke down as 1 (XGMII register) + 2
(Ethernet header) + 1 (Ethernet output register) + 3 (IPv4 header) + 1 (IPv4
output register) + 1 (UDP header). The header *decode* never needed those
cycles: all three headers are at fixed, known offsets.

Two real bugs were fixed along the way.

**Ethernet padding leaked into the payload.** v1 decided where a packet ended by
watching for XGMII TERM rather than reading the UDP length, so the padding that
every NIC and switch appends to short frames was forwarded as payload. Measured
on the v1 chain: a 10-byte payload in a correctly padded 60-byte frame comes out
as **18 bytes**. The original testbench never caught it because it hand-built a
52-byte frame and never padded it.

**`axis_fifo` could not be synthesised.** `wr_ptr` was incremented in one always
block and reset in another. Two procedural blocks driving one reg is a
multiply-driven net — a simulation race that synthesis rejects outright. That is
why the module had never been instantiated anywhere.

Also corrected: `tb_udp_stack.v` anchored its latency measurement on an idle
cycle before the preamble, counting two cycles that are not parse latency and
reporting 11 for a 9-cycle design.
