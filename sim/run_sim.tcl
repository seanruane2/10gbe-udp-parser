# run_sim.tcl
# Vivado xsim script for the UDP receive path.
#
# COMMAND LINE (run this from a scratch directory - xvlog dumps its work library
# into whatever the current directory is):
#
#   v2, the fused low-latency parser (6-cycle latency):
#     xvlog ../rtl/udp_rx_parser.v ../rtl/udp_rx_top.v ../tb/tb_udp_rx.v
#     xelab tb_udp_rx -s sim_v2
#     xsim sim_v2 -runall
#
#   v2 with a registered output boundary (7-cycle latency):
#     xvlog -d OUT_REG ../rtl/udp_rx_parser.v ../rtl/udp_rx_top.v ../tb/tb_udp_rx.v
#     xelab tb_udp_rx -s sim_v2r
#     xsim sim_v2r -runall
#
#   v1, my original four-stage chain, kept around for the latency comparison:
#     xvlog ../rtl/xgmii_rx.v ../rtl/eth_rx.v ../rtl/ipv4_rx.v ../rtl/udp_rx.v \
#           ../rtl/udp_stack_top.v ../tb/tb_udp_stack.v
#     xelab tb_udp_stack -s sim_v1
#     xsim sim_v1 -runall
#
# VIVADO GUI:
#   Add rtl/*.v as design sources and tb/tb_udp_rx.v as a simulation source,
#   set tb_udp_rx as the simulation top, then Run Behavioral Simulation.
#
# VIVADO TCL CONSOLE:
#   source {.../sim/run_sim.tcl}

set here [file dirname [file normalize [info script]]]
set root [file dirname $here]

# ---- v2: the design under test ----
set v2_files [list \
    $root/rtl/udp_rx_parser.v \
    $root/rtl/udp_rx_top.v    \
    $root/tb/tb_udp_rx.v      \
]

foreach f $v2_files { xvlog $f }
xelab -debug typical tb_udp_rx -s tb_udp_rx_sim
xsim tb_udp_rx_sim -runall

# ---- v1: the original layered chain, for the latency comparison ----
# Uncomment this lot if you want to run the old design as well.
#
# set v1_files [list \
#     $root/rtl/xgmii_rx.v      \
#     $root/rtl/eth_rx.v        \
#     $root/rtl/ipv4_rx.v       \
#     $root/rtl/udp_rx.v        \
#     $root/rtl/udp_stack_top.v \
#     $root/tb/tb_udp_stack.v   \
# ]
# foreach f $v1_files { xvlog $f }
# xelab -debug typical tb_udp_stack -s tb_udp_stack_sim
# xsim tb_udp_stack_sim -runall
