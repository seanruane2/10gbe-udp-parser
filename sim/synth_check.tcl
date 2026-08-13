# synth_check.tcl
# Out-of-context synthesis + place + route of udp_rx_top, which reports back the
# internal reg-to-reg critical path. This is where the Fmax numbers in the README
# came from, so re-run it whenever you change anything in the parser.
#
#   vivado -mode batch -source synth_check.tcl -tclargs <part> <OUTPUT_REG>
#
# e.g.
#   vivado -mode batch -source synth_check.tcl -tclargs xc7k160tfbg484-2 0
#   vivado -mode batch -source synth_check.tcl -tclargs xc7z020clg400-1 0
#
# Why I only look at reg-to-reg: in out-of-context mode the ports don't have any
# real placement, so the clk->port and port->reg routes are meaningless numbers.
# reg-to-reg is the bit that actually describes my logic. Note that with
# OUTPUT_REG=0 the payload bus is combinational on purpose, so it hands about
# 2 ns of path over to whatever consumes it - either budget for that in the
# consuming module, or set OUTPUT_REG=1 and spend the cycle instead.

set here [file dirname [file normalize [info script]]]
set rtl  [file dirname $here]/rtl

set part   [lindex $argv 0]
set outreg [lindex $argv 1]
if {$part   eq ""} { set part   xc7k160tfbg484-2 }
if {$outreg eq ""} { set outreg 0 }

read_verilog [list $rtl/udp_rx_parser.v $rtl/udp_rx_top.v]
synth_design -top udp_rx_top -part $part -mode out_of_context \
             -generic OUTPUT_REG=$outreg

# 156.25 MHz is the 10GbE XGMII clock. 64 bits x 156.25 MHz = 10 Gbit/s.
create_clock -period 6.4 -name clk [get_ports clk]

opt_design
place_design
route_design

report_utilization -file util.rpt
report_timing -from [all_registers] -to [all_registers] \
              -max_paths 5 -nworst 1 -path_type full -file regreg.rpt

set p [get_timing_paths -from [all_registers] -to [all_registers] -max_paths 1]
puts "============================================================"
puts " part            : $part   (OUTPUT_REG=$outreg)"
puts " reg-to-reg slack: [format %.3f [get_property SLACK $p]] ns against 6.4 ns"
puts " implied Fmax    : [format %.1f [expr {1000.0/(6.4 - [get_property SLACK $p])}]] MHz"
puts " critical path   : [get_property STARTPOINT_PIN $p] -> [get_property ENDPOINT_PIN $p]"
puts " logic levels    : [get_property LOGIC_LEVELS $p]"
puts "============================================================"
