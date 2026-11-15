#=============================================================================
# signoff.sdc -- ASIC (OpenLane2 / sky130A) 签核用的时序约束
#
# 时钟周期: 52 ns (19.23 MHz)
# 和 pnr.sdc 一样。签核这步用它重新跑一遍后布线 STA。
#
# 跑出来的结果:
#   Setup WNS = +1.884 ns   违例 0 个
#   Hold  WNS = +0.064 ns   违例 0 个
#=============================================================================

create_clock -name clk -period 52 [get_ports clk]

set_input_delay 0.0 -clock [get_clocks clk] \
    [get_ports {a[*] b[*] start}]

set_output_delay 0.0 -clock [get_clocks clk] \
    [get_ports {y[*] busy done}]

set_false_path -from [get_ports rst_n]
