#=============================================================================
# pnr.sdc -- ASIC (OpenLane2 / sky130A) 布局布线的时序约束
#
# 时钟周期: 52 ns (19.23 MHz)
#
# 为什么取 52 ns:
#   1) 本设计是面积优先: 只用一个 34x34 基乘单元和一个 261 位加减器,
#      让它们轮流用。最长的路径就是 261 位加法器的进位链。
#   2) sky130 慢速角 (ss_100C_1v60) 下, 加了三级流水之后, 实测最长
#      路径约 49.2 ns (起点 op_b_reg, 终点 sum_pipe)。所以周期取
#      52 ns, 留 3 ns 左右余量。
#   3) 跑完的结果: Setup WNS = +1.884 ns, 违例 0 个。
#
# 约束了什么:
#   - 输入 a[127:0] / b[127:0] / start, 相对时钟延迟 0
#   - 输出 y[255:0] / busy / done, 相对时钟延迟 0
#   - rst_n 是异步复位, 设成 false path
#=============================================================================

create_clock -name clk -period 52 [get_ports clk]

set_input_delay 0.0 -clock [get_clocks clk] \
    [get_ports {a[*] b[*] start}]

set_output_delay 0.0 -clock [get_clocks clk] \
    [get_ports {y[*] busy done}]

set_false_path -from [get_ports rst_n]
