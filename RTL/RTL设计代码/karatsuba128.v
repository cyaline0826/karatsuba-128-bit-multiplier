//=============================================================================
// 文件名 : karatsuba128.v
// 功能   : 128 位 Karatsuba 快速乘法器, 输出 256 位完整乘积。
//          只有一个基乘单元(32x32)和一个加减器, 让它们轮流干活,
//          由微码控制器指挥。
//
//-----------------------------------------------------------------------------
// 一、算法怎么拆(两级 Karatsuba, 128 = 32 x 2^2 正好对齐)
//-----------------------------------------------------------------------------
//  第一级 128bit -> 64bit :
//      x = a1*2^64 + a0 ,  y = b1*2^64 + b0
//      w0f = a0*b0
//      w2f = a1*b1
//      wpz = (a1+a0)*(b1+b0)
//      z1  = wpz - w0f - w2f
//      y   = w0f + (z1<<64) + (w2f<<128)
//
//  第二级 64bit -> 32bit : w0f、w2f、wpz 三路各再拆一次, 每路 3 次基乘
//      拿 w0f = a0*b0 举例, a0 = A1*2^32 + A0, b0 = B1*2^32 + B0 :
//          s0 = A0*B0 ,  s2 = A1*B1
//          M  = (A0+A1)*(B0+B1)
//          xsum = s0 + s2*2^32
//          s1   = M - xsum
//          w0f  = xsum + s1*2^32          (也就是 s0 + s1*2^32 + s2*2^64)
//      加减都交给 261 位加减器按补码算, 不用单独管符号。
//
//-----------------------------------------------------------------------------
// 二、基乘怎么调度(这部分是本模块的关键)
//-----------------------------------------------------------------------------
//  基乘要花多少拍是固定的(实测 start 之后第 33 拍 p 就稳了), 所以不用
//  动态握手, 改成纯计数器控制的调度器, 用 mul_k / mul_cnt 两个变量:
//      M_IDLE  : 等当前这次乘法做完、序号切过去
//      M_SETUP : 等一拍, 让 mul_k 换了之后操作数稳定下来
//      M_RUN   : 发一拍 start, 等基乘做完再存 base_p, 然后 mul_k 加 1
//  9 组操作数在整个过程里都不变, 基单元不会拿到"下一组"的值。
//
//  9 组操作数: k=0..2 走 w0f 路, 是 A0*B0 / A1*B1 / (A0+A1)(B0+B1)
//              k=3..5 走 w2f 路;  k=6..8 走 wpz 路
//
//-----------------------------------------------------------------------------
// 三、微码程序
//-----------------------------------------------------------------------------
//   OP_LOAD  : 存下 a/b, 同时用组合逻辑算出所有拆出来的操作数
//   OP_MMUL  : 基乘段(调度器自己把 9 次乘法做完, 状态机只等它结束)
//   OP_w0_*  : w0f 路的第二级装配
//   OP_w2_*  : w2f 路的第二级装配
//   OP_zp_*  : wpz 路的第二级装配
//   OP_z1_0  : z1_0 = wpz - w0f
//   OP_z1    : z1   = z1_0 - w2f
//   OP_y     : y = w0f + (z1<<64) + (w2f<<128), 下一拍 done
//
// 语言 : Verilog-2001。纯 RTL, 没用厂商原语。
//=============================================================================
`timescale 1ns/1ps

module karatsuba128 (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         start,
    input  wire [127:0] a,
    input  wire [127:0] b,
    output reg          busy,
    output reg          done,
    output reg  [255:0] y
);

    //==================================================================
    // 1. 存下输入, 把操作数拆开(纯组合逻辑)
    //==================================================================
    reg [127:0] a_i, b_i;

    wire [63:0] a0 = a_i[63:0],   a1 = a_i[127:64];
    wire [63:0] b0 = b_i[63:0],   b1 = b_i[127:64];

    wire [64:0] a01 = {1'b0, a1} + {1'b0, a0};      // 65 位
    wire [64:0] b01 = {1'b0, b1} + {1'b0, b0};

    // w0f 路二阶拆分 (a0 * b0)
    wire [31:0] A0 = a0[31:0],  A1 = a0[63:32];
    wire [31:0] B0 = b0[31:0],  B1 = b0[63:32];
    wire [32:0] ABs = {1'b0, A0} + {1'b0, A1};
    wire [32:0] BBs = {1'b0, B0} + {1'b0, B1};
    // w2f 路二阶拆分 (a1 * b1)
    wire [31:0] C0 = a1[31:0],  C1 = a1[63:32];
    wire [31:0] D0 = b1[31:0],  D1 = b1[63:32];
    wire [32:0] CDs = {1'b0, C0} + {1'b0, C1};
    wire [32:0] DDs = {1'b0, D0} + {1'b0, D1};
    // wpz 路二阶拆分 (a01 * b01)
    // a01/b01 是 65 位, 所以高半部分是 33 位; 两半相加最多要 34 位。
    wire [31:0] E0 = a01[31:0];
    wire [32:0] E1 = a01[64:32];
    wire [31:0] F0 = b01[31:0];
    wire [32:0] F1 = b01[64:32];
    wire [33:0] EFs = {2'b0, E0} + {1'b0, E1};
    wire [33:0] FFs = {2'b0, F0} + {1'b0, F1};

    //==================================================================
    // 2. 9 次基乘的结果放哪 + 所有中间量寄存器
    //==================================================================
    reg [67:0]  m0, m1, m2, m3, m4, m5, m6, m7, m8;   // 34x34 最大 68 位
    reg [67:0]  s0_w0, s2_w0, s0_w2, s2_w2, s0_zp, s2_zp;
    reg [67:0]  s1_w0, s1_w2, s1_zp;
    reg [99:0]  midsum_w0, midsum_w2, midsum_zp;   // 要 98 位, 多留点取 100
    reg [127:0] w0f, w2f;
    reg [129:0] wpz;
    reg [129:0] z1_0, z1;

    localparam [2:0] S_IDLE  = 3'd0,
                     S_RUN   = 3'd1,
                     S_DONE  = 3'd2,
                     S_START = 3'd3;   // 新一次运算: 先把中间量全清掉再进 S_RUN

    reg [2:0] state;
    reg       start_ff;
    // 基乘的槽位 m0..m8 和 base_done 是同一拍写的, 所以 mul_done 那一拍
    // 槽位里还是旧值。必须多等几拍再进装配段, 不然会读到上一轮的残留。
    reg [2:0] asm_wait;

    localparam [17:0] OP_LOAD   = 18'b000000000000000001,
                      OP_MMUL   = 18'b000000000000000010,
                      OP_w0_s1a = 18'b000000000000000100,
                      OP_w0_s1b = 18'b000000000000001000,
                      OP_w0_mid = 18'b000000000000010000,
                      OP_w0_f   = 18'b000000000000100000,
                      OP_w2_s1a = 18'b000000000001000000,
                      OP_w2_s1b = 18'b000000000010000000,
                      OP_w2_mid = 18'b000000000100000000,
                      OP_w2_f   = 18'b000000001000000000,
                      OP_zp_s1a = 18'b000000010000000000,
                      OP_zp_s1b = 18'b000000100000000000,
                      OP_zp_mid = 18'b000001000000000000,
                      OP_zp_f   = 18'b000010000000000000,
                      OP_z1_0   = 18'b000100000000000000,
                      OP_z1     = 18'b001000000000000000,
                      OP_NOP    = 18'b010000000000000000,
                      OP_y      = 18'b100000000000000000;

    // 操作码直接存在寄存器里, 不让组合译码链接到主状态机。
    // 后端报告里, 这条控制路径是最差时序路径的主要来源。
    reg [17:0] op_oh;

    //==========================================================================
    // 三级流水控制 (为了 ASIC 时序)
    //
    // 原来把"控制译码 + 261 位加法 + 写回选择"都压在一个时钟周期里。
    // sky130 后端实测这条路径 42.6 ns(周期 24.59 ns 时 Setup slack -16.33 ns),
    // 走法是: op_oh 寄存器 -> 组合译码 -> 261 位加法 -> 写回选择 -> 目标寄存器。
    //
    // 现在把每条微指令拆成三相(微码节奏从 1 拍一条变成 3 拍一条):
    //   PH_SEL  : 组合选出操作数, 存进 op_a_reg/op_b_reg, 同时记住目的
    //   PH_ADD  : 用寄存器里的操作数做 261 位加法, 结果存进 sum_pipe
    //   PH_WRITE: 把 sum_pipe 写到目的寄存器(寄存器到寄存器, 路径很短)
    // 功能最后的结果完全一样。
    //==========================================================================
    localparam [1:0] PH_SEL   = 2'd0,   // 选操作数, 记住目的
                     PH_ADD   = 2'd1,   // 做 261 位加法(进出都是寄存器)
                     PH_WRITE = 2'd2;   // 写回目的寄存器

    reg [1:0]    phase;          // 现在在哪一相
    reg          sum_vld;        // 流水里有没有有效结果
    reg [260:0]  sum_pipe;       // 存下来的加法结果 (UW=261)
    reg [260:0]  op_a_reg;       // 存下来的加法操作数 A (把控制位到加法器的路切断)
    reg [260:0]  op_b_reg;       // 存下来的加法操作数 B
    reg          sub_reg;        // 存下来是加还是减
    reg [17:0]   dst_sel;        // PH_SEL 记住的目的
    reg [17:0]   dst_pipe;       // PH_ADD 记住的目的

    //==================================================================
    // 3. 基乘单元 + 调度器
    //==================================================================
    localparam integer MUL_PERIOD = 36;    // 参考用的周期数, 实际靠 base_done 驱动
    localparam [1:0] M_IDLE  = 2'd0,
                     M_SETUP = 2'd1,
                     M_RUN   = 2'd2;

    reg [1:0] mstate;
    reg [3:0] mul_k;
    reg [5:0] mul_cnt;
    reg       base_done_d;

    reg  [33:0] mult_op_a, mult_op_b;   // 34 位: EFs/FFs 的进位要装得下
    wire [67:0] base_p;
    wire        base_busy, base_done, base_rdy;

    // 调度器必须只在 MMUL 阶段工作。
    // 不然复位后 mstate 就是 M_IDLE, base_start 立刻变高, 基单元会拿
    // 还没存进去的 a_i/b_i(全是 0)去做第一次乘法(实测 k=0 结果是 0)。
    // 这里直接用已经寄存好的操作码, 不再走一遍范围译码。
    // 加了流水之后还要限定在 PH_SEL: 不然写回相(phase=1)的时候
    // mstate 会被反复复位, 基乘永远走不下去。
    wire mul_en    = (state == S_RUN) && op_oh[1] && (phase == PH_SEL);

    // 开始新一次运算时把调度器复位(FSM 在 S_IDLE/S_DONE 收下 start 的那一拍)
    wire mul_rst   = (start) && ((state == S_IDLE) || (state == S_DONE));

    // M_IDLE 判断要不要启动一次基乘;
    // M_SETUP 给足一个时钟周期, 让 mul_k 换了之后操作数稳定;
    // 进 M_RUN 的那一拍发一个单拍 start, 把基乘单元启动起来。
    //
    // 不能写成 (mstate==M_SETUP) && mul_en:
    //   加了流水之后 mul_en 只在 PH_SEL 是 1, 进 M_SETUP 时 phase 已经
    //   切到 PH_ADD, mul_en=0, start 永远发不出去, 基乘永远做不完(死锁)。
    reg  base_start_r;
    wire mul_go    = (mstate == M_SETUP);
    // 基单元在 base_done 拉高的那个时钟沿同时更新 p_r。
    // 这里故意晚一拍再存, 免得槽位存到更新之前的旧 p_r。
    wire mul_latch = (mstate == M_RUN) && base_done_d;
    wire mul_done  = (mul_k >= 4'd9);

    wire base_start = base_start_r;
    wire base_ack   = 1'b0;   // 新的基单元不需要 ack

    always @(*) begin
        case (mul_k)
            4'd0 : begin mult_op_a = A0;        mult_op_b = B0;        end
            4'd1 : begin mult_op_a = A1;        mult_op_b = B1;        end
            4'd2 : begin mult_op_a = ABs; mult_op_b = BBs; end
            4'd3 : begin mult_op_a = C0;        mult_op_b = D0;        end
            4'd4 : begin mult_op_a = C1;        mult_op_b = D1;        end
            4'd5 : begin mult_op_a = CDs; mult_op_b = DDs; end
            4'd6 : begin mult_op_a = E0;        mult_op_b = F0;        end
            4'd7 : begin mult_op_a = E1;        mult_op_b = F1;        end
            4'd8 : begin mult_op_a = EFs; mult_op_b = FFs; end
            default: begin mult_op_a = 34'd0;   mult_op_b = 34'd0;     end
        endcase
    end

    mult_base #(.W(34)) u_base (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (base_start),
        .a          (mult_op_a),
        .b          (mult_op_b),
        .busy       (base_busy),
        .done       (base_done),
        .result_rdy (base_rdy),
        .ack        (base_ack),
        .p          (base_p)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // 每开始一次新乘法都要复位调度器。
            // 不然 mul_k 停在 9, mul_done 一直是真的, FSM 会直接跳过 MMUL 段
            // (操作码从 OP_MMUL 直接跳到装配起点), 第二次运算肯定出错。
            mstate  <= M_IDLE;
            mul_k   <= 4'd0;
            mul_cnt <= 6'd0;
            base_done_d  <= 1'b0;
            base_start_r <= 1'b0;
        end else if (mul_rst) begin
            mstate  <= M_IDLE;
            mul_k   <= 4'd0;
            mul_cnt <= 6'd0;
            base_done_d  <= 1'b0;
            base_start_r <= 1'b0;
        end else begin
            base_done_d <= base_done;
            case (mstate)
                M_IDLE : begin
                    base_start_r <= 1'b0;
                    if (mul_en && !mul_done) begin
                        mstate <= M_SETUP;
                    end
                end

                M_SETUP : begin
                    // 无条件进 M_RUN(修死锁的第一处)。
                    // 加了流水之后 mul_en 只在 PH_SEL 是 1, 如果这里再判一次
                    // mul_en, 会马上退回 M_IDLE, 基乘永远启动不了。
                    mstate       <= M_RUN;
                    mul_cnt      <= 6'd0;
                    base_start_r <= 1'b1;     // 只高 1 拍的 start
                end

                M_RUN : begin
                    base_start_r <= 1'b0;
                    if (mul_latch) begin
                        mul_cnt <= 6'd0;
                        mul_k   <= mul_k + 4'd1;
                        mstate  <= M_IDLE;     // 下一拍先过 M_SETUP, 再启动下一组
                    end else begin
                        mul_cnt <= mul_cnt + 6'd1;
                    end
                end

                default : mstate <= M_IDLE;
            endcase
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m0 <= 68'd0; m1 <= 68'd0; m2 <= 68'd0;
            m3 <= 68'd0; m4 <= 68'd0; m5 <= 68'd0;
            m6 <= 68'd0; m7 <= 68'd0; m8 <= 68'd0;
        end else if (mul_latch) begin
            case (mul_k)
                4'd0: m0 <= base_p;
                4'd1: m1 <= base_p;
                4'd2: m2 <= base_p;
                4'd3: m3 <= base_p;
                4'd4: m4 <= base_p;
                4'd5: m5 <= base_p;
                4'd6: m6 <= base_p;
                4'd7: m7 <= base_p;
                4'd8: m8 <= base_p;
                default: ;
            endcase
        end
    end

    //==================================================================
    // 4. 共用的宽位加减器
    //==================================================================
    localparam integer UW = 261;

    reg  [UW-1:0] add_x, add_y;
    reg           do_sub;
    wire [UW-1:0] arith_s;

    // x_m 不用了。装配段改成用各路对应的 m 槽位。
    wire [UW-1:0] x_m2   = {{(UW-68){1'b0}},  m2};   // w0f 路 M = (A0+A1)(B0+B1)
    wire [UW-1:0] x_m5   = {{(UW-68){1'b0}},  m5};   // w2f 路 M
    wire [UW-1:0] x_m8   = {{(UW-68){1'b0}},  m8};   // wpz 路 M
    wire [UW-1:0] x_s0w0 = {{(UW-68){1'b0}},  m0};
    wire [UW-1:0] x_s0w2 = {{(UW-68){1'b0}},  m3};
    wire [UW-1:0] x_s0zp = {{(UW-68){1'b0}},  m6};
    wire [UW-1:0] x_s1w0 = {{(UW-68){1'b0}},  s1_w0};
    wire [UW-1:0] x_s1w2 = {{(UW-68){1'b0}},  s1_w2};
    wire [UW-1:0] x_s1zp = {{(UW-68){1'b0}},  s1_zp};
    wire [UW-1:0] x_midw0= {{(UW-100){1'b0}},  midsum_w0};
    wire [UW-1:0] x_midw2= {{(UW-100){1'b0}},  midsum_w2};
    wire [UW-1:0] x_midzp= {{(UW-100){1'b0}},  midsum_zp};
    wire [UW-1:0] x_w0f  = {{(UW-128){1'b0}}, w0f};
    wire [UW-1:0] x_w2f  = {{(UW-128){1'b0}}, w2f};
    wire [UW-1:0] x_wpz  = {{(UW-130){1'b0}}, wpz};
    wire [UW-1:0] x_z10  = {{(UW-130){1'b0}}, z1_0};

    wire [UW-1:0] sh_s2w0_32 = {{(UW-68){1'b0}}, m1} << 32;
    wire [UW-1:0] sh_s2w2_32 = {{(UW-68){1'b0}}, m4} << 32;
    wire [UW-1:0] sh_s2zp_32 = {{(UW-68){1'b0}}, m7} << 32;
    wire [UW-1:0] sh_s2w0_64 = {{(UW-68){1'b0}}, m1} << 64;
    wire [UW-1:0] sh_s2w2_64 = {{(UW-68){1'b0}}, m4} << 64;
    wire [UW-1:0] sh_s2zp_64 = {{(UW-68){1'b0}}, m7} << 64;
    wire [UW-1:0] sh_s1w0_32 = {{(UW-68){1'b0}}, s1_w0} << 32;
    wire [UW-1:0] sh_s1w2_32 = {{(UW-68){1'b0}}, s1_w2} << 32;
    wire [UW-1:0] sh_s1zp_32 = {{(UW-68){1'b0}}, s1_zp} << 32;
    wire [UW-1:0] sh_z1     = {{(UW-130){1'b0}}, z1}    << 64;
    wire [UW-1:0] sh_w2f    = {{(UW-128){1'b0}}, w2f}   << 128;

    //==================================================================
    // 5. 操作码和微码程序
    //==================================================================
    always @(*) begin
        add_x  = {UW{1'b0}};
        add_y  = {UW{1'b0}};
        do_sub = 1'b0;
        case (1'b1)
            // ---- w0f 路: s1=M-s0; s1=s1-s2; mid=s0+(s1<<32); w0f=mid+(s2<<64) ----
            op_oh[2]  : begin add_x = x_m2;   add_y = x_s0w0; do_sub = 1'b1; end
            op_oh[3]  : begin add_x = x_s1w0; add_y = {{(UW-68){1'b0}}, m1}; do_sub = 1'b1; end
            op_oh[4]  : begin add_x = x_s0w0; add_y = sh_s1w0_32;              end
            op_oh[5]  : begin add_x = x_midw0;add_y = sh_s2w0_64;              end
            // ---- w2f 路 ----
            op_oh[6]  : begin add_x = x_m5;   add_y = x_s0w2; do_sub = 1'b1; end
            op_oh[7]  : begin add_x = x_s1w2; add_y = {{(UW-68){1'b0}}, m4}; do_sub = 1'b1; end
            op_oh[8]  : begin add_x = x_s0w2; add_y = sh_s1w2_32;              end
            op_oh[9]  : begin add_x = x_midw2;add_y = sh_s2w2_64;              end
            // ---- wpz 路 ----
            op_oh[10] : begin add_x = x_m8;   add_y = x_s0zp; do_sub = 1'b1; end
            op_oh[11] : begin add_x = x_s1zp; add_y = {{(UW-68){1'b0}}, m7}; do_sub = 1'b1; end
            op_oh[12] : begin add_x = x_s0zp; add_y = sh_s1zp_32;              end
            op_oh[13] : begin add_x = x_midzp;add_y = sh_s2zp_64;              end
            // ---- 第一级 ----
            op_oh[14] : begin add_x = x_wpz;  add_y = x_w0f;  do_sub = 1'b1; end
            op_oh[15] : begin add_x = x_z10;  add_y = x_w2f;  do_sub = 1'b1; end
            op_oh[17] : begin add_x = x_w0f;  add_y = sh_z1 + sh_w2f;         end
            default: ;
        endcase
    end

    addsub #(.WD(UW)) u_addsub (
        .op (do_sub),
        .x  (add_x),
        .y  (add_y),
        .s  (arith_s)
    );

    //==================================================================
    // 6. 主状态机
    //==================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= S_IDLE;
            op_oh    <= OP_LOAD;
            start_ff <= 1'b0;
            busy     <= 1'b0;
            done     <= 1'b0;
            y        <= 256'd0;
            a_i      <= 128'd0;
            b_i      <= 128'd0;
            s0_w0 <= 64'd0; s2_w0 <= 64'd0; s0_w2 <= 64'd0; s2_w2 <= 64'd0;
            s0_zp <= 64'd0; s2_zp <= 64'd0;
            s1_w0 <= 68'd0; s1_w2 <= 68'd0; s1_zp <= 68'd0;
            midsum_w0 <= 100'd0; midsum_w2 <= 100'd0; midsum_zp <= 100'd0;
            w0f <= 128'd0; w2f <= 128'd0; wpz <= 130'd0;
            z1_0 <= 130'd0; z1 <= 130'd0;
            asm_wait <= 3'd0;
            phase    <= PH_SEL;
            sum_vld  <= 1'b0;
            sum_pipe <= 261'd0;
            op_a_reg <= 261'd0;
            op_b_reg <= 261'd0;
            sub_reg  <= 1'b0;
            dst_sel  <= 18'd0;
            dst_pipe <= 18'd0;
        end else begin
            start_ff <= start;

            case (state)

                S_IDLE : begin
                    busy <= 1'b0;
                    done <= 1'b0;
                    if (start && !start_ff) begin
                        a_i   <= a;
                        b_i   <= b;
                        busy  <= 1'b1;
                        state <= S_START;
                    end
                end

                //----------------------------------------------
                // 新一次运算的起点: 把中间量全清掉, 免得上一轮的残留污染结果
                //----------------------------------------------
                S_START : begin
                    // 每次都要把所有中间量清零。
                    // 漏掉任何一个, 上一轮的值就会参与本轮运算, 表现是
                    // "单次算对, 批量回归大面积出错"。
                    busy  <= 1'b1;
                    done  <= 1'b0;
                     op_oh <= OP_LOAD;
                    y     <= 256'd0;
                    // m0..m8 由基乘槽位那个时序块统一驱动。
                    // 新一轮会在后面 9 次 mul_latch 里把它们全覆盖掉,
                    // 所以这里不能再赋值, 否则综合会报多重驱动。
                    // 三路中间量
                    s1_w0 <= 68'd0; s1_w2 <= 68'd0; s1_zp <= 68'd0;
                    midsum_w0 <= 100'd0; midsum_w2 <= 100'd0; midsum_zp <= 100'd0;
                    w0f <= 128'd0; w2f <= 128'd0; wpz <= 130'd0;
                    z1_0 <= 130'd0; z1 <= 130'd0;
                    asm_wait <= 3'd0;
                    phase    <= PH_SEL;
                    sum_vld  <= 1'b0;
                    // dst_pipe/dst_sel 必须清:
                    //   PH_WRITE 靠 dst_pipe[17] 判断 y 写没写完,
                    //   如果留着上一轮的 OP_y, 新一轮第一拍就会误判"已经完了"而收尾。
                    dst_pipe <= 18'd0;
                    dst_sel  <= 18'd0;
                    state <= S_RUN;
                end

                S_RUN : begin
                    busy <= 1'b1;
                    done <= 1'b0;

                    //==================================================
                    // 三级流水: 每条微指令占 3 拍
                    //   PH_SEL   : op_oh -> 操作数选择 -> op_a/b_reg (顺便记住目的)
                    //   PH_ADD   : op_a/b_reg -> 261 位加法 -> sum_pipe
                    //   PH_WRITE : sum_pipe -> 目的寄存器
                    // 控制位和加法器各占一拍, 把 op_oh 到加法器那条长组合路径切断。
                    //==================================================
                    case (phase)

                    //--------------------------------------------------
                    PH_SEL : begin
                        // (a) 载入相: 不碰加法器
                        if (op_oh[0]) begin
                            op_oh <= OP_MMUL;
                            phase <= PH_ADD;
                        end
                        // (b) 基乘相: 不碰加法器
                        else if (op_oh[1]) begin
                            phase <= PH_ADD;
                            if (mul_done) begin
                                if (asm_wait >= 3'd3) op_oh <= OP_w0_s1a;
                                else                  asm_wait <= asm_wait + 3'd1;
                            end else begin
                                asm_wait <= 3'd0;
                            end
                        end
                        // (c) 装配相: 存下操作数和目的(控制位只驱动这一相)
                        else begin
                            op_a_reg <= add_x;
                            op_b_reg <= add_y;
                            sub_reg  <= do_sub;
                            dst_sel  <= op_oh;
                            phase    <= PH_ADD;

                            // y 是最后一步, 它的写回由 PH_WRITE 收尾
                            if      (op_oh[2])  op_oh <= OP_w0_s1b;
                            else if (op_oh[3])  op_oh <= OP_w0_mid;
                            else if (op_oh[4])  op_oh <= OP_w0_f;
                            else if (op_oh[5])  op_oh <= OP_w2_s1a;
                            else if (op_oh[6])  op_oh <= OP_w2_s1b;
                            else if (op_oh[7])  op_oh <= OP_w2_mid;
                            else if (op_oh[8])  op_oh <= OP_w2_f;
                            else if (op_oh[9])  op_oh <= OP_zp_s1a;
                            else if (op_oh[10]) op_oh <= OP_zp_s1b;
                            else if (op_oh[11]) op_oh <= OP_zp_mid;
                            else if (op_oh[12]) op_oh <= OP_zp_f;
                            else if (op_oh[13]) op_oh <= OP_z1_0;
                            else if (op_oh[14]) op_oh <= OP_z1;
                            else if (op_oh[15]) op_oh <= OP_NOP;
                            else if (op_oh[16]) op_oh <= OP_y;
                            else                op_oh <= OP_y;
                        end
                    end

                    //--------------------------------------------------
                    PH_ADD : begin
                        // 加法器的输入和输出都是寄存器, 这一相只做一次加减。
                        // 载入相和基乘相不产生结果, 直接跳过。
                        if (op_oh[0] || op_oh[1]) begin
                            sum_vld <= 1'b0;
                        end else begin
                            sum_pipe <= sub_reg ? (op_a_reg - op_b_reg)
                                                : (op_a_reg + op_b_reg);
                            dst_pipe <= dst_sel;
                            sum_vld  <= 1'b1;
                        end
                        phase <= PH_WRITE;
                    end

                    //--------------------------------------------------
                    PH_WRITE : begin
                        if (sum_vld) begin
                            if (dst_pipe[2] || dst_pipe[3])      s1_w0     <= sum_pipe[67:0];
                            else if (dst_pipe[4])                midsum_w0 <= sum_pipe[99:0];
                            else if (dst_pipe[5])                w0f       <= sum_pipe[127:0];
                            else if (dst_pipe[6] || dst_pipe[7]) s1_w2     <= sum_pipe[67:0];
                            else if (dst_pipe[8])                midsum_w2 <= sum_pipe[99:0];
                            else if (dst_pipe[9])                w2f       <= sum_pipe[127:0];
                            else if (dst_pipe[10] || dst_pipe[11]) s1_zp   <= sum_pipe[67:0];
                            else if (dst_pipe[12])               midsum_zp <= sum_pipe[99:0];
                            else if (dst_pipe[13])               wpz       <= sum_pipe[129:0];
                            else if (dst_pipe[14])               z1_0      <= sum_pipe[129:0];
                            else if (dst_pipe[15])               z1        <= sum_pipe[129:0];
                            else if (dst_pipe[17])               y         <= sum_pipe[255:0];
                        end

                        phase   <= PH_SEL;
                        sum_vld <= 1'b0;

                        if (dst_pipe[17]) begin
                            done  <= 1'b1;
                            busy  <= 1'b0;
                            state <= S_DONE;
                        end
                    end

                    default : phase <= PH_SEL;
                    endcase
                end

                S_DONE : begin
                    busy <= 1'b0;
                    done <= 1'b1;
                    // 这里必须用 start 的电平判断, 不能写成 start && !start_ff。
                    // 原因: S_DONE 是个"保持"状态。测试台在 negedge 把 start 拉高,
                    // 到下一个 posedge 采样时, start_ff 已经被这一拍更新成 1 了,
                    // 边沿条件就被吃掉, 没法重新启动。
                    // 用电平判断不会重复启动: S_DONE 只消费一次 start 就转进 S_RUN,
                    // 而 S_RUN 不看 start。
                    if (start) begin
                        a_i   <= a;
                        b_i   <= b;
                        done  <= 1'b0;
                        busy  <= 1'b1;
                        state <= S_START;
                    end
                end

                default : state <= S_IDLE;
            endcase
        end
    end

endmodule
