//=============================================================================
// 文件名 : mult_base.v
// 功能   : W 位 x W 位 移位相加乘法器 (没有 ack 的版本)
//
// 接口约定 (很简单, 不用 ack):
//   - start 来一个上升沿 -> 重新装 a/b, 从零开始算
//   - 算的时候 busy=1。第 W+1 拍出结果: done 只高 1 拍, 同一拍 p 有效
//   - 之后 p 一直保持 (result_rdy 也保持), 直到下一次 start 上升沿
//   - 上层不用给 ack。只要两次 start 之间隔够久就行 (本设计隔 35 拍)
//
//   为什么把 ack 去掉: ack 和上层的锁存信号互相依赖, 很容易自己锁死。
//   实际测出来第二次运算开始就锁到上一次的残留值。去掉 ack 后信号
//   只有一个方向, 不会绕成环。
//
// 实现: 状态机和数据通路分成两个独立的 always 块
//       (以前合在一起, 结果 p_r 和完成信号差一拍)。
// 语言: Verilog-2001, 纯 RTL。
//=============================================================================
`timescale 1ns/1ps

module mult_base #(
    parameter integer W = 32
) (
    input  wire            clk,
    input  wire            rst_n,
    input  wire            start,
    input  wire [W-1:0]    a,
    input  wire [W-1:0]    b,
    output reg             busy,
    output reg             done,
    output reg             result_rdy,
    input  wire            ack,          // 留着这个端口是为了兼容, 内部没用
    output wire [2*W-1:0]  p
);

    localparam [1:0] S_IDLE  = 2'd0,
                     S_RUN   = 2'd1,
                     S_DONE  = 2'd2;

    reg [1:0]     state, state_n;
    reg [2*W-1:0] acc;
    reg [2*W-1:0] mcand;
    reg [W-1:0]   mplier;
    reg [9:0]     cnt;
    reg [2*W-1:0] p_r;
    reg           start_r;

    wire           add_now  = mplier[0];
    wire [2*W-1:0] acc_next = add_now ? (acc + mcand) : acc;
    wire           last_it  = (state == S_RUN) && (cnt == 10'd0);
    wire           new_go   = start && !start_r;    // start 上升沿

    assign p = p_r;

    // ---- 下一步状态 ----
    always @(*) begin
        state_n = state;
        case (state)
            S_IDLE : if (new_go)  state_n = S_RUN;
            S_RUN  : if (last_it) state_n = S_DONE;
            S_DONE : if (new_go)  state_n = S_RUN;   // 可以直接重启
            default:              state_n = S_IDLE;
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) state <= S_IDLE;
        else        state <= state_n;
    end

    // ---- 数据通路 ----
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc     <= {(2*W){1'b0}};
            mcand   <= {(2*W){1'b0}};
            mplier  <= {W{1'b0}};
            cnt     <= 10'd0;
            p_r     <= {(2*W){1'b0}};
            start_r <= 1'b0;
        end else begin
            start_r <= start;
            case (state)
                S_IDLE : if (new_go) begin
                    acc    <= {(2*W){1'b0}};
                    mcand  <= {{W{1'b0}}, a};
                    mplier <= b;
                    cnt    <= W[9:0];
                end

                S_RUN : begin
                    if (last_it) p_r <= acc;          // 算完了, 把乘积存下来
                    else begin
                        acc    <= acc_next;
                        mcand  <= {mcand[2*W-2:0], 1'b0};
                        mplier <= {1'b0, mplier[W-1:1]};
                        cnt    <= cnt - 10'd1;
                    end
                end

                S_DONE : if (new_go) begin            // 重启
                    acc    <= {(2*W){1'b0}};
                    mcand  <= {{W{1'b0}}, a};
                    mplier <= b;
                    cnt    <= W[9:0];
                end

                default : ;
            endcase
        end
    end

    // ---- 握手输出 (done 只高 1 拍, 和 p_r 同一拍) ----
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy       <= 1'b0;
            done       <= 1'b0;
            result_rdy <= 1'b0;
        end else begin
            done <= (state == S_RUN) && last_it;      // 只高 1 拍
            case (state)
                S_IDLE : begin
                    busy       <= 1'b0;
                    result_rdy <= 1'b0;
                    if (new_go) busy <= 1'b1;
                end
                S_RUN  : begin
                    busy       <= 1'b1;
                    result_rdy <= 1'b0;
                end
                S_DONE : begin
                    busy       <= 1'b0;
                    result_rdy <= 1'b1;               // 一直保持, 直到下次 start
                    if (new_go) begin
                        busy       <= 1'b1;
                        result_rdy <= 1'b0;
                    end
                end
                default : begin busy <= 1'b0; result_rdy <= 1'b0; end
            endcase
        end
    end

endmodule
