//=============================================================================
// 文件名 : karatsuba_top.v
// 功能   : 128x128 Karatsuba 乘法器的 FPGA 封装, 目的是减少 I/O 引脚数。
//
// 怎么送数据进来:
//   当 in_ready=1 时, 给一个 32 位数, 同时把 in_valid 拉高一个时钟。
//   一共送 8 个, 顺序是:
//       0: a[127:96]
//       1: a[95:64]
//       2: a[63:32]
//       3: a[31:0]
//       4: b[127:96]
//       5: b[95:64]
//       6: b[63:32]
//       7: b[31:0]
//
// 怎么取结果:
//   算完之后 out_valid 会高 8 个时钟。out_data 给的顺序是:
//       0: y[255:224]
//       1: y[223:192]
//       2: y[191:160]
//       3: y[159:128]
//       4: y[127:96]
//       5: y[95:64]
//       6: y[63:32]
//       7: y[31:0]
//
// 核心乘法器不改, 也不把它直接当 FPGA 顶层用, 因为那样会引出
// 几百个 I/O 引脚, 器件放不下。
//=============================================================================
`timescale 1ns/1ps

module karatsuba_top (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        in_valid,
    input  wire [31:0] in_data,
    output wire        in_ready,

    output wire        out_valid,
    output reg  [31:0] out_data,

    output wire        busy,
    output wire        done
);

    localparam [1:0] S_INPUT = 2'd0,
                     S_START = 2'd1,
                     S_WAIT  = 2'd2,
                     S_OUTPUT = 2'd3;

    reg [1:0] state;
    reg [3:0] in_count;
    reg [3:0] out_count;

    reg [127:0] a_reg;
    reg [127:0] b_reg;
    reg [255:0] result_reg;
    reg         core_start;

    wire        core_busy;
    wire        core_done;
    wire [255:0] core_y;

    assign in_ready  = (state == S_INPUT);
    assign out_valid = (state == S_OUTPUT);
    assign busy      = (state != S_INPUT);
    assign done      = out_valid;

    karatsuba128 u_karatsuba128 (
        .clk   (clk),
        .rst_n (rst_n),
        .start (core_start),
        .a     (a_reg),
        .b     (b_reg),
        .busy  (core_busy),
        .done  (core_done),
        .y     (core_y)
    );

    always @(*) begin
        case (out_count)
            4'd0: out_data = result_reg[255:224];
            4'd1: out_data = result_reg[223:192];
            4'd2: out_data = result_reg[191:160];
            4'd3: out_data = result_reg[159:128];
            4'd4: out_data = result_reg[127:96];
            4'd5: out_data = result_reg[95:64];
            4'd6: out_data = result_reg[63:32];
            4'd7: out_data = result_reg[31:0];
            default: out_data = 32'd0;
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_INPUT;
            in_count   <= 4'd0;
            out_count  <= 4'd0;
            a_reg      <= 128'd0;
            b_reg      <= 128'd0;
            result_reg <= 256'd0;
            core_start <= 1'b0;
        end else begin
            core_start <= 1'b0;

            case (state)
                S_INPUT: begin
                    if (in_valid) begin
                        case (in_count)
                            4'd0: a_reg[127:96] <= in_data;
                            4'd1: a_reg[95:64]  <= in_data;
                            4'd2: a_reg[63:32]  <= in_data;
                            4'd3: a_reg[31:0]   <= in_data;
                            4'd4: b_reg[127:96] <= in_data;
                            4'd5: b_reg[95:64]  <= in_data;
                            4'd6: b_reg[63:32]  <= in_data;
                            4'd7: begin
                                b_reg[31:0] <= in_data;
                                state <= S_START;
                            end
                            default: ;
                        endcase

                        if (in_count != 4'd7)
                            in_count <= in_count + 4'd1;
                        else
                            in_count <= 4'd0;
                    end
                end

                // 这个状态隔开最后一个输入字和 core_start。
                // 等到下一个时钟上升沿, a_reg/b_reg 已经稳定, 核心再采 start。
                S_START: begin
                    core_start <= 1'b1;
                    state <= S_WAIT;
                end

                S_WAIT: begin
                    // core_done 和 core_y 是核心里的寄存器输出。
                    // 这里晚一个时钟再采, 这时 core_y 才是稳的。
                    if (core_done) begin
                        result_reg <= core_y;
                        out_count <= 4'd0;
                        state <= S_OUTPUT;
                    end
                end

                S_OUTPUT: begin
                    if (out_count == 4'd7) begin
                        out_count <= 4'd0;
                        state <= S_INPUT;
                    end else begin
                        out_count <= out_count + 4'd1;
                    end
                end

                default: state <= S_INPUT;
            endcase
        end
    end

endmodule
