//=============================================================================
// 文件名 : tb_align2.v  (调时序用)
// 作用   : 单独放一个 mult_base, 逐拍打印 state/cnt/last_it/p_r/done/result_rdy,
//          把 p_r 和 result_rdy 差几拍看清楚。
//=============================================================================
`timescale 1ns/1ps

module tb_align2;

    reg        clk, rst_n, start, ack;
    reg [31:0] a, b;
    wire       busy, done, rdy;
    wire [63:0] p;

    mult_base #(.W(32)) dut (
        .clk(clk), .rst_n(rst_n), .start(start), .a(a), .b(b),
        .busy(busy), .done(done), .result_rdy(rdy), .ack(ack), .p(p)
    );

    initial clk = 1'b0;
    always #5 clk = ~clk;

    integer t;
    initial begin
        rst_n = 0; start = 0; a = 32'd3; b = 32'd5; ack = 0; t = 0;
        repeat (4) @(posedge clk);
        rst_n = 1;
        repeat (2) @(posedge clk);
        @(negedge clk); start = 1'b1;
        @(negedge clk); start = 1'b0;

        repeat (38) begin
            @(negedge clk);
            t = t + 1;
            $display("t=%0d st=%0d cnt=%0d last=%b p_r=%h busy=%b done=%b rdy=%b p=%h",
                t, dut.state, dut.cnt, dut.last_it, dut.p_r,
                busy, done, rdy, p);
        end
        $display("期望乘积 = %0d  (3*5=15=0x%h)", 32'd3*32'd5, 32'd3*32'd5);
        $finish;
    end

endmodule
