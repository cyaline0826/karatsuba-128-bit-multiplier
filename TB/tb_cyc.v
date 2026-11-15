// 测一次运算要多少拍: 0x0 / 1x1 / max*max
`timescale 1ns/1ps
module tb_cyc;
    reg clk, rst_n, start;
    reg [127:0] a, b;
    wire busy, done;
    wire [255:0] y;

    karatsuba128 dut (.clk(clk),.rst_n(rst_n),.start(start),
                      .a(a),.b(b),.busy(busy),.done(done),.y(y));

    initial clk = 1'b0;
    always #5 clk = ~clk;

    integer t;
    task one;
        input [127:0] ta, tb;
        begin
            @(negedge clk); a = ta; b = tb;
            @(negedge clk); start = 1'b1;
            @(negedge clk); start = 1'b0;
            t = 0;
            while (!done && t < 5000) begin @(posedge clk); t = t + 1; end
            $display("a=%h b=%h -> cyc=%0d  y=%s", ta, tb, t, (y === (ta*tb)) ? "OK" : "FAIL");
            @(posedge clk);
        end
    endtask

    initial begin
        rst_n=0; start=0; a=0; b=0;
        repeat (4) @(posedge clk);
        rst_n=1;
        repeat (2) @(posedge clk);
        one(128'd0, 128'd0);
        one(128'd1, 128'd1);
        one({128{1'b1}}, {128{1'b1}});
        $finish;
    end
endmodule
