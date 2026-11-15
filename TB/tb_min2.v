//=============================================================================
// 文件名 : tb_min2.v  向量回归
// 结构和时序是照抄已经验证过的 tb_min 的:
//   复位 -> 等操作数稳定 -> 发一拍 start -> 等 done
// 操作数在 negedge 赋值, 这样 RTL 在接下来的第一个 posedge 采到的是新值。
//=============================================================================
`timescale 1ns/1ps
module tb_min2;

    reg          clk, rst_n, start;
    reg  [127:0] a, b;
    wire         busy, done;
    wire [255:0] y;

    karatsuba128 dut (.clk(clk),.rst_n(rst_n),.start(start),
                      .a(a),.b(b),.busy(busy),.done(done),.y(y));

    initial clk = 1'b0;
    always #5 clk = ~clk;

    integer t;
    integer fd, fo;
    integer total, pass, fail;
    reg [127:0] va, vb;
    reg [255:0] gold;
    integer code;

    initial begin
        rst_n=0; start=0; a=128'd0; b=128'd0; t=0;
        total=0; pass=0; fail=0;
        fo = $fopen("min2.log","w");
        fd = $fopen("sweep_in.txt","r");

        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);

        code = $fscanf(fd, "%h %h\n", va, vb);
        while (code == 2) begin
            gold = va * vb;

            // 先在 negedge 把操作数摆好(上一轮的 done 在这之前已经落下了)
            @(negedge clk);
            a = va; b = vb;

            // 再发一拍 start
            @(negedge clk); start = 1'b1;
            @(negedge clk); start = 1'b0;

            t = 0;
            while (!done && t < 4000) begin
                @(posedge clk);
                t = t + 1;
            end

            total = total + 1;
            if (y === gold) pass = pass + 1;
            else begin
                fail = fail + 1;
                if (fail <= 100000)
                    $fwrite(fo, "FAIL a=%032h b=%032h exp=%064h got=%064h cyc=%0d\n",
                            va, vb, gold, y, t);
            end

            // 等 done 落下
            @(posedge clk);

            code = $fscanf(fd, "%h %h\n", va, vb);
        end
        $fclose(fd);
        $fwrite(fo, "TOTAL=%0d PASS=%0d FAIL=%0d\n", total, pass, fail);
        $fclose(fo);
        $display("min2 done");
        $finish;
    end
endmodule
