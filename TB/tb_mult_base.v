//=============================================================================
// 文件名 : tb_mult_base.v
// 功能   : 给 mult_base(32x32 移位相加基乘单元)做自检
//          1) 定向向量: 0、1、最大值、2 的幂、交替模式这些边界
//          2) 随机向量: 2000 组, 用 $random 生成
//          3) 查周期数: 数 busy 持续多少拍, 看是不是 W+1 = 33
//          对答案用 Verilog 自带的 * 运算符, 跟 DUT 的结构完全独立
//
// 怎么跑 : vsim -c work.tb_mult_base -do "run -all; quit -f"
//=============================================================================
`timescale 1ns/1ps

module tb_mult_base;

    localparam integer W = 32;

    reg           clk;
    reg           rst_n;
    reg           start;
    reg  [W-1:0]  a;
    reg  [W-1:0]  b;
    wire          busy;
    wire          done;
    wire          result_rdy;
    reg           ack;
    wire [2*W-1:0] p;

    integer errors;
    integer tests;
    integer cycles;
    integer i;
    reg [2*W-1:0] gold;

    //--------------------------------------------------------------
    // DUT
    //--------------------------------------------------------------
    mult_base #(.W(W)) dut (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (start),
        .a          (a),
        .b          (b),
        .busy       (busy),
        .done       (done),
        .result_rdy (result_rdy),
        .ack        (ack),
        .p          (p)
    );

    //--------------------------------------------------------------
    // 时钟, 周期 10ns
    //--------------------------------------------------------------
    initial clk = 1'b0;
    always #5 clk = ~clk;

    //--------------------------------------------------------------
    // 答一次手: result_rdy 有效就给一个单拍的 ack, 让基单元回 IDLE
    //--------------------------------------------------------------
    reg ack_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ack   <= 1'b0;
            ack_d <= 1'b0;
        end else begin
            ack   <= result_rdy && !ack_d;
            ack_d <= result_rdy;
        end
    end

    //--------------------------------------------------------------
    // 数 busy 持续了多少拍
    //--------------------------------------------------------------
    always @(posedge clk) if (busy) cycles = cycles + 1;

    //--------------------------------------------------------------
    // 比一个用例
    //--------------------------------------------------------------
    task run_case;
        input [W-1:0] ta;
        input [W-1:0] tb;
        begin
            a = ta; b = tb;
            gold = ta * tb;

            start = 1'b1;
            @(posedge clk);
            start = 1'b0;
            @(posedge clk);          // DUT 这时候已经采到 start 了

            cycles = 0;
            while (!done) @(posedge clk);

            tests = tests + 1;
            if (p !== gold) begin
                errors = errors + 1;
                $display("[FAIL] a=%h b=%h  期望=%h 实际=%h",
                         ta, tb, gold, p);
            end
            else if (cycles !== W+1) begin   // 没有 ack 的版本: busy 持续 W+1 拍
                errors = errors + 1;
                $display("[FAIL] a=%h b=%h  结果对但 busy 周期数=%0d, 期望 %0d",
                         ta, tb, cycles, W);
            end

            @(posedge clk);          // 等 done 落回去, 好做下一个
        end
    endtask

    //--------------------------------------------------------------
    // 主流程
    //--------------------------------------------------------------
    initial begin
        errors = 0;
        tests  = 0;
        cycles = 0;
        clk    = 1'b0;
        start  = 1'b0;
        rst_n  = 1'b0;
        a      = {W{1'b0}};
        b      = {W{1'b0}};
        ack_d  = 1'b0;

        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);

        $display("=================================================");
        $display(" tb_mult_base : %0d 位 x %0d 位 移位相加基乘单元自检", W, W);
        $display("=================================================");

        //---------------- 定向向量 ----------------
        $display("--- 定向向量 ---");
        run_case(32'h0000_0000, 32'h0000_0000);   // 0*0
        run_case(32'h0000_0000, 32'hFFFF_FFFF);   // 0*最大
        run_case(32'h0000_0001, 32'h0000_0001);
        run_case(32'h0000_0001, 32'hFFFF_FFFF);
        run_case(32'hFFFF_FFFF, 32'hFFFF_FFFF);   // 最大*最大
        run_case(32'h8000_0000, 32'h8000_0000);   // 2^31 * 2^31
        run_case(32'hFFFF_FFFF, 32'h0000_0002);
        run_case(32'hAAAA_AAAA, 32'h5555_5555);   // 交替模式
        run_case(32'h5555_5555, 32'hAAAA_AAAA);

        // 每一位单独置 1 走一遍
        for (i = 0; i < W; i = i + 1) begin
            run_case(32'h1 << i, 32'h1 << (W - 1 - i));
        end

        //---------------- 随机向量 ----------------
        $display("--- 随机向量 2000 组 ---");
        for (i = 0; i < 2000; i = i + 1) begin
            run_case($random, $random);
        end

        //---------------- 汇总 ----------------
        $display("=================================================");
        $display(" TOTAL = %0d   PASS = %0d   FAIL = %0d", tests, tests - errors, errors);
        if (errors == 0) $display(" RESULT: *** ALL TESTS PASSED ***");
        else             $display(" RESULT: *** %0d TEST(S) FAILED ***", errors);
        $display("=================================================");
        $finish;
    end

    // 防止跑不完挂死
    initial begin
        #20_000_000;
        $display("[TIMEOUT] 仿真超时退出");
        $finish;
    end

endmodule
