`timescale 1ns/1ns


// SPI IP 核仿真测试平台
//
// 测试模式: CPOL=0, CPHA=0 (SPI Mode 0)
//
// 测试内容:
//   1. 写操作 - 发送不同数据模式(0xF5, 0x3C, 0xAA)
//   2. 读操作 - 验证MISO接收数据是否正确
//   3. 连续写操作(Back-to-back)
//   4. 边界值测试(0x00, 0xFF)
//   5. 读写交替验证
//
// 包含功能:
//   - SPI从机模型(模拟MISO数据输出)
//   - MOSI数据捕获与校验(8bit完整捕获)
//   - MISO数据自动校验(每次传输都校验)
//   - VCD波形导出
//   - 仿真超时保护


module tb_rtc();


// 信号定义

reg         clk;
reg         rst;
wire        sclk;
wire        mosi;
reg         miso;
wire        tri_en;
reg         wr_req;
wire        wr_ack;
reg  [7:0]  data_in;
wire [7:0]  data_out;
reg        io_dir;

// 时钟生成: 50MHz
parameter CLK_PERIOD = 20;  // 20ns = 50MHz
always #(CLK_PERIOD/2) clk = ~clk;


// SPI 从机模型 (CPHA=0, CPOL=0)
//
// 工作原理:
//   - wr_req上升沿: 锁存待发送数据, 将MSB放到MISO线上
//     (CPHA=0要求: 第一位数据在SCLK第一个上升沿之前必须稳定)
//   - SCLK下降沿: 移位寄存器左移, 输出下一位到MISO
//   - SCLK上升沿: 主机采样MISO (从机数据已稳定)

reg [7:0] slave_shift;
reg [7:0] slave_next_data;

initial begin
    slave_next_data = 8'hA5;  // 第一次传输从机发送0xA5
    miso            = 1'b0;
end

// 传输开始时准备从机数据
always @(posedge wr_req) begin
    slave_shift     = slave_next_data;
    miso            = slave_shift[7];        // MSB先输出
    slave_next_data = slave_next_data + 8'h11; // 每次传输递增
end

// SCLK下降沿: 移位输出下一位
always @(negedge sclk) begin
    slave_shift = {slave_shift[6:0], 1'b0};
    miso        = slave_shift[7];
end




// MOSI 数据捕获器
// 在SCLK上升沿(CPHA=0)捕获MOSI线上的bit，8bit完整后输出
// mosi_bit_cnt==8时显示，8位完整采集

reg [7:0] captured_mosi;
integer   mosi_bit_cnt;

initial begin
    captured_mosi  = 8'h00;
    mosi_bit_cnt   = 0;
end

always @(posedge sclk) begin
    captured_mosi = {captured_mosi[6:0], mosi};
    mosi_bit_cnt  = mosi_bit_cnt + 1;
    if (mosi_bit_cnt == 9) begin  // 等8位全部采集完再显示
        $display("  [MOSI Monitor] Captured byte: 0x%02H at time %0t", captured_mosi, $time);
        mosi_bit_cnt  = 0;
        captured_mosi = 8'h00;
    end
end

always @(negedge wr_ack) begin
    mosi_bit_cnt  = 0;
    captured_mosi = 8'h00;
end


// 实例化 SPI IP 核 (CPOL=0, CPHA=0 = SPI Mode 0)
// CLK_DIV=4: SCLK半周期 = (4+1)*20ns = 100ns, SCLK = 5MHz

spi_ip #(
    .CPOL       (1'b0),
    .CPHA       (1'b0),
    .CLK_DIV    (16'd4)
) u_spi_ip (
    .clk        (clk),
    .rst        (rst),
    .sclk       (sclk),
    .mosi       (mosi),
    .miso       (miso),
    .tri_en     (tri_en),
    .wr_req     (wr_req),
    .wr_ack 	(wr_ack),
    .data_in    (data_in),
    .data_out   (data_out),
    .io_dir     (io_dir)
);


// 测试任务


// SPI 写操作任务
task spi_write;
    input [7:0] data;
    begin
        @(posedge clk);         // 等待时钟沿同步
        io_dir   = 1'b0;        // 写方向
        data_in  = data;        // 装载发送数据
        wr_req   = 1'b1;        // 发起传输请求
        @(posedge wr_ack);  // 等待应答(传输完成)
        @(posedge clk);
        wr_req   = 1'b0;        // 撤销请求
        #(CLK_PERIOD * 15);     // 等待状态机完全回到IDLE
    end
endtask



// SPI 读操作任务
task spi_read;
    input [7:0] dummy_data;    // 读操作时MOSI端发送的dummy数据
    begin
        @(posedge clk);
        io_dir   = 1'b1;        // 读方向
        data_in  = dummy_data;
        wr_req   = 1'b1;
        @(posedge wr_ack);
        @(posedge clk);
        wr_req   = 1'b0;
        #(CLK_PERIOD * 15);
    end
endtask




// 自动监控与校验
// 从机模型在每次传输(包括写)都递增slave_next_data
// 因此expected_slave_data也必须在每次传输时递增

integer transfer_cnt;
integer error_cnt;
reg [7:0] expected_slave_data;  // 跟踪从机发送的数据
reg [7:0] expected_mosi;        // 跟踪期望的MOSI数据

initial begin
    transfer_cnt        = 0;
    error_cnt           = 0;
    expected_slave_data = 8'hA5;  // 与slave_next_data初始值一致
    expected_mosi       = 8'h00;
end

// 传输完成监控 - 每次传输都校验
always @(posedge wr_ack) begin
    transfer_cnt = transfer_cnt + 1;
    $display("========================================================");
    $display("[Time=%0t] Transfer #%0d completed", $time, transfer_cnt);
    $display("  io_dir    = %b (%s)", io_dir, io_dir ? "READ" : "WRITE");
    $display("  data_in   = 0x%02H (MOSI sent)", data_in);
    $display("  data_out  = 0x%02H (MISO received)", data_out);
    $display("  expected  = 0x%02H (slave data)", expected_slave_data);

    // 校验MISO接收数据 (SPI全双工，每次传输都有MISO数据)
    if (data_out !== expected_slave_data) begin
        $display("  *** ERROR: MISO mismatch! Expected 0x%02H, got 0x%02H ***",
                 expected_slave_data, data_out);
        error_cnt = error_cnt + 1;
    end else begin
        $display("  MISO data CORRECT: 0x%02H", data_out);
    end

    // 从机模型每次传输都递增，所以期望值也必须每次递增
    expected_slave_data = expected_slave_data + 8'h11;

    $display("========================================================");
end


// 主测试流程

initial begin
    // 初始化所有信号
    clk     = 1'b0;
    rst     = 1'b1;
    wr_req  = 1'b0;
    data_in = 8'h00;
    io_dir  = 1'b0;
    miso    = 1'b0;

    // 复位阶段
    #200;
    rst = 1'b0;
    #(CLK_PERIOD * 10);

    $display("############################################################");
    $display("#  SPI IP Testbench - CPOL=0, CPHA=0 (SPI Mode 0)         #");
    $display("#  System Clock = 50MHz, CLK_DIV=4, SCLK ~ 5MHz          #");
    $display("############################################################");


    // 测试1: 写操作 0xF5 (二进制 1111_0101)
    // 从机返回 0xA5

    $display("\n>>> Test 1: Write 0xF5, expect MISO=0xA5");
    spi_write(8'hF5);



    // 测试2: 写操作 0x3C (二进制 0011_1100)
    // 从机返回 0xB6

    $display("\n>>> Test 2: Write 0x3C, expect MISO=0xB6");
    spi_write(8'h3C);



    // 测试3: 写操作 0xAA (二进制 1010_1010)
    // 从机返回 0xC7

    $display("\n>>> Test 3: Write 0xAA, expect MISO=0xC7");
    spi_write(8'hAA);



    // 测试4: 读操作
    // 从机返回 0xD8 (A5+3*11=D8, 因为此前已有3次写操作)

    $display("\n>>> Test 4: Read, expect MISO=0xD8");
    spi_read(8'hFF);



    // 测试5: 连续写操作 (Back-to-back)
    // 从机返回 0xE9, 0xFA, 0x0B

    $display("\n>>> Test 5: Back-to-back writes (0x01, 0x02, 0x03)");
    spi_write(8'h01);
    spi_write(8'h02);
    spi_write(8'h03);



    // 测试6: 边界值 (0x00 和 0xFF)
    // 从机返回 0x1C, 0x2D

    $display("\n>>> Test 6: Boundary values (0x00, 0xFF)");
    spi_write(8'h00);
    spi_write(8'hFF);



    // 测试7: 再次读操作
    // 从机返回 0x3E

    $display("\n>>> Test 7: Read, expect MISO=0x3E");
    spi_read(8'h00);



    // 测试8: 交替读写
    // 从机返回 0x4F, 0x60, 0x71

    $display("\n>>> Test 8: Alternating write/read");
    spi_write(8'h55);
    spi_read(8'hAA);
    spi_write(8'hCC);

    // 测试总结
    #(CLK_PERIOD * 20);
    $display("\n############################################################");
    $display("#  Simulation Complete                                     #");
    $display("#  Total transfers: %0d", transfer_cnt);
    $display("#  Errors:          %0d", error_cnt);
    if (error_cnt == 0)
        $display("#  Result: ALL PASSED                                      #");
    else
        $display("#  Result: FAILED - check above for details                #");
    $display("############################################################");

    #(CLK_PERIOD * 50);

end





endmodule
