`timescale  1ns/1ns  

//==========================================================================
//  Testbench: 测试 UART 回环 + KEY3主动发送"0-7"
//
//  测试流程：
//    1. 上电复位 → 验证默认回环模式
//    2. UART发送"1234567" → 观察 uart_txd 回环是否完整收到"1234567"
//    3. 按KEY0进入模式1 → 按KEY3触发发送"0-7" → 观察8字节完整发送
//==========================================================================
module   tb_test();

reg         clk;
reg         rst;
reg         uart_rxd;
wire        uart_txd;
reg  [3:0]  key;
parameter   MAX_COUNT   = 20;          // 50MHz clock (20ns period)
parameter   BIT_PERIOD  = 8680;        // 115200bps @ 50MHz: 50000000/115200 ≈ 434 clks
                                        // Use real: 8680ns ≈ 434*20ns

// 按键按下再释放 (Rising edge detection: 按下=低，释放=高)
task key_press_release;
    input [3:0] key_mask;
    begin
        key = ~key_mask;                // 按下(低电平)
        #100;                           // 保持100ns > 消抖MAX_CNT=5*20ns=100ns
        key = 4'b1111;                  // 释放(高电平，产生上升沿)
        #100;
    end
endtask

// UART 发送一个字节 (LSB first, no parity, 1 stop bit)
task uart_send_byte;
    input [7:0] data;
    integer i;
    begin
        uart_rxd = 1'b0;                // 起始位
        #BIT_PERIOD;
        for (i = 0; i < 8; i = i + 1) begin
            uart_rxd = data[i];         // LSB first
            #BIT_PERIOD;
        end
        uart_rxd = 1'b1;                // 停止位
        #BIT_PERIOD;
    end
endtask

always #(MAX_COUNT/2) clk = ~clk;       // 10ns half period → 20ns clock

initial begin
    clk       = 1'b0;
    rst       = 1'b1;
    key       = 4'b1111;
    uart_rxd  = 1'b1;

    // 1. 复位释放
    #200
    rst = 1'b0;
    #2000;

    //========================================================================
    // 测试1: UART回环测试 — 发送 "1234567" 到 FPGA，观察回环结果
    //========================================================================
    $display("==========================================");
    $display(" Test 1: UART Echo (Loopback)");
    $display("   Sending: 0x31 0x32 0x33 0x34 0x35 0x36 0x37");
    $display("   Expected on TXD: same 7 bytes back");
    $display("==========================================");

    uart_send_byte(8'h31);  // '1'
    // 等一段时间让回环字节开始发送后再发下一个
    #(BIT_PERIOD * 11);     // 每个UART帧约10个位周期，留余量
    uart_send_byte(8'h32);  // '2'
    #(BIT_PERIOD * 11);
    uart_send_byte(8'h33);  // '3'
    #(BIT_PERIOD * 11);
    uart_send_byte(8'h34);  // '4'
    #(BIT_PERIOD * 11);
    uart_send_byte(8'h35);  // '5'
    #(BIT_PERIOD * 11);
    uart_send_byte(8'h36);  // '6'
    #(BIT_PERIOD * 11);
    uart_send_byte(8'h37);  // '7'

    #(BIT_PERIOD * 15);     // 等最后字节回环完成
    #5000;

    //========================================================================
    // 测试2: 按KEY0进入模式1 → 按KEY3发送"0-7"
    //========================================================================
    $display("==========================================");
    $display(" Test 2: KEY0 → Mode1 → KEY3 → Send 0-7");
    $display("   Expected on TXD: 0x30 0x31 0x32 0x33 0x34 0x35 0x36 0x37");
    $display("==========================================");

    // 按KEY0进入模式1
    key_press_release(4'b0001); // KEY0
    #5000;

    // 按KEY3触发发送"0-7"
    key_press_release(4'b1000); // KEY3
    #(BIT_PERIOD * 90);        // 等8个字节全部发送完成 (8*10+余量)

    //========================================================================
    // 测试3: 再次按KEY3发送（验证可以重复触发）
    //========================================================================
    key_press_release(4'b1000); // KEY3 again
    #(BIT_PERIOD * 90);

    #10000;
    $display("==========================================");
    $display(" Simulation finished.");
    $display(" Check waveforms for:");
    $display("  - Test1: uart_txd outputs 0x31 0x32 0x33 0x34 0x35 0x36 0x37");
    $display("  - Test2: uart_txd outputs 0x30 0x31 0x32 0x33 0x34 0x35 0x36 0x37");
    $display("==========================================");
    $stop;
end

top_uart_system #(
    .CLOCK_FREQ   ( 50_000_000    ),
    .UART_BPS     ( 115_200       ),
    .MAX_CNT      ( 5             ),   // 消抖计数(仿真加速)
    .TIME_40MS    ( 5             ),   // 仿真加速
    .TIME_100MS   ( 10            ),   // 仿真加速
    .PWM_ARR      ( 10            ),   // 仿真加速
    .MAX_count    ( 5             ),   // 仿真加速
    .CHECK_BIT    ( "None"        )
) u_top_uart_system (
    .clk      (clk      ),
    .rst      (rst      ),
    .key      (key      ),
    .uart_rxd (uart_rxd ),
    .uart_txd (uart_txd )
);

endmodule