
//  UART 接收模块:
//    1. 三段式状态机（独热编码），结构清晰可扩展
//    2. 三级触发器同步，消除亚稳态
//    3. 下降沿检测 + IDLE态保护，防止接收中被误触发
//    4. 起始位中点校验，抵抗噪声引起的假起始位
//    5. 数据位中点采样，远离边沿跳变区域
//    6. 停止位提前半拍释放，提升连续接收鲁棒性
//    7. 寄存器输出（done + data），无毛刺，严格1拍脉冲
//    8. 参数化校验位支持：None / Odd / Even


module uart_rx #(
    parameter   CLOCK_FREQ = 50_000_000,       // 系统时钟频率
    parameter   UART_BPS   = 115200,           // 波特率
    parameter   MAX_1BIT   = CLOCK_FREQ/UART_BPS, // 每位时钟周期数
    parameter   CHECK_BIT  = "None"            // 校验位: "None" / "Odd" / "Even"
)(
    input                       clk,
    input                       rst,
    input                       uart_rxd,      // 串口接收输入
    output reg                  uart_rx_done,   // 接收完成脉冲（1拍）
    output reg  [7:0]           uart_rx_data    // 接收数据（寄存器输出）
);


//  状态定义（独热编码）

localparam  IDLE  = 5'b00001,
            START = 5'b00010,
            DATA  = 5'b00100,
            CHECK = 5'b01000,
            STOP  = 5'b10000;

reg [4:0]   cstate;                           // 当前状态
reg [4:0]   nstate;                           // 下一状态


//  三级寄存器同步 + 下降沿检测

reg         rx_d0;                             // 第1级：同步外部信号
reg         rx_d1;                             // 第2级：稳定值（用于采样）
reg         rx_d2;                             // 第3级：边沿检测参考

always @(posedge clk or posedge rst) begin
    if (rst) begin
        rx_d0 <= 1'b1;
        rx_d1 <= 1'b1;
        rx_d2 <= 1'b1;
    end else begin
        rx_d0 <= uart_rxd;                    // 第1级同步
        rx_d1 <= rx_d0;                       // 第2级同步
        rx_d2 <= rx_d1;                       // 第3级同步
    end
end

// 下降沿检测：仅IDLE态有效
wire rx_nege = ~rx_d1 & rx_d2 & (cstate == IDLE);


//  波特率计数器

reg [8:0]   baud_cnt;                         // 9位，最大511，足够115200bps
wire        baud_cnt_add;
wire        baud_cnt_end;

always @(posedge clk or posedge rst) begin
    if (rst)
        baud_cnt <= 9'd0;
    else if (baud_cnt_add) begin
        if (baud_cnt_end)
            baud_cnt <= 9'd0;
        else
            baud_cnt <= baud_cnt + 9'd1;
    end
end

assign baud_cnt_add = cstate != IDLE;//非空闲状态，比特率累加计数
assign baud_cnt_end = baud_cnt_add && baud_cnt == MAX_1BIT - 9'd1;


//  数据位计数器

reg [2:0]   bit_cnt;                          // 3位，0~7
reg [3:0]   bit_max;
wire        bit_cnt_add;
wire        bit_cnt_end;

always @(posedge clk or posedge rst) begin
    if (rst)
        bit_cnt <= 3'd0;
    else if (bit_cnt_add) begin
        if (bit_cnt_end)
            bit_cnt <= 3'd0;
        else
            bit_cnt <= bit_cnt + 3'd1;
    end
end

assign bit_cnt_add = baud_cnt_end;
assign bit_cnt_end = bit_cnt_add && (bit_cnt == bit_max - 3'd1);


//  动态位宽（根据状态决定当前阶段需要计多少位）

always @(*) begin
    case (cstate)
        IDLE:  bit_max = 4'd0;
        START: bit_max = 4'd1;
        DATA:  bit_max = 4'd8;
        CHECK: bit_max = 4'd1;
        STOP:  bit_max = 4'd1;
        default: bit_max = 4'd0;
    endcase
end


//  状态转移条件
wire IDLE_START  = (cstate == IDLE)  && rx_nege;                    // 检测到起始位
wire START_DATA  = (cstate == START) && bit_cnt_end;                // 起始位结束
wire DATA_CHECK  = (cstate == DATA)  && bit_cnt_end && CHECK_BIT != "None"; // 有校验
wire DATA_STOP   = (cstate == DATA)  && bit_cnt_end && CHECK_BIT == "None"; // 无校验
wire CHECK_STOP  = (cstate == CHECK) && bit_cnt_end;                // 校验位结束
wire STOP_IDLE   = (cstate == STOP)  && baud_cnt == (MAX_1BIT >> 1);// 停止位半程→提前释放

// 起始位中点校验：若中点时仍为高电平，说明是假起始位
wire start_invalid = (cstate == START) && baud_cnt == (MAX_1BIT >> 1) && rx_d1;


//  状态机：状态寄存
always @(posedge clk or posedge rst) begin
    if (rst)
        cstate <= IDLE;
    else
        cstate <= nstate;
end


//  状态机：转移逻辑

always @(*) begin
    case (cstate)
        IDLE: begin
            if (IDLE_START)
                nstate = START;
            else
                nstate = cstate;
        end
        START: begin
            if (start_invalid) // 假起始位，回到IDLE
                nstate = IDLE;
            else if (START_DATA)
                nstate = DATA;
            else
                nstate = cstate;
        end
        DATA: begin
            if (DATA_CHECK)
                nstate = CHECK;
            else if (DATA_STOP)
                nstate = STOP;
            else
                nstate = cstate;
        end
        CHECK: begin
            if (CHECK_STOP)
                nstate = STOP;
            else
                nstate = cstate;
        end
        STOP: begin
            if (STOP_IDLE)                     // 提前半拍回到IDLE
                nstate = IDLE;
            else
                nstate = cstate;
        end
        default: nstate = IDLE;
    endcase
end


//  数据位中点采样
reg [7:0]   rx_temp;

always @(posedge clk or posedge rst) begin
    if (rst)
        rx_temp <= 8'd0;
    else if (cstate == DATA && baud_cnt == (MAX_1BIT >> 1))
        rx_temp[bit_cnt] <= rx_d1;            // 用第2级同步信号采样
end


//  校验位中点采样

reg         rx_check;

always @(posedge clk or posedge rst) begin
    if (rst)
        rx_check <= 1'b0;
    else if (cstate == CHECK && baud_cnt == (MAX_1BIT >> 1))
        rx_check <= rx_d1;
end

// 校验值计算：Odd=异或取反，Even=异或
wire check_val = (CHECK_BIT == "Odd") ? ~^rx_temp : ^rx_temp;


//  输出：寄存器锁存，严格1拍完成脉冲

always @(posedge clk or posedge rst) begin
    if (rst) begin
        uart_rx_done <= 1'b0;
        uart_rx_data <= 8'd0;
    end
    else if (cstate == STOP && baud_cnt == (MAX_1BIT >> 1)) begin
        // 无校验：直接有效
        // 有校验：校验匹配才有效
        if (CHECK_BIT == "None")
            uart_rx_done <= 1'b1;
        else if (check_val == rx_check)
            uart_rx_done <= 1'b1;
        else
            uart_rx_done <= 1'b0;
        uart_rx_data <= rx_temp;
    end
    else begin
        uart_rx_done <= 1'b0;
        uart_rx_data <= uart_rx_data;   // 保持
    end
end

endmodule
