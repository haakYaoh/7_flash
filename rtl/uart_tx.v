
//  UART 发送模块
//  特性：
//    1. 三段式状态机，结构清晰可扩展
//    2. 时序逻辑输出 tx 信号，无组合逻辑毛刺
//    3. 数据锁存，防止发送期间外部数据变化
//    4. 参数化校验位支持：None / Odd / Even
//    5. busy 信号反馈，便于流控

module uart_tx #(
    parameter   CLOCK_FREQ = 50_000_000,
    parameter   UART_BPS   = 115200,
    parameter   MAX_1BIT   = CLOCK_FREQ/UART_BPS,
    parameter   CHECK_BIT  = "None"
)(
    input                       clk,
    input                       rst,
    input       [7:0]           uart_tx_data,  // 待发送数据
    input                       uart_tx_en,    // 发送使能
    output reg                  uart_txd,      // 串口发送输出
    output wire                 uart_tx_busy,  // 发送忙标志
    output reg                  uart_tx_accept // 握手应答：uart_tx接受数据时拉高1拍
);


//  状态定义（独热编码）

localparam  IDLE  = 5'b00001,
            START = 5'b00010,
            DATA  = 5'b00100,
            CHECK = 5'b01000,
            STOP  = 5'b10000;

reg [4:0]   cstate;
reg [4:0]   nstate;


//  波特率计数器

reg [8:0]   cnt_baud;
wire        baud_cnt_add;
wire        baud_cnt_end;

always @(posedge clk or posedge rst) begin
    if (rst)
        cnt_baud <= 9'd0;
    else if (baud_cnt_add) begin
        if (baud_cnt_end)
            cnt_baud <= 9'd0;
        else
            cnt_baud <= cnt_baud + 9'd1;
    end
end

assign baud_cnt_add = cstate != IDLE;
assign baud_cnt_end = baud_cnt_add && cnt_baud == MAX_1BIT - 9'd1;


//  数据位计数器

reg [2:0]   bit_cnt;
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
assign bit_cnt_end = bit_cnt_add && bit_cnt == bit_max - 3'd1;


//  动态位宽
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
wire IDLE_START  = (cstate == IDLE)  && uart_tx_en;
wire START_DATA  = (cstate == START) && bit_cnt_end;
wire DATA_CHECK  = (cstate == DATA)  && bit_cnt_end && CHECK_BIT != "None";
wire DATA_STOP   = (cstate == DATA)  && bit_cnt_end && CHECK_BIT == "None";
wire CHECK_STOP  = (cstate == CHECK) && bit_cnt_end;
wire STOP_IDLE   = (cstate == STOP)  && bit_cnt_end;


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
            if (START_DATA)
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
            if (STOP_IDLE)
                nstate = IDLE;
            else
                nstate = cstate;
        end
        default: nstate = IDLE;
    endcase
end


//  发送数据锁存（使能时锁存，发送期间保持不变）
reg [7:0]   tx_data_r;

always @(posedge clk or posedge rst) begin
    if (rst)
        tx_data_r <= 8'd0;
    else if (uart_tx_en)
        tx_data_r <= uart_tx_data;
end


//  校验值计算
wire check_val = (CHECK_BIT == "Odd") ? ~^tx_data_r : ^tx_data_r;


//  发送输出：时序逻辑（无毛刺，比组合逻辑更稳定）
always @(posedge clk or posedge rst) begin
    if (rst)
        uart_txd <= 1'b1;                     // 空闲高电平
    else begin
        case (cstate)
            IDLE:  uart_txd <= 1'b1;          // 空闲
            START: uart_txd <= 1'b0;          // 起始位
            DATA:  uart_txd <= tx_data_r[bit_cnt]; // 数据位，LSB first
            CHECK: uart_txd <= check_val;     // 校验位
            STOP:  uart_txd <= 1'b1;          // 停止位
            default: uart_txd <= 1'b1;
        endcase
    end
end


//  忙标志（组合逻辑，与状态同步）
assign uart_tx_busy = (cstate != IDLE);

//  握手应答信号：uart_tx 从 IDLE 进入 START 的瞬间拉高1拍
//  上游模块可用此信号确认数据已被 uart_tx 锁存并开始发送

reg tx_busy_r;  // 寄存上一拍的busy，用于检测上升沿

always @(posedge clk or posedge rst) begin
    if (rst) begin
        tx_busy_r     <= 1'b0;
        uart_tx_accept <= 1'b0;
    end else begin
        tx_busy_r     <= uart_tx_busy;
        // busy 从0→1的上升沿即表示 uart_tx 接受了新数据并开始发送
        uart_tx_accept <= ~tx_busy_r && uart_tx_busy;
    end
end

endmodule
