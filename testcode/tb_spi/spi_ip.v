/*
<可复用SPI IP核>

功能：
  - 支持CPOL/CPHA四种SPI模式配置
  - 支持时钟分频配置
  - 握手式请求/应答接口（wr_req/wr_ack）
  - 支持半双工三线模式（MOSI/MISO复用为IO），用于DS1302等器件
  - 每次传输8bit数据

接口说明：
  - wr_req  : 写请求，高有效，发起一次8bit SPI传输
  - wr_ack  : 写应答，高有效一个时钟周期，表示一次传输完成
  - data_in : 待发送的8bit数据（在wr_req有效时锁存）
  - data_out: 接收到的8bit数据（在wr_ack有效时稳定）
  - io_dir  : IO方向控制，0=输出(MOSI)，1=输入(MISO)，用于三线模式
  - tri_en  : 三态使能，0=IO作为输出驱动MOSI，1=IO高阻释放给MISO

时序：
  CPOL=0,CPHA=0: SCLK空闲低，第一个边沿采样，第二个边沿移位（DS1302使用此模式）
  CPOL=0,CPHA=1: SCLK空闲低，第一个边沿移位，第二个边沿采样
  CPOL=1,CPHA=0: SCLK空闲高，第一个边沿采样，第二个边沿移位
  CPOL=1,CPHA=1: SCLK空闲高，第一个边沿移位，第二个边沿采样
*/
module spi_ip #(
    parameter   CPOL    = 1'b0,         // 时钟极性
    parameter   CPHA    = 1'b0,         // 时钟相位
    parameter   CLK_DIV = 16'd50        // SCLK分频系数（系统时钟周期数/半SCLK周期），SCLK为1MHz
)(
    input   wire                clk,
    input   wire                rst,

    // SPI物理接口
    output  wire                sclk,       // SPI时钟
    output  wire                mosi,       // 主出从入
    input   wire                miso,       // 主入从出
    output  wire                tri_en,     // 三态使能: 0=输出MOSI, 1=高阻(MISO)

    // 用户接口
    input   wire                wr_req,     // 传输请求
    output  wire                wr_ack,     // 传输完成应答
    input   wire    [7:0]       data_in,    // 发送数据
    output  wire    [7:0]       data_out,   // 接收数据
    input   wire                io_dir      // 0=发送方向, 1=接收方向
);

// 状态定义
localparam  S_IDLE      = 3'd0,
            S_CLK_HALF  = 3'd1,   // SCLK半周期等待
            S_CLK_EDGE  = 3'd2,   // SCLK边沿翻转
            S_LAST_HALF = 3'd3,   // 最后半周期（CPHA=1时需要）
            S_ACK       = 3'd4,   // 应答
            S_ACK_WAIT  = 3'd5;   // 应答后等待一拍

// 寄存器
reg     [2:0]   cstate;
reg     [2:0]   nstate;
reg     [15:0]  clk_cnt;            // 分频计数器
reg     [4:0]   edge_cnt;           // 边沿计数器(0~15, 共8个SCLK周期)
reg             sclk_reg;           // SCLK寄存器
reg     [7:0]   mosi_shift;         // 发送移位寄存器
reg     [7:0]   miso_shift;         // 接收移位寄存器

// 输出赋值
assign sclk     = sclk_reg;
assign mosi     = mosi_shift[7];
assign data_out = miso_shift;
assign tri_en   = io_dir;           // io_dir=1时释放IO线给MISO
assign wr_ack   = (cstate == S_ACK);

// 状态机 - 状态寄存器
always @(posedge clk or posedge rst) begin
    if (rst)
        cstate <= S_IDLE;
    else
        cstate <= nstate;
end

// 状态机 - 次态逻辑
always @(*) begin
    case (cstate)
        S_IDLE:
            if (wr_req)
                nstate = S_CLK_HALF;
            else
                nstate = S_IDLE;

        S_CLK_HALF:
            if (clk_cnt == CLK_DIV - 1)
                nstate = S_CLK_EDGE;
            else
                nstate = S_CLK_HALF;

        S_CLK_EDGE:
            if (edge_cnt == 5'd15)
                nstate = S_LAST_HALF;
            else
                nstate = S_CLK_HALF;

        S_LAST_HALF:
            if (clk_cnt == CLK_DIV - 1)
                nstate = S_ACK;
            else
                nstate = S_LAST_HALF;

        S_ACK:
            nstate = S_ACK_WAIT;

        S_ACK_WAIT:
            nstate = S_IDLE;

        default:
            nstate = S_IDLE;
    endcase
end

// SCLK生成
always @(posedge clk or posedge rst) begin
    if (rst)
        sclk_reg <= CPOL;
    else if (cstate == S_IDLE)
        sclk_reg <= CPOL;
    else if (cstate == S_CLK_EDGE)
        sclk_reg <= ~sclk_reg;      // 翻转SCLK
end

// 分频计数器
always @(posedge clk or posedge rst) begin
    if (rst)
        clk_cnt <= 16'd0;
    else if (cstate == S_CLK_HALF || cstate == S_LAST_HALF)
        clk_cnt <= clk_cnt + 16'd1;
    else
        clk_cnt <= 16'd0;
end

// 边沿计数器
always @(posedge clk or posedge rst) begin
    if (rst)
        edge_cnt <= 5'd0;
    else if (cstate == S_CLK_EDGE)
        edge_cnt <= edge_cnt + 5'd1;
    else if (cstate == S_IDLE)
        edge_cnt <= 5'd0;
end

// MOSI移位寄存器 - 发送数据
// CPOL=0,CPHA=0: 在偶数边沿(0,2,4,6,8,10,12,14)之后移位
// CPOL=0,CPHA=1: 在奇数边沿(1,3,5,7,9,11,13,15)之后移位
always @(posedge clk or posedge rst) begin
    if (rst)
        mosi_shift <= 8'd0;
    else if (cstate == S_IDLE && wr_req)
        mosi_shift <= data_in;      // 锁存发送数据
    else if (cstate == S_CLK_EDGE) begin
        if (CPHA == 1'b0 && edge_cnt[0] == 1'b1)         // CPHA=0: 偶数边沿后移位
            mosi_shift <= {mosi_shift[6:0], mosi_shift[7]};
        else if (CPHA == 1'b1 && edge_cnt != 5'd0 && edge_cnt[0] == 1'b0)  // CPHA=1: 奇数边沿后移位
            mosi_shift <= {mosi_shift[6:0], mosi_shift[7]};
    end
end

// MISO移位寄存器 - 接收数据
// CPOL=0,CPHA=0: 在偶数边沿(0,2,4,6,8,10,12,14)采样
// CPOL=0,CPHA=1: 在奇数边沿(1,3,5,7,9,11,13,15)采样
always @(posedge clk or posedge rst) begin
    if (rst)
        miso_shift <= 8'd0;
    else if (cstate == S_IDLE && wr_req)
        miso_shift <= 8'h00;
    else if (cstate == S_CLK_EDGE) begin
        if (CPHA == 1'b0 && edge_cnt[0] == 1'b0)         // CPHA=0: 偶数边沿采样
            miso_shift <= {miso_shift[6:0], miso};
        else if (CPHA == 1'b1 && edge_cnt[0] == 1'b1)    // CPHA=1: 奇数边沿采样
            miso_shift <= {miso_shift[6:0], miso};
    end
end

endmodule
