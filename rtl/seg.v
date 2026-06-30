/*
数码管动态显示模块

*/
module seg #(
    parameter MAX_count = 16'd50000  // 计数值为50000*20ns=1ms, 动态扫描8个数码管 频率为f=1/(1ms*8)约为125Hz
)(
    input   wire            clk     ,
    input   wire            rst     ,
    input   wire    [39:0]  data_in ,  // 8个数码管 ×5位 = 40位(39:0)
    input   wire    [7:0]   dp_in   ,
    output  reg     [7:0]   seg     ,  // 段选，低电平有效
    output  reg     [7:0]   sel        // 位选，低电平有效
);

// seg code 段选,低电平点亮
// 编码表扩展：支持0-9数字、A-F十六进制、以及H、U、n等特殊字符
localparam [7:0] DIGIT0  = 8'hC0,  // 0  -> 段码: 1100_0000
                 DIGIT1  = 8'hF9,  // 1  -> 段码: 1111_1001
                 DIGIT2  = 8'hA4,  // 2  -> 段码: 1010_0100
                 DIGIT3  = 8'hB0,  // 3  -> 段码: 1011_0000
                 DIGIT4  = 8'h99,  // 4  -> 段码: 1001_1001
                 DIGIT5  = 8'h92,  // 5  -> 段码: 1001_0010
                 DIGIT6  = 8'h82,  // 6  -> 段码: 1000_0010
                 DIGIT7  = 8'hF8,  // 7  -> 段码: 1111_1000
                 DIGIT8  = 8'h80,  // 8  -> 段码: 1000_0000
                 DIGIT9  = 8'h90,  // 9  -> 段码: 1001_0000
                 DIGIT10 = 8'h88,  // A  -> 段码: 1000_1000
                 DIGIT11 = 8'h83,  // b  -> 段码: 1000_0011
                 DIGIT12 = 8'hC6,  // C  -> 段码: 1100_0110
                 DIGIT13 = 8'hA1,  // d  -> 段码: 1010_0001
                 DIGIT14 = 8'h86,  // E  -> 段码: 1000_0110
                 DIGIT15 = 8'h8E,  // F  -> 段码: 1000_1110
                 DIGIT16 = 8'h89,  // H  -> 段码: 1000_1001
                 DIGIT17 = 8'hC1,  // U  -> 段码: 1100_0001
                 DIGIT18 = 8'hFF,  // 熄灭 -> 段码: 1111_1111
                 DIGIT19 = 8'hC8,  // n  -> 段码: 1100_1000 
                 DIGIT20 = 8'hBF;  // - 横杠 -> 段码: 1011_1111

reg         [19:0]  count ;
reg         [3:0]   bits  ;

// ========== 显示数据锁存 ==========
// 在扫描周期开始时锁存一次显示数据，防止扫描过程中数据变化导致显示闪烁或错位
reg [39:0] data_in_r;
reg [7:0]  dp_in_r;
wire       scan_update;  // 扫描更新标志

assign scan_update = (count == MAX_count - 1);

always @(posedge clk or posedge rst) begin
    if (rst) begin
        data_in_r <= {5'd18,5'd18,5'd18,5'd18,5'd18,5'd18,5'd18,5'd18};  // 全黑
        dp_in_r   <= 8'b1111_1111;
    end else if (scan_update) begin
        // 在每个扫描周期结束时锁存数据，下一个周期使用
        data_in_r <= data_in;
        dp_in_r   <= dp_in;
    end
end

// 5位抽取：从40位锁存数据中提取当前扫描位的5位数据
// data_in_r格式: [第7位(5bit), 第6位(5bit), ..., 第0位(5bit)]
// 使用Verilog的+：运算符（部分选择），从高位到低位依次抽取
// 当bits=0时，抽取第7位(最高位); bits=7时，抽取第0位(最低位)
wire [4:0] data_new;
assign data_new = data_in_r[5*(8-1-bits) +: 5];

// 1ms counter
always @(posedge clk or posedge rst) begin
    if(rst)
        count <= 16'd0;
    else if(count == MAX_count - 1)
        count <= 16'd0;
    else
        count <= count + 16'd1;
end

// dynamic display
always @(posedge clk or posedge rst) begin
    if(rst) begin
        seg  <= 8'b1111_1111;
        sel  <= 8'b1111_1111;
        bits <= 4'd0;
    end
    else if(count == MAX_count - 1) begin
        if(bits == 4'd7)
            bits <= 4'd0;
        else
            bits <= bits + 4'd1;

        // 5位case，覆盖0~18
        case(data_new)
            5'd0   : seg <= DIGIT0;   // 显示 0
            5'd1   : seg <= DIGIT1;   // 显示 1
            5'd2   : seg <= DIGIT2;   // 显示 2
            5'd3   : seg <= DIGIT3;   // 显示 3
            5'd4   : seg <= DIGIT4;   // 显示 4
            5'd5   : seg <= DIGIT5;   // 显示 5
            5'd6   : seg <= DIGIT6;   // 显示 6
            5'd7   : seg <= DIGIT7;   // 显示 7
            5'd8   : seg <= DIGIT8;   // 显示 8
            5'd9   : seg <= DIGIT9;   // 显示 9
            5'd10  : seg <= DIGIT10;  // 显示 A
            5'd11  : seg <= DIGIT11;  // 显示 b
            5'd12  : seg <= DIGIT12;  // 显示 C
            5'd13  : seg <= DIGIT13;  // 显示 d
            5'd14  : seg <= DIGIT14;  // 显示 E
            5'd15  : seg <= DIGIT15;  // 显示 F
            5'd16  : seg <= DIGIT16;  // 显示 H
            5'd17  : seg <= DIGIT17;  // 显示 U
            5'd18  : seg <= DIGIT18;  // 熄灭
            5'd19  : seg <= DIGIT19;  // 显示 n
            5'd20  : seg <= DIGIT20;  // 显示 -
            default: seg <= DIGIT18;  // 默认熄灭
        endcase
        // 位选：低电平有效，每次只选中一位数码管
        sel <= ~(8'b0000_0001 << bits);
        // 小数点控制：dp_in_r[7-bits]对应当前位的小数点
        seg[7] <= dp_in_r[8-1-bits];
    end
end

endmodule
