module flash_contorl (
    input   wire            sys_clk             ,
    input   wire            rst                 ,

    input   wire            read_id_req         ,
    output  wire[23:0]      flash_id            ,
    output  wire            read_id_end         ,

    input   wire            read_req            ,
    input   wire[23:0]      read_addr           ,
    input   wire[9:0]       read_size           ,
    output  wire[7:0]       read_data           ,    
    output  wire            read_ack            ,
    output  wire            read_end            ,

    input   wire            write_enable_req    ,
    output  wire            write_enable_end    ,

    input   wire            write_req           , 
    input   wire[23:0]      write_page          ,
    input   wire[8:0]       write_size          ,

    input   wire[7:0]       write_data          ,
    output  wire            write_ack           ,
    output  wire            write_end           ,
    
    input   wire            erase_sector_req    ,
    input   wire[23:0]      erase_sector_addr   ,
    output  wire            erase_sector_end    ,

    input   wire            erase_bulk_req      ,
    output  wire            erase_bulk_end      ,

    output  wire            spi_clk             ,
    output  wire            spi_mosi            ,
    input   wire            spi_miso            ,
    output  wire            spi_csn
);

//flash command   
`define Write_Enable                    8'h06
`define Write_Disable                   8'h04
`define Read_Identification             8'h9F
`define Read_Status_Reg                 8'h05
`define Write_Status_Reg                8'h01
`define Read_Data_Bytes                 8'h03
`define Read_Data_At_Higher_Speed       8'h0B
`define Page_Program                    8'h02
`define Sector_Erase                    8'hD8
`define Bulk_Erase                      8'hC7               

`define Read_Identification_Bytes       4'd4
`define Sector_Erase_Bytes              4'd4
`define Bulk_Erase_Bytes                4'd1

//para define
localparam  Page_Size                   =   9'd256;
localparam  Write_Enable_Wait           =   'd10;
localparam  Read_Data_Bytes_Wait        =   'd10;            
localparam  Sector_Erase_Wait           =   32'd35_000_000;//700ms   
localparam  Page_Program_Wait           =   32'd350_000;      
localparam  Bulk_Erase_Wait             =   32'd650_000_000;//1s  
localparam  Flash_Idle                  =   13'b0_0000_0000_0001;
localparam  Flash_Write_Enable          =   13'b0_0000_0000_0010;
localparam  Flash_Write_Disable         =   13'b0_0000_0000_0100;
localparam  Flash_Read_Identification   =   13'b0_0000_0000_1000;
localparam  Flash_Read_Status_Reg       =   13'b0_0000_0001_0000;
localparam  Flash_Write_Status_Reg      =   13'b0_0000_0010_0000;
localparam  Flash_Read_Data_Bytes       =   13'b0_0000_0100_0000;
localparam  Flash_Read_Data_At_hSpeed   =   13'b0_0000_1000_0000;
localparam  Flash_Page_Program          =   13'b0_0001_0000_0000;
localparam  Flash_Sector_Erase          =   13'b0_0010_0000_0000;
localparam  Flash_Bulk_Erase            =   13'b0_0100_0000_0000;
localparam  Flash_Wait                  =   13'b0_1000_0000_0000;
localparam  Flash_End                   =   13'b1_0000_0000_0000;

//reg define
reg     [12:0]  state , next_state      ;
reg     [12:0]  state_ts                ;
reg                 spi_write_req           ;
reg     [7:0]       spi_write_data          ;
reg                 spi_csn_reg             ;
reg     [9:0]       spi_wr_byte_cnt         ;
reg     [23:0]      flash_id_reg            ;
reg     [32:0]      pp_erase_wait_cnt       ;
reg                 spi_read_req            ;

//新增寄存器：自动写使能标志和目标操作
reg                 auto_we;
reg     [12:0]      target_op;

//wire define
wire    [7:0]       spi_read_data;
wire                spi_read_ack;
wire                spi_write_ack;

//
assign      spi_csn             =   spi_csn_reg; 
assign      flash_id            =   flash_id_reg;
assign      read_id_end         =   ((spi_wr_byte_cnt == `Read_Identification_Bytes - 1'b1) && spi_read_ack == 1'b1) ? 1'b1 : 1'b0;
assign      read_data           =   spi_read_data;
assign      read_ack            =   ((state == Flash_Read_Data_Bytes) && (spi_wr_byte_cnt > 'd3) && spi_read_ack == 1'b1) ? 1'b1 : 1'b0;
assign      read_end            =   ((state == Flash_Read_Data_Bytes) &&(spi_wr_byte_cnt == read_size + 'd1 + 'd3 - 1'b1) && spi_read_ack == 1'b1) ? 1'b1 : 1'b0;
assign      write_enable_end    =   ((state == Flash_Write_Enable) &&(spi_wr_byte_cnt == 'd0) && spi_write_ack == 1'b1) ? 1'b1 : 1'b0;
assign      write_ack           =   ((state == Flash_Page_Program) && (spi_wr_byte_cnt > 'd3) && spi_write_ack == 1'b1) ? 1'b1 : 1'b0;
assign      write_end           =   ((state == Flash_Page_Program) && (spi_wr_byte_cnt == write_size + 'd1 + 'd3 - 1'b1) && spi_write_ack == 1'b1) ? 1'b1 : 1'b0;
assign      erase_sector_end    =   ((state == Flash_Sector_Erase) && (spi_wr_byte_cnt == `Sector_Erase_Bytes - 1'b1) && spi_write_ack == 1'b1 ) ? 1'b1 : 1'b0; 
assign      erase_bulk_end      =   ((state == Flash_Bulk_Erase) && (spi_wr_byte_cnt == `Bulk_Erase_Bytes - 1'b1)   && spi_write_ack == 1'b1 ) ? 1'b1 : 1'b0; 

//
always@(posedge sys_clk or negedge rst)begin
    if( rst == 1'b0)
        state <= Flash_Idle;
    else
        state <= next_state;
end

//
always@(*)  begin
    case (state)
        Flash_Idle: 
            if( read_id_req == 1'b1)
                next_state <= Flash_Read_Identification;
            else if( write_enable_req == 1'b1)   //外部直接请求写使能
                next_state <= Flash_Write_Enable;
            // 写/擦除操作：先进入写使能，自动插入
            else if( write_req == 1'b1 )
                next_state <= Flash_Write_Enable;
            else if( erase_sector_req == 1'b1)
                next_state <= Flash_Write_Enable;
            else if( erase_bulk_req == 1'b1 )
                next_state <= Flash_Write_Enable;
            else if( read_req == 1'b1 )
                next_state <= Flash_Read_Data_Bytes;
            else
                next_state <= Flash_Idle; 

        Flash_Read_Identification:
            if( (spi_wr_byte_cnt == `Read_Identification_Bytes - 1'b1) && spi_read_ack == 1'b1)
                next_state <= Flash_End;
            else
                next_state <= Flash_Read_Identification;
        Flash_Write_Enable:
            if( spi_write_ack == 1'b1)
                next_state <= Flash_Wait;
            else
                next_state <= Flash_Write_Enable;
        Flash_Page_Program:
            if( (spi_wr_byte_cnt == write_size + 'd1 + 'd3 - 1'b1) && spi_write_ack == 1'b1) 
                next_state <= Flash_Wait;
            else
                next_state <= Flash_Page_Program;
        Flash_Read_Data_Bytes:
            if( (spi_wr_byte_cnt == read_size + 'd1 + 'd3 - 1'b1) && spi_read_ack == 1'b1)  
                next_state <= Flash_Wait;
            else
                next_state <= Flash_Read_Data_Bytes;
        Flash_Sector_Erase:
            if( (spi_wr_byte_cnt == `Sector_Erase_Bytes - 1'b1) && spi_write_ack == 1'b1)
                next_state <= Flash_Wait;
            else
                next_state <= Flash_Sector_Erase;
        Flash_Bulk_Erase:
            if( (spi_wr_byte_cnt == `Bulk_Erase_Bytes - 1'b1) && spi_write_ack == 1'b1)
                next_state <= Flash_Wait;
            else
                next_state <= Flash_Bulk_Erase;
        Flash_Wait:
            if( state_ts == Flash_Page_Program && pp_erase_wait_cnt == Page_Program_Wait)
                next_state <= Flash_End;
            else if(state_ts == Flash_Sector_Erase && pp_erase_wait_cnt == Sector_Erase_Wait)
                next_state <= Flash_End;
            else if(state_ts == Flash_Bulk_Erase && pp_erase_wait_cnt == Bulk_Erase_Wait)
                next_state <= Flash_End;
            else if(state_ts == Flash_Read_Data_Bytes && pp_erase_wait_cnt == Read_Data_Bytes_Wait)
                next_state <= Flash_End;
            else if(state_ts == Flash_Write_Enable && pp_erase_wait_cnt == Write_Enable_Wait) begin
                if(auto_we == 1'b1)
                    next_state <= target_op;      // 自动插入后跳转到目标操作
                else
                    next_state <= Flash_End;      // 外部请求写使能则结束
            end
            else
                next_state <= Flash_Wait;
        Flash_End:
            next_state <= Flash_Idle;
        default:    next_state <= Flash_Idle;
    endcase
end

//
always@(posedge sys_clk or negedge rst ) begin
    if( rst == 1'b0)
        state_ts <= Flash_Idle;
    else if( state == Flash_Idle )
        if( write_req == 1'b1 )
            state_ts <= Flash_Page_Program;
        else if( erase_sector_req == 1'b1)
            state_ts <= Flash_Sector_Erase;
        else if( erase_bulk_req == 1'b1 )
            state_ts <= Flash_Bulk_Erase;
        else if( read_req == 1'b1)
            state_ts <= Flash_Read_Data_Bytes;
        else if( write_enable_req == 1'b1)
            state_ts <= Flash_Write_Enable;
        else
            state_ts <= Flash_Idle; 
    // 自动插入写使能后，进入目标操作时更新 state_ts
    else if( state == Flash_Wait && auto_we == 1'b1 && next_state == target_op)
        state_ts <= target_op;
    else
        state_ts <= state_ts;
end

//
always@(posedge sys_clk or negedge rst)begin
    if( rst == 1'b0)
        pp_erase_wait_cnt <= 'd0;
    else if(state == Flash_Wait)
        pp_erase_wait_cnt <= pp_erase_wait_cnt + 1'b1;
    else    
        pp_erase_wait_cnt <= 'd0;
end

//
always@(posedge sys_clk or negedge rst )begin
    if( rst == 1'b0)
        spi_csn_reg <= 1'b1;
    else if( state == Flash_Idle || state == Flash_End || state == Flash_Wait)
        spi_csn_reg <= 1'b1;
    else
        spi_csn_reg <= 1'b0;
end

//
always@(posedge sys_clk or negedge rst)begin
    if( rst == 1'b0 )
        spi_wr_byte_cnt <= 'd0;
    else if( state != next_state)   
        spi_wr_byte_cnt <= 'd0;
    else if( spi_read_ack == 1'b1 || spi_write_ack == 1'b1)
        spi_wr_byte_cnt <= spi_wr_byte_cnt + 1'b1;
    else    
        spi_wr_byte_cnt <= spi_wr_byte_cnt;
end

//
always@(posedge sys_clk or negedge rst)begin
    if( rst == 1'b0 )
        spi_read_req <= 1'b0;
    else if( state == Flash_Read_Identification && spi_wr_byte_cnt > 'd0)  
        if((spi_wr_byte_cnt == `Read_Identification_Bytes - 1'b1) && spi_read_ack == 1'b1)
            spi_read_req <= 1'b0;
        else
            spi_read_req <= 1'b1;
    else if(state == Flash_Read_Data_Bytes && spi_wr_byte_cnt > 'd3)
        if((spi_wr_byte_cnt == read_size + 'd1 + 'd3 - 1'b1) && spi_read_ack == 1'b1)
            spi_read_req <= 1'b0;
        else
            spi_read_req <= 1'b1;
    else
        spi_read_req <= 1'b0;
end

//
always@(posedge sys_clk or negedge rst)begin
    if( rst == 1'b0)
        flash_id_reg <= 'd0;
    else if( state == Flash_Read_Identification && spi_wr_byte_cnt > 'd0)
        if( spi_read_ack == 1'b1)
            flash_id_reg <= {flash_id_reg[15:0],spi_read_data};
        else
            flash_id_reg <= flash_id_reg;
    else
        flash_id_reg <= flash_id_reg;
end

//
always@(posedge sys_clk or negedge rst)begin
    if( rst == 1'b0 )
        spi_write_req <= 1'b0;
    else if( state == Flash_Write_Enable)
        spi_write_req <= 1'b1;
    else if( state == Flash_Read_Identification && spi_wr_byte_cnt == 'd0)
        spi_write_req <= 1'b1;
    else if( state == Flash_Page_Program)
        spi_write_req <= 1'b1;
    else if( state == Flash_Read_Data_Bytes && spi_wr_byte_cnt < 'd4)
        spi_write_req <= 1'b1;
    else if( state == Flash_Sector_Erase && spi_wr_byte_cnt < 'd4 )
        spi_write_req <= 1'b1;
    else if( state == Flash_Bulk_Erase )    
        spi_write_req <= 1'b1;
    else
        spi_write_req <= 1'b0;
end

//
always@(posedge sys_clk or negedge rst)begin
    if( rst == 1'b0 )
        spi_write_data <= 8'd0;
    else if( state == Flash_Write_Enable )
        spi_write_data <= `Write_Enable;
    else if( state == Flash_Read_Identification && spi_wr_byte_cnt == 'd0)
        spi_write_data <= `Read_Identification;    
    else if( state == Flash_Page_Program)
        case(spi_wr_byte_cnt)
        'd0:     spi_write_data <= `Page_Program;
        'd1:     spi_write_data <= write_page[23:16];
        'd2:     spi_write_data <= write_page[15:8];
        'd3:     spi_write_data <= write_page[7:0];    
        default: spi_write_data <= write_data;
        endcase
    else if(state == Flash_Read_Data_Bytes)
        if( spi_wr_byte_cnt == 'd0)
            spi_write_data <= `Read_Data_Bytes;
        else if( spi_wr_byte_cnt == 'd1)
            spi_write_data <= read_addr[23:16];
        else if( spi_wr_byte_cnt == 'd2)
            spi_write_data <= read_addr[15:8];
        else
            spi_write_data <= read_addr[7:0];
    else if( state == Flash_Sector_Erase)
        if( spi_wr_byte_cnt == 'd0)
            spi_write_data <= `Sector_Erase;
        else if( spi_wr_byte_cnt == 'd1)
            spi_write_data <= erase_sector_addr[23:16];
        else if( spi_wr_byte_cnt == 'd2)
            spi_write_data <= erase_sector_addr[15:8];
        else
            spi_write_data <= erase_sector_addr[7:0];
    else if( state == Flash_Bulk_Erase)
        spi_write_data <= `Bulk_Erase;
    else
        spi_write_data <= 8'd0;
end

// 新增：auto_we 和 target_op 的时序逻辑
always@(posedge sys_clk or negedge rst) begin
    if(rst == 1'b0) begin
        auto_we <= 1'b0;
        target_op <= Flash_Idle;
    end else begin
        // 在Idle状态，当检测到需要自动写使能的请求时，设置标志和目标
        if(state == Flash_Idle) begin
            if(write_req == 1'b1) begin
                auto_we <= 1'b1;
                target_op <= Flash_Page_Program;
            end else if(erase_sector_req == 1'b1) begin
                auto_we <= 1'b1;
                target_op <= Flash_Sector_Erase;
            end else if(erase_bulk_req == 1'b1) begin
                auto_we <= 1'b1;
                target_op <= Flash_Bulk_Erase;
            end else if(write_enable_req == 1'b1) begin
                auto_we <= 1'b0;   // 外部请求，非自动
                target_op <= Flash_Idle;
            end else begin
                // 其他情况保持
            end
        end else if(state == Flash_Wait && auto_we == 1'b1 && next_state == target_op) begin
            // 跳转到目标操作时清除自动标志（在下一个周期）
            auto_we <= 1'b0;
        end else if(state == target_op && auto_we == 1'b1) begin
            // 进入目标操作时清除（安全措施）
            auto_we <= 1'b0;
        end
    end
end

//spi module
spi_master inst_spi_master(
    .sys_clk    (sys_clk),
    .rst   		(rst),
    .read_req   (spi_read_req),
    .read_data  (spi_read_data),
    .read_ack   (spi_read_ack),
    .write_req  (spi_write_req),
    .write_data (spi_write_data),
    .write_ack  (spi_write_ack),
    .spi_clk    (spi_clk), 
    .spi_mosi   (spi_mosi),
    .spi_miso   (spi_miso)
);

endmodule