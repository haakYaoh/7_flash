module spi_master (
    input   wire            sys_clk     ,
    input   wire            rst         ,

    input   wire            read_req    ,
    output  wire[7:0]       read_data   ,
    output  wire            read_ack    ,

    input   wire            write_req   ,
    input   wire[7:0]       write_data  ,
    output  wire            write_ack   ,

    output  wire            spi_clk     ,
    output  wire            spi_mosi    ,
    input   wire            spi_miso
);
    
//para define
localparam SPI_IDLE         = 4'b0001   ;
localparam SPI_DATA         = 4'b0010   ;
localparam SPI_END          = 4'b0100   ;
localparam SPI_END2         = 4'b1000   ;  

//reg define
reg [3:0]   state , next_state          ;
reg [3:0]   spi_rev_send_bit_cnt        ;
reg [7:0]   write_data_reg              ;
reg [7:0]   read_data_reg               ;
reg         spi_clk_reg                 ;
reg         spi_mosi_reg                ;
reg         spi_csn_reg                 ;
reg         spi_clk_inverse_cnt;

//
assign      spi_clk           = spi_clk_reg;
assign      spi_mosi          = spi_mosi_reg;
assign      spi_csn           = spi_csn_reg;
assign      read_data         = read_data_reg;
assign      read_ack          = (state == SPI_END) ? 1'b1 : 1'b0;
assign      write_ack         = (state == SPI_END) ? 1'b1 : 1'b0;

//
always @(posedge sys_clk or negedge rst)   begin
    if (rst == 1'b0)
        state <= SPI_IDLE;
    else
        state <= next_state;
end

//
always@(*)  begin
    case (state)
        SPI_IDLE: 
            if( write_req == 1'b1  || read_req == 1'b1)
                next_state <= SPI_DATA;
            else
                next_state <= SPI_IDLE;
        SPI_DATA:
            if( spi_rev_send_bit_cnt == 'd7 && spi_clk_inverse_cnt == 1'b1)
                next_state <= SPI_END;
            else
                next_state <= SPI_DATA;
        SPI_END:
            next_state <= SPI_END2;
        SPI_END2:
            next_state <= SPI_IDLE;
        default: next_state <= SPI_IDLE; 
    endcase
end

//
always @(posedge sys_clk or negedge rst)   begin
    if( rst == 1'b0)
        spi_clk_inverse_cnt <= 1'b0;
    else if( state == SPI_DATA)
        spi_clk_inverse_cnt <= spi_clk_inverse_cnt + 1'b1;
    else
        spi_clk_inverse_cnt <= 1'b0;       
end

//
always@(posedge sys_clk or negedge rst)    begin
    if( rst == 1'b0)
        spi_rev_send_bit_cnt <= 4'd0;
    else if( spi_clk_inverse_cnt == 1'b1)
        spi_rev_send_bit_cnt <= spi_rev_send_bit_cnt + 1'b1;
    else if( state == SPI_DATA)
        spi_rev_send_bit_cnt <= spi_rev_send_bit_cnt;
    else
        spi_rev_send_bit_cnt <= 4'd0;
end

//
always @(posedge sys_clk or negedge rst)   begin
    if( rst == 1'b0)
        write_data_reg <= 8'd0;
    else if( state == SPI_IDLE && (write_req == 1'b1  || read_req == 1'b1))
        write_data_reg <= write_data;
    else if( state == SPI_DATA && spi_clk_inverse_cnt == 1'b1)
        write_data_reg <= {write_data_reg[6:0],write_data_reg[7]};
    else
        write_data_reg <= write_data_reg;
end

//
always@(posedge sys_clk or negedge rst)    begin
    if( rst == 1'b0)
        spi_clk_reg <= 1'b1;
    else if(state == SPI_DATA)
        spi_clk_reg <= ~spi_clk_reg;
    else   
        spi_clk_reg <= 1'b1;
end

//
always@(posedge sys_clk or negedge rst)    begin
    if( rst == 1'b0)
        spi_mosi_reg <= 1'b1;
    else if(state == SPI_DATA  && write_req == 1'b1)
        spi_mosi_reg <= write_data_reg[7];
    else    
        spi_mosi_reg <= 1'b1;
end

//
always@(posedge sys_clk or negedge rst)    begin
    if( rst == 1'b0)
        read_data_reg <= 1'b0;
    else if(state == SPI_DATA && spi_clk_inverse_cnt == 1'b1)
        read_data_reg <= {read_data_reg[6:0] , spi_miso};
    else
        read_data_reg <= read_data_reg;
end

endmodule