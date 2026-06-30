module top(

    input                   sys_clk     ,
    input                   rst         ,
    output                  spi_clk     ,
    input                   spi_miso    ,
    output                  spi_mosi    ,
    output                  spi_csn     ,
    output  [7:0]           seg         ,
    output  [7:0]           sel
);

//para define
localparam  s_idle      =    6'b000_001 ;
localparam  s_erase     =    6'b000_010 ;
localparam  s_wb        =    6'b000_100 ;
localparam  s_w         =    6'b001_000 ;
localparam  s_r         =    6'b010_000 ;
localparam  s_wb2       =    6'b100_000 ;

//reg define
reg     [5:0]       state , next_state  ;
reg     [7:0]       data                ;
reg					read_id				;

//wire define
wire                erase_sector_req    ;
wire                erase_sector_end    ;
wire                write_enable_req    ;
wire                write_enable_end    ;
wire                write_req           ;
wire                write_end           ;
wire                write_ack           ;
wire    [23:0]      seg_data            ;  
wire    [1:0]       key_out             ;
wire                read_req            ;

assign  erase_sector_req = (state == s_erase) ? 1'b1 : 1'b0;
assign  write_enable_req = (state == s_wb || state == s_wb2) ? 1'b1 : 1'b0;
assign  write_req = (state == s_w) ? 1'b1 : 1'b0;
assign  read_req = (state == s_r) ? 1'b1 : 1'b0;


//
always@(posedge sys_clk or posedge rst )begin
    if( rst == 1'b1) begin
        data <= 8'hbc;
		read_id = 1'b1;
	end else
		read_id = 1'b0;
end

//seg module
segdisplay segdisplay_inst(
	.clk 				    (sys_clk)                         ,
	.rst 			        (~rst)                            ,
	
	.seg_number_in          ({4'd10,4'd10,seg_data})          ,
	.seg_number 	        (seg)                             ,
	.seg_choice 	        (sel)
);

//flash module
flash_control flash_control_hp(
    .sys_clk                (sys_clk)           ,  
    .rst                    (~rst)              ,

    .read_id_req            (read_id)        	,
    .flash_id               (seg_data)          ,
    .read_id_end            ()                  ,

    .read_req               (read_req)          ,
    .read_addr              ('d0)               ,
    .read_size              ('d256)             ,
    .read_data              ()                  ,
    .read_ack               ()                  ,
    .read_end               ()                  ,
    .write_enable_req       (write_enable_req)  ,
    .write_enable_end       (write_enable_end)  ,

    .write_req              (write_req)         ,
    .write_page             ('d0)               , 
    .write_size             ('d256)             , 
    .write_data             (data)              ,
    .write_ack              (write_ack)         ,
    .write_end              (write_end)         ,

    .erase_sector_req       (erase_sector_req)  ,
    .erase_sector_addr      ('d0)               ,
    .erase_sector_end       (erase_sector_end)  ,

    .erase_bulk_req         (1'b0)              ,
    .erase_bulk_end         ()                  ,

    .spi_clk                (spi_clk)           ,
    .spi_mosi               (spi_mosi)          ,
    .spi_miso               (spi_miso)          ,
    .spi_csn                (spi_csn)
);

endmodule