module flash_driver (
    input   wire            sys_clk,
    input   wire            rst,              // active low

    input   wire            read_id_req,
    output  wire [23:0]     flash_id,
    output  wire            read_id_end,

    input   wire            read_req,
    input   wire [23:0]     read_addr,
    input   wire [9:0]      read_size,
    output  wire [7:0]      read_data,
    output  wire            read_ack,
    output  wire            read_end,

    input   wire            write_enable_req,
    output  wire            write_enable_end,

    input   wire            write_req,
    input   wire [23:0]     write_page,
    input   wire [8:0]      write_size,
    input   wire [7:0]      write_data,
    output  wire            write_ack,
    output  wire            write_end,

    input   wire            erase_sector_req,
    input   wire [23:0]     erase_sector_addr,
    output  wire            erase_sector_end,

    input   wire            erase_bulk_req,
    output  wire            erase_bulk_end,

    output  wire            spi_clk,
    output  wire            spi_mosi,
    input   wire            spi_miso,
    output  wire            spi_csn
);

localparam CMD_WRITE_ENABLE = 8'h06;
localparam CMD_READ_ID      = 8'h9F;
localparam CMD_READ_STATUS  = 8'h05;
localparam CMD_READ_DATA    = 8'h03;
localparam CMD_PAGE_PROGRAM = 8'h02;
localparam CMD_SECTOR_ERASE = 8'h20;   // W25Q128 4KB sector erase
localparam CMD_BULK_ERASE   = 8'hC7;

localparam ST_IDLE          = 5'd0;
localparam ST_WRITE_ENABLE  = 5'd1;
localparam ST_GAP           = 5'd2;
localparam ST_READ_ID       = 5'd3;
localparam ST_READ_DATA     = 5'd4;
localparam ST_PAGE_PROGRAM  = 5'd5;
localparam ST_SECTOR_ERASE  = 5'd6;
localparam ST_BULK_ERASE    = 5'd7;
localparam ST_READ_STATUS   = 5'd8;
localparam ST_END           = 5'd9;

localparam OP_NONE          = 4'd0;
localparam OP_WRITE_ENABLE  = 4'd1;
localparam OP_READ_ID       = 4'd2;
localparam OP_READ_DATA     = 4'd3;
localparam OP_PAGE_PROGRAM  = 4'd4;
localparam OP_SECTOR_ERASE  = 4'd5;
localparam OP_BULK_ERASE    = 4'd6;

localparam SPI_CLK_DIV      = 16'd20;
localparam CS_GAP_CYCLES    = 8'd10;

reg  [4:0]  state;
reg  [4:0]  gap_next_state;
reg  [4:0]  status_done_state;
reg  [3:0]  active_op;
reg  [3:0]  done_op;
reg  [9:0]  byte_cnt;
reg  [7:0]  gap_cnt;
reg         spi_wr_req;
reg  [7:0]  spi_data_in;
reg         spi_csn_reg;
reg  [23:0] flash_id_reg;
reg  [7:0]  read_data_reg;
reg         status_busy;

wire        spi_wr_ack;
wire [7:0]  spi_data_out;
wire        spi_tri_en;

assign spi_csn = spi_csn_reg;
assign flash_id = flash_id_reg;
assign read_data = read_ack ? spi_data_out : read_data_reg;

assign read_id_end      = (state == ST_END) && (done_op == OP_READ_ID);
assign read_end         = (state == ST_END) && (done_op == OP_READ_DATA);
assign write_enable_end = (state == ST_END) && (done_op == OP_WRITE_ENABLE);
assign write_end        = (state == ST_END) && (done_op == OP_PAGE_PROGRAM);
assign erase_sector_end = (state == ST_END) && (done_op == OP_SECTOR_ERASE);
assign erase_bulk_end   = (state == ST_END) && (done_op == OP_BULK_ERASE);

assign read_ack  = spi_wr_ack && (state == ST_READ_DATA) && (byte_cnt >= 10'd4);
assign write_ack = spi_wr_ack && (state == ST_PAGE_PROGRAM) && (byte_cnt >= 10'd4);

spi_ip #(
    .CPOL    (1'b0),
    .CPHA    (1'b0),
    .CLK_DIV (SPI_CLK_DIV)
) u_spi_ip (
    .clk      (sys_clk),
    .rst      (~rst),
    .sclk     (spi_clk),
    .mosi     (spi_mosi),
    .miso     (spi_miso),
    .tri_en   (spi_tri_en),
    .wr_req   (spi_wr_req),
    .wr_ack   (spi_wr_ack),
    .data_in  (spi_data_in),
    .data_out (spi_data_out),
    .io_dir   (1'b0)
);

always @(posedge sys_clk or negedge rst) begin
    if (!rst) begin
        state <= ST_IDLE;
        gap_next_state <= ST_IDLE;
        status_done_state <= ST_IDLE;
        active_op <= OP_NONE;
        done_op <= OP_NONE;
        byte_cnt <= 10'd0;
        gap_cnt <= 8'd0;
        spi_wr_req <= 1'b0;
        spi_data_in <= 8'd0;
        spi_csn_reg <= 1'b1;
        flash_id_reg <= 24'd0;
        read_data_reg <= 8'd0;
        status_busy <= 1'b0;
    end else begin
        spi_wr_req <= 1'b0;
        done_op <= OP_NONE;

        case (state)
            ST_IDLE: begin
                spi_csn_reg <= 1'b1;
                byte_cnt <= 10'd0;
                gap_cnt <= 8'd0;
                if (read_id_req) begin
                    state <= ST_READ_ID;
                    active_op <= OP_READ_ID;
                    flash_id_reg <= 24'd0;
                    spi_csn_reg <= 1'b0;
                end else if (read_req && (read_size != 10'd0)) begin
                    state <= ST_READ_DATA;
                    active_op <= OP_READ_DATA;
                    spi_csn_reg <= 1'b0;
                end else if (write_enable_req) begin
                    state <= ST_WRITE_ENABLE;
                    active_op <= OP_WRITE_ENABLE;
                    gap_next_state <= ST_END;
                    spi_csn_reg <= 1'b0;
                end else if (write_req && (write_size != 9'd0)) begin
                    state <= ST_WRITE_ENABLE;
                    active_op <= OP_PAGE_PROGRAM;
                    gap_next_state <= ST_PAGE_PROGRAM;
                    spi_csn_reg <= 1'b0;
                end else if (erase_sector_req) begin
                    state <= ST_WRITE_ENABLE;
                    active_op <= OP_SECTOR_ERASE;
                    gap_next_state <= ST_SECTOR_ERASE;
                    spi_csn_reg <= 1'b0;
                end else if (erase_bulk_req) begin
                    state <= ST_WRITE_ENABLE;
                    active_op <= OP_BULK_ERASE;
                    gap_next_state <= ST_BULK_ERASE;
                    spi_csn_reg <= 1'b0;
                end
            end

            ST_WRITE_ENABLE: begin
                spi_csn_reg <= 1'b0;
                spi_wr_req <= 1'b1;
                spi_data_in <= CMD_WRITE_ENABLE;
                if (spi_wr_ack) begin
                    byte_cnt <= 10'd0;
                    state <= ST_GAP;
                    spi_csn_reg <= 1'b1;
                    gap_cnt <= 8'd0;
                end
            end

            ST_GAP: begin
                spi_csn_reg <= 1'b1;
                byte_cnt <= 10'd0;
                if (gap_cnt == CS_GAP_CYCLES) begin
                    gap_cnt <= 8'd0;
                    if (gap_next_state == ST_END) begin
                        state <= ST_END;
                    end else begin
                        state <= gap_next_state;
                        spi_csn_reg <= 1'b0;
                    end
                end else begin
                    gap_cnt <= gap_cnt + 1'b1;
                end
            end

            ST_READ_ID: begin
                spi_csn_reg <= 1'b0;
                spi_wr_req <= 1'b1;
                spi_data_in <= (byte_cnt == 10'd0) ? CMD_READ_ID : 8'h00;
                if (spi_wr_ack) begin
                    if (byte_cnt != 10'd0)
                        flash_id_reg <= {flash_id_reg[15:0], spi_data_out};
                    if (byte_cnt == 10'd3) begin
                        state <= ST_END;
                        spi_csn_reg <= 1'b1;
                    end else begin
                        byte_cnt <= byte_cnt + 1'b1;
                    end
                end
            end

            ST_READ_DATA: begin
                spi_csn_reg <= 1'b0;
                spi_wr_req <= 1'b1;
                case (byte_cnt)
                    10'd0: spi_data_in <= CMD_READ_DATA;
                    10'd1: spi_data_in <= read_addr[23:16];
                    10'd2: spi_data_in <= read_addr[15:8];
                    10'd3: spi_data_in <= read_addr[7:0];
                    default: spi_data_in <= 8'h00;
                endcase
                if (spi_wr_ack) begin
                    if (byte_cnt >= 10'd4)
                        read_data_reg <= spi_data_out;
                    if (byte_cnt == ({1'b0, read_size} + 10'd3)) begin
                        state <= ST_END;
                        spi_csn_reg <= 1'b1;
                    end else begin
                        byte_cnt <= byte_cnt + 1'b1;
                    end
                end
            end

            ST_PAGE_PROGRAM: begin
                spi_csn_reg <= 1'b0;
                spi_wr_req <= 1'b1;
                case (byte_cnt)
                    10'd0: spi_data_in <= CMD_PAGE_PROGRAM;
                    10'd1: spi_data_in <= write_page[23:16];
                    10'd2: spi_data_in <= write_page[15:8];
                    10'd3: spi_data_in <= write_page[7:0];
                    default: spi_data_in <= write_data;
                endcase
                if (spi_wr_ack) begin
                    if (byte_cnt == ({1'b0, write_size} + 10'd3)) begin
                        state <= ST_GAP;
                        spi_csn_reg <= 1'b1;
                        gap_next_state <= ST_READ_STATUS;
                        status_done_state <= ST_END;
                        byte_cnt <= 10'd0;
                        gap_cnt <= 8'd0;
                    end else begin
                        byte_cnt <= byte_cnt + 1'b1;
                    end
                end
            end

            ST_SECTOR_ERASE: begin
                spi_csn_reg <= 1'b0;
                spi_wr_req <= 1'b1;
                case (byte_cnt)
                    10'd0: spi_data_in <= CMD_SECTOR_ERASE;
                    10'd1: spi_data_in <= erase_sector_addr[23:16];
                    10'd2: spi_data_in <= erase_sector_addr[15:8];
                    default: spi_data_in <= erase_sector_addr[7:0];
                endcase
                if (spi_wr_ack) begin
                    if (byte_cnt == 10'd3) begin
                        state <= ST_GAP;
                        spi_csn_reg <= 1'b1;
                        gap_next_state <= ST_READ_STATUS;
                        status_done_state <= ST_END;
                        byte_cnt <= 10'd0;
                        gap_cnt <= 8'd0;
                    end else begin
                        byte_cnt <= byte_cnt + 1'b1;
                    end
                end
            end

            ST_BULK_ERASE: begin
                spi_csn_reg <= 1'b0;
                spi_wr_req <= 1'b1;
                spi_data_in <= CMD_BULK_ERASE;
                if (spi_wr_ack) begin
                    state <= ST_GAP;
                    spi_csn_reg <= 1'b1;
                    gap_next_state <= ST_READ_STATUS;
                    status_done_state <= ST_END;
                    byte_cnt <= 10'd0;
                    gap_cnt <= 8'd0;
                end
            end

            ST_READ_STATUS: begin
                spi_csn_reg <= 1'b0;
                spi_wr_req <= 1'b1;
                spi_data_in <= (byte_cnt == 10'd0) ? CMD_READ_STATUS : 8'h00;
                if (spi_wr_ack) begin
                    if (byte_cnt == 10'd1) begin
                        status_busy <= spi_data_out[0];
                        state <= ST_GAP;
                        spi_csn_reg <= 1'b1;
                        gap_next_state <= spi_data_out[0] ? ST_READ_STATUS : status_done_state;
                        byte_cnt <= 10'd0;
                        gap_cnt <= 8'd0;
                    end else begin
                        byte_cnt <= byte_cnt + 1'b1;
                    end
                end
            end

            ST_END: begin
                spi_csn_reg <= 1'b1;
                done_op <= active_op;
                active_op <= OP_NONE;
                state <= ST_IDLE;
            end

            default: begin
                state <= ST_IDLE;
                spi_csn_reg <= 1'b1;
            end
        endcase
    end
end

endmodule
