module top (
    input                   sys_clk,
    input                   rst,
    input                   uart_rxd,
    output                  uart_txd,
    output                  spi_clk,
    input                   spi_miso,
    output                  spi_mosi,
    output                  spi_csn,
    output  [7:0]           seg,
    output  [7:0]           sel
);

localparam MSG_PROMPT     = 4'd0;
localparam MSG_WADDR      = 4'd1;
localparam MSG_DATA       = 4'd2;
localparam MSG_RADDR      = 4'd3;
localparam MSG_DADDR      = 4'd4;
localparam MSG_ID_PREFIX  = 4'd5;
localparam MSG_READ_HEAD  = 4'd6;
localparam MSG_WRITE_OK   = 4'd7;
localparam MSG_DEL_OK     = 4'd8;
localparam MSG_ERR        = 4'd9;
localparam MSG_CRLF       = 4'd10;

localparam MSG_PROMPT_LEN    = 9'd45;
localparam MSG_WADDR_LEN     = 9'd42;
localparam MSG_DATA_LEN      = 9'd26;
localparam MSG_RADDR_LEN     = 9'd41;
localparam MSG_DADDR_LEN     = 9'd42;
localparam MSG_ID_PREFIX_LEN = 9'd12;
localparam MSG_READ_HEAD_LEN = 9'd14;
localparam MSG_WRITE_OK_LEN  = 9'd28;
localparam MSG_DEL_OK_LEN    = 9'd22;
localparam MSG_ERR_LEN       = 9'd36;
localparam MSG_CRLF_LEN      = 9'd2;

localparam [8*45-1:0] STR_PROMPT    = "\r\nPlease enter operation: write/read/ID/del\r\n";
localparam [8*42-1:0] STR_WADDR     = "\r\nWhich address to write? 000000-FFFFFF:\r\n";
localparam [8*26-1:0] STR_DATA      = "\r\nPlease enter the data:\r\n";
localparam [8*41-1:0] STR_RADDR     = "\r\nWhich address to read? 000000-FFFFFF:\r\n";
localparam [8*42-1:0] STR_DADDR     = "\r\nWhich sector to delete? 000000-FFFFFF:\r\n";
localparam [8*12-1:0] STR_ID_PREFIX = "\r\nFlash ID: ";
localparam [8*14-1:0] STR_READ_HEAD = "\r\nRead data:\r\n";
localparam [8*28-1:0] STR_WRITE_OK  = "\r\nData storage successful!\r\n";
localparam [8*22-1:0] STR_DEL_OK    = "\r\nDelete successful!\r\n";
localparam [8*36-1:0] STR_ERR       = "\r\nInvalid input, please try again.\r\n";
localparam [8*2-1:0]  STR_CRLF      = "\r\n";

localparam TX_MSG       = 2'd0;
localparam TX_ID_VALUE  = 2'd1;
localparam TX_READ_DATA = 2'd2;

localparam S_BOOT           = 6'd0;
localparam S_WAIT_PROMPT    = 6'd1;
localparam S_WAIT_CMD       = 6'd2;
localparam S_WAIT_WADDR_MSG = 6'd3;
localparam S_RECV_WADDR     = 6'd4;
localparam S_WAIT_DATA_MSG  = 6'd5;
localparam S_RECV_DATA      = 6'd6;
localparam S_ERASE1_START   = 6'd7;
localparam S_ERASE1_WAIT    = 6'd8;
localparam S_ERASE2_START   = 6'd9;
localparam S_ERASE2_WAIT    = 6'd10;
localparam S_WRITE_START    = 6'd11;
localparam S_WRITE_WAIT     = 6'd12;
localparam S_WAIT_WRITE_OK  = 6'd13;
localparam S_WAIT_RADDR_MSG = 6'd14;
localparam S_RECV_RADDR     = 6'd15;
localparam S_READ_LEN_START = 6'd16;
localparam S_READ_LEN_WAIT  = 6'd17;
localparam S_READ_START     = 6'd18;
localparam S_READ_WAIT      = 6'd19;
localparam S_WAIT_READ_HEAD = 6'd20;
localparam S_WAIT_READ_DATA = 6'd21;
localparam S_WAIT_READ_CRLF = 6'd22;
localparam S_ID_START       = 6'd23;
localparam S_ID_WAIT        = 6'd24;
localparam S_WAIT_ID_PREFIX = 6'd25;
localparam S_WAIT_ID_VALUE  = 6'd26;
localparam S_WAIT_DADDR_MSG = 6'd27;
localparam S_RECV_DADDR     = 6'd28;
localparam S_DEL_START      = 6'd29;
localparam S_DEL_WAIT       = 6'd30;
localparam S_WAIT_DEL_OK    = 6'd31;
localparam S_WAIT_ERR       = 6'd32;

reg [5:0]  state;
reg [5:0]  return_state;
reg [7:0]  cmd0, cmd1, cmd2, cmd3, cmd4;
reg [5:0]  line_len;
reg [23:0] addr_shift;
reg [2:0]  addr_digits;
reg        input_error;
reg [8:0]  data_len;
reg        data_overflow;
reg [23:0] op_addr;
reg [23:0] cur_addr;
reg [8:0]  total_len;
reg [8:0]  write_offset;
reg [8:0]  chunk_len;
reg [8:0]  write_index;
reg [8:0]  read_index;
reg [8:0]  read_len;
reg [7:0]  len_byte;
reg [23:0] display_value;

reg [7:0]  data_buf [0:255];
reg [7:0]  read_buf [0:255];

wire       uart_rx_done;
wire [7:0] uart_rx_data;
reg        uart_tx_en;
reg [7:0]  uart_tx_data;
wire       uart_tx_busy;
wire       uart_tx_accept;

reg        tx_active;
reg [1:0]  tx_kind;
reg [3:0]  tx_msg_id;
reg [8:0]  tx_len;
reg [8:0]  tx_index;
reg        tx_done;

reg        read_id_req;
wire [23:0] flash_id;
wire       read_id_end;
reg        read_req;
reg [23:0] read_addr;
reg [9:0]  read_size;
wire [7:0] flash_read_data;
wire       read_ack;
wire       read_end;
reg        write_req;
reg [23:0] write_page;
reg [8:0]  write_size;
wire       write_ack;
wire       write_end;
reg        erase_sector_req;
reg [23:0] erase_sector_addr;
wire       erase_sector_end;

wire [39:0] seg_digits;
wire [8:0]  write_abs_index;
wire [7:0]  write_buf_index;
wire [7:0]  write_data_mux;

assign seg_digits = {
    5'd18, 5'd18,
    {1'b0, display_value[23:20]},
    {1'b0, display_value[19:16]},
    {1'b0, display_value[15:12]},
    {1'b0, display_value[11:8]},
    {1'b0, display_value[7:4]},
    {1'b0, display_value[3:0]}
};

uart_rx u_uart_rx (
    .clk          (sys_clk),
    .rst          (rst),
    .uart_rxd     (uart_rxd),
    .uart_rx_done (uart_rx_done),
    .uart_rx_data (uart_rx_data)
);

uart_tx u_uart_tx (
    .clk            (sys_clk),
    .rst            (rst),
    .uart_tx_data   (uart_tx_data),
    .uart_tx_en     (uart_tx_en),
    .uart_txd       (uart_txd),
    .uart_tx_busy   (uart_tx_busy),
    .uart_tx_accept (uart_tx_accept)
);

seg u_seg (
    .clk     (sys_clk),
    .rst     (rst),
    .data_in (seg_digits),
    .dp_in   (8'hFF),
    .seg     (seg),
    .sel     (sel)
);

flash_driver u_flash_driver (
    .sys_clk          (sys_clk),
    .rst              (~rst),
    .read_id_req      (read_id_req),
    .flash_id         (flash_id),
    .read_id_end      (read_id_end),
    .read_req         (read_req),
    .read_addr        (read_addr),
    .read_size        (read_size),
    .read_data        (flash_read_data),
    .read_ack         (read_ack),
    .read_end         (read_end),
    .write_enable_req (1'b0),
    .write_enable_end (),
    .write_req        (write_req),
    .write_page       (write_page),
    .write_size       (write_size),
    .write_data       (write_data_mux),
    .write_ack        (write_ack),
    .write_end        (write_end),
    .erase_sector_req (erase_sector_req),
    .erase_sector_addr(erase_sector_addr),
    .erase_sector_end (erase_sector_end),
    .erase_bulk_req   (1'b0),
    .erase_bulk_end   (),
    .spi_clk          (spi_clk),
    .spi_mosi         (spi_mosi),
    .spi_miso         (spi_miso),
    .spi_csn          (spi_csn)
);

assign write_abs_index = write_offset + write_index;
assign write_buf_index = write_abs_index[7:0];
assign write_data_mux = ((write_offset + write_index) == 9'd0) ?
                        data_len[7:0] : data_buf[write_buf_index - 8'd1];

function [7:0] to_lower;
    input [7:0] ch;
    begin
        if (ch >= "A" && ch <= "Z")
            to_lower = ch + 8'd32;
        else
            to_lower = ch;
    end
endfunction

function is_enter;
    input [7:0] ch;
    begin
        is_enter = (ch == 8'h0D) || (ch == 8'h0A);
    end
endfunction

function is_hex;
    input [7:0] ch;
    begin
        is_hex = ((ch >= "0") && (ch <= "9")) ||
                 ((ch >= "a") && (ch <= "f")) ||
                 ((ch >= "A") && (ch <= "F"));
    end
endfunction

function [3:0] hex_value;
    input [7:0] ch;
    begin
        if (ch >= "0" && ch <= "9")
            hex_value = ch - "0";
        else if (ch >= "a" && ch <= "f")
            hex_value = ch - "a" + 4'd10;
        else
            hex_value = ch - "A" + 4'd10;
    end
endfunction

function [7:0] hex_char;
    input [3:0] val;
    begin
        hex_char = (val < 4'd10) ? ("0" + val) : ("A" + val - 4'd10);
    end
endfunction

function [8:0] msg_len;
    input [3:0] msg_id;
    begin
        case (msg_id)
            MSG_PROMPT:    msg_len = MSG_PROMPT_LEN;
            MSG_WADDR:     msg_len = MSG_WADDR_LEN;
            MSG_DATA:      msg_len = MSG_DATA_LEN;
            MSG_RADDR:     msg_len = MSG_RADDR_LEN;
            MSG_DADDR:     msg_len = MSG_DADDR_LEN;
            MSG_ID_PREFIX: msg_len = MSG_ID_PREFIX_LEN;
            MSG_READ_HEAD: msg_len = MSG_READ_HEAD_LEN;
            MSG_WRITE_OK:  msg_len = MSG_WRITE_OK_LEN;
            MSG_DEL_OK:    msg_len = MSG_DEL_OK_LEN;
            MSG_ERR:       msg_len = MSG_ERR_LEN;
            default:       msg_len = MSG_CRLF_LEN;
        endcase
    end
endfunction

function [7:0] msg_char;
    input [3:0] msg_id;
    input [8:0] idx;
    begin
        case (msg_id)
            MSG_PROMPT:    msg_char = STR_PROMPT[8*(MSG_PROMPT_LEN - 9'd1 - idx) +: 8];
            MSG_WADDR:     msg_char = STR_WADDR[8*(MSG_WADDR_LEN - 9'd1 - idx) +: 8];
            MSG_DATA:      msg_char = STR_DATA[8*(MSG_DATA_LEN - 9'd1 - idx) +: 8];
            MSG_RADDR:     msg_char = STR_RADDR[8*(MSG_RADDR_LEN - 9'd1 - idx) +: 8];
            MSG_DADDR:     msg_char = STR_DADDR[8*(MSG_DADDR_LEN - 9'd1 - idx) +: 8];
            MSG_ID_PREFIX: msg_char = STR_ID_PREFIX[8*(MSG_ID_PREFIX_LEN - 9'd1 - idx) +: 8];
            MSG_READ_HEAD: msg_char = STR_READ_HEAD[8*(MSG_READ_HEAD_LEN - 9'd1 - idx) +: 8];
            MSG_WRITE_OK:  msg_char = STR_WRITE_OK[8*(MSG_WRITE_OK_LEN - 9'd1 - idx) +: 8];
            MSG_DEL_OK:    msg_char = STR_DEL_OK[8*(MSG_DEL_OK_LEN - 9'd1 - idx) +: 8];
            MSG_ERR:       msg_char = STR_ERR[8*(MSG_ERR_LEN - 9'd1 - idx) +: 8];
            default:       msg_char = STR_CRLF[8*(MSG_CRLF_LEN - 9'd1 - idx) +: 8];
        endcase
    end
endfunction

function [7:0] tx_current_byte;
    input [1:0] kind;
    input [3:0] msg_id;
    input [8:0] idx;
    begin
        case (kind)
            TX_ID_VALUE: begin
                case (idx)
                    9'd0: tx_current_byte = hex_char(flash_id[23:20]);
                    9'd1: tx_current_byte = hex_char(flash_id[19:16]);
                    9'd2: tx_current_byte = hex_char(flash_id[15:12]);
                    9'd3: tx_current_byte = hex_char(flash_id[11:8]);
                    9'd4: tx_current_byte = hex_char(flash_id[7:4]);
                    9'd5: tx_current_byte = hex_char(flash_id[3:0]);
                    9'd6: tx_current_byte = 8'h0D;
                    default: tx_current_byte = 8'h0A;
                endcase
            end
            TX_READ_DATA:
                tx_current_byte = read_buf[idx[7:0]];
            default:
                tx_current_byte = msg_char(msg_id, idx);
        endcase
    end
endfunction

task start_msg;
    input [3:0] msg_id;
    begin
        tx_active <= 1'b1;
        tx_kind <= TX_MSG;
        tx_msg_id <= msg_id;
        tx_len <= msg_len(msg_id);
        tx_index <= 9'd0;
    end
endtask

task start_id_value;
    begin
        tx_active <= 1'b1;
        tx_kind <= TX_ID_VALUE;
        tx_msg_id <= MSG_CRLF;
        tx_len <= 9'd8;
        tx_index <= 9'd0;
    end
endtask

task start_read_data;
    begin
        tx_active <= 1'b1;
        tx_kind <= TX_READ_DATA;
        tx_msg_id <= MSG_CRLF;
        tx_len <= read_len;
        tx_index <= 9'd0;
    end
endtask

function [8:0] page_space;
    input [23:0] addr;
    begin
        page_space = 9'd256 - {1'b0, addr[7:0]};
    end
endfunction

integer i;

always @(posedge sys_clk or posedge rst) begin
    if (rst) begin
        state <= S_BOOT;
        return_state <= S_WAIT_CMD;
        cmd0 <= 8'd0;
        cmd1 <= 8'd0;
        cmd2 <= 8'd0;
        cmd3 <= 8'd0;
        cmd4 <= 8'd0;
        line_len <= 6'd0;
        addr_shift <= 24'd0;
        addr_digits <= 3'd0;
        input_error <= 1'b0;
        data_len <= 9'd0;
        data_overflow <= 1'b0;
        op_addr <= 24'd0;
        cur_addr <= 24'd0;
        total_len <= 9'd0;
        write_offset <= 9'd0;
        chunk_len <= 9'd0;
        write_index <= 9'd0;
        read_index <= 9'd0;
        read_len <= 9'd0;
        len_byte <= 8'd0;
        display_value <= 24'd0;
        uart_tx_en <= 1'b0;
        uart_tx_data <= 8'hFF;
        tx_active <= 1'b0;
        tx_kind <= TX_MSG;
        tx_msg_id <= MSG_PROMPT;
        tx_len <= 9'd0;
        tx_index <= 9'd0;
        tx_done <= 1'b0;
        read_id_req <= 1'b0;
        read_req <= 1'b0;
        read_addr <= 24'd0;
        read_size <= 10'd0;
        write_req <= 1'b0;
        write_page <= 24'd0;
        write_size <= 9'd0;
        erase_sector_req <= 1'b0;
        erase_sector_addr <= 24'd0;
        for (i = 0; i < 256; i = i + 1) begin
            data_buf[i] <= 8'd0;
            read_buf[i] <= 8'd0;
        end
    end else begin
        uart_tx_en <= 1'b0;
        tx_done <= 1'b0;
        read_id_req <= 1'b0;
        read_req <= 1'b0;
        write_req <= 1'b0;
        erase_sector_req <= 1'b0;

        if (tx_active) begin
            if (uart_tx_accept) begin
                if (tx_index == tx_len - 9'd1) begin
                    tx_active <= 1'b0;
                    tx_done <= 1'b1;
                end else begin
                    tx_index <= tx_index + 1'b1;
                end
            end else if (!uart_tx_busy) begin
                uart_tx_data <= tx_current_byte(tx_kind, tx_msg_id, tx_index);
                uart_tx_en <= 1'b1;
            end
        end

        if (write_ack)
            write_index <= write_index + 1'b1;

        if (read_ack) begin
            if (state == S_READ_LEN_WAIT) begin
                len_byte <= flash_read_data;
            end else if (read_index < 9'd256) begin
                read_buf[read_index[7:0]] <= flash_read_data;
                read_index <= read_index + 1'b1;
            end
        end

        case (state)
            S_BOOT: begin
                if (!tx_active) begin
                    start_msg(MSG_PROMPT);
                    state <= S_WAIT_PROMPT;
                end
            end

            S_WAIT_PROMPT: begin
                if (tx_done) begin
                    state <= S_WAIT_CMD;
                    line_len <= 6'd0;
                end
            end

            S_WAIT_CMD: begin
                if (uart_rx_done) begin
                    if (is_enter(uart_rx_data)) begin
                        if (line_len == 6'd0) begin
                            state <= S_WAIT_CMD;
                        end else if (line_len == 6'd5 && cmd0 == "w" && cmd1 == "r" && cmd2 == "i" && cmd3 == "t" && cmd4 == "e") begin
                            start_msg(MSG_WADDR);
                            state <= S_WAIT_WADDR_MSG;
                        end else if (line_len == 6'd4 && cmd0 == "r" && cmd1 == "e" && cmd2 == "a" && cmd3 == "d") begin
                            start_msg(MSG_RADDR);
                            state <= S_WAIT_RADDR_MSG;
                        end else if (line_len == 6'd2 && cmd0 == "i" && cmd1 == "d") begin
                            state <= S_ID_START;
                        end else if (line_len == 6'd3 && cmd0 == "d" && cmd1 == "e" && cmd2 == "l") begin
                            start_msg(MSG_DADDR);
                            state <= S_WAIT_DADDR_MSG;
                        end else begin
                            start_msg(MSG_ERR);
                            state <= S_WAIT_ERR;
                        end
                        line_len <= 6'd0;
                    end else begin
                        if (line_len == 6'd0) cmd0 <= to_lower(uart_rx_data);
                        if (line_len == 6'd1) cmd1 <= to_lower(uart_rx_data);
                        if (line_len == 6'd2) cmd2 <= to_lower(uart_rx_data);
                        if (line_len == 6'd3) cmd3 <= to_lower(uart_rx_data);
                        if (line_len == 6'd4) cmd4 <= to_lower(uart_rx_data);
                        if (line_len < 6'd31)
                            line_len <= line_len + 1'b1;
                    end
                end
            end

            S_WAIT_WADDR_MSG: begin
                if (tx_done) begin
                    state <= S_RECV_WADDR;
                    addr_shift <= 24'd0;
                    addr_digits <= 3'd0;
                    input_error <= 1'b0;
                end
            end

            S_RECV_WADDR: begin
                if (uart_rx_done) begin
                    if (is_enter(uart_rx_data)) begin
                        if (!input_error && addr_digits != 3'd0) begin
                            op_addr <= addr_shift;
                            display_value <= addr_shift;
                            start_msg(MSG_DATA);
                            state <= S_WAIT_DATA_MSG;
                        end else begin
                            start_msg(MSG_ERR);
                            state <= S_WAIT_ERR;
                        end
                    end else if (is_hex(uart_rx_data) && addr_digits < 3'd6) begin
                        addr_shift <= {addr_shift[19:0], hex_value(uart_rx_data)};
                        addr_digits <= addr_digits + 1'b1;
                    end else begin
                        input_error <= 1'b1;
                    end
                end
            end

            S_WAIT_DATA_MSG: begin
                if (tx_done) begin
                    state <= S_RECV_DATA;
                    data_len <= 9'd0;
                    data_overflow <= 1'b0;
                end
            end

            S_RECV_DATA: begin
                if (uart_rx_done) begin
                    if (is_enter(uart_rx_data)) begin
                        if (!data_overflow && data_len != 9'd0) begin
                            total_len <= data_len + 9'd1;
                            cur_addr <= op_addr;
                            write_offset <= 9'd0;
                            state <= S_ERASE1_START;
                        end else begin
                            start_msg(MSG_ERR);
                            state <= S_WAIT_ERR;
                        end
                    end else if (data_len < 9'd256) begin
                        data_buf[data_len[7:0]] <= uart_rx_data;
                        data_len <= data_len + 1'b1;
                    end else begin
                        data_overflow <= 1'b1;
                    end
                end
            end

            S_ERASE1_START: begin
                erase_sector_addr <= {op_addr[23:12], 12'h000};
                erase_sector_req <= 1'b1;
                state <= S_ERASE1_WAIT;
            end

            S_ERASE1_WAIT: begin
                if (erase_sector_end) begin
                    if ({1'b0, op_addr[11:0]} + total_len > 13'd4096)
                        state <= S_ERASE2_START;
                    else
                        state <= S_WRITE_START;
                end
            end

            S_ERASE2_START: begin
                erase_sector_addr <= {op_addr[23:12], 12'h000} + 24'h001000;
                erase_sector_req <= 1'b1;
                state <= S_ERASE2_WAIT;
            end

            S_ERASE2_WAIT: begin
                if (erase_sector_end)
                    state <= S_WRITE_START;
            end

            S_WRITE_START: begin
                cur_addr <= op_addr + write_offset;
                if ((total_len - write_offset) > page_space(op_addr + write_offset))
                    chunk_len <= page_space(op_addr + write_offset);
                else
                    chunk_len <= total_len - write_offset;
                write_page <= op_addr + write_offset;
                write_size <= ((total_len - write_offset) > page_space(op_addr + write_offset)) ?
                              page_space(op_addr + write_offset) : (total_len - write_offset);
                write_index <= 9'd0;
                write_req <= 1'b1;
                state <= S_WRITE_WAIT;
            end

            S_WRITE_WAIT: begin
                if (write_end) begin
                    write_offset <= write_offset + chunk_len;
                    if (write_offset + chunk_len >= total_len) begin
                        start_msg(MSG_WRITE_OK);
                        state <= S_WAIT_WRITE_OK;
                    end else begin
                        state <= S_WRITE_START;
                    end
                end
            end

            S_WAIT_WRITE_OK: begin
                if (tx_done) begin
                    start_msg(MSG_PROMPT);
                    state <= S_WAIT_PROMPT;
                end
            end

            S_WAIT_RADDR_MSG: begin
                if (tx_done) begin
                    state <= S_RECV_RADDR;
                    addr_shift <= 24'd0;
                    addr_digits <= 3'd0;
                    input_error <= 1'b0;
                end
            end

            S_RECV_RADDR: begin
                if (uart_rx_done) begin
                    if (is_enter(uart_rx_data)) begin
                        if (!input_error && addr_digits != 3'd0) begin
                            op_addr <= addr_shift;
                            display_value <= addr_shift;
                            state <= S_READ_LEN_START;
                        end else begin
                            start_msg(MSG_ERR);
                            state <= S_WAIT_ERR;
                        end
                    end else if (is_hex(uart_rx_data) && addr_digits < 3'd6) begin
                        addr_shift <= {addr_shift[19:0], hex_value(uart_rx_data)};
                        addr_digits <= addr_digits + 1'b1;
                    end else begin
                        input_error <= 1'b1;
                    end
                end
            end

            S_READ_LEN_START: begin
                read_addr <= op_addr;
                read_size <= 10'd1;
                read_req <= 1'b1;
                state <= S_READ_LEN_WAIT;
            end

            S_READ_LEN_WAIT: begin
                if (read_end) begin
                    read_len <= (len_byte == 8'd0) ? 9'd256 : {1'b0, len_byte};
                    read_index <= 9'd0;
                    state <= S_READ_START;
                end
            end

            S_READ_START: begin
                read_addr <= op_addr + 24'd1;
                read_size <= {1'b0, read_len};
                read_req <= 1'b1;
                state <= S_READ_WAIT;
            end

            S_READ_WAIT: begin
                if (read_end) begin
                    start_msg(MSG_READ_HEAD);
                    state <= S_WAIT_READ_HEAD;
                end
            end

            S_WAIT_READ_HEAD: begin
                if (tx_done) begin
                    start_read_data;
                    state <= S_WAIT_READ_DATA;
                end
            end

            S_WAIT_READ_DATA: begin
                if (tx_done) begin
                    start_msg(MSG_CRLF);
                    state <= S_WAIT_READ_CRLF;
                end
            end

            S_WAIT_READ_CRLF: begin
                if (tx_done) begin
                    start_msg(MSG_PROMPT);
                    state <= S_WAIT_PROMPT;
                end
            end

            S_ID_START: begin
                read_id_req <= 1'b1;
                state <= S_ID_WAIT;
            end

            S_ID_WAIT: begin
                if (read_id_end) begin
                    display_value <= flash_id;
                    start_msg(MSG_ID_PREFIX);
                    state <= S_WAIT_ID_PREFIX;
                end
            end

            S_WAIT_ID_PREFIX: begin
                if (tx_done) begin
                    start_id_value;
                    state <= S_WAIT_ID_VALUE;
                end
            end

            S_WAIT_ID_VALUE: begin
                if (tx_done) begin
                    start_msg(MSG_PROMPT);
                    state <= S_WAIT_PROMPT;
                end
            end

            S_WAIT_DADDR_MSG: begin
                if (tx_done) begin
                    state <= S_RECV_DADDR;
                    addr_shift <= 24'd0;
                    addr_digits <= 3'd0;
                    input_error <= 1'b0;
                end
            end

            S_RECV_DADDR: begin
                if (uart_rx_done) begin
                    if (is_enter(uart_rx_data)) begin
                        if (!input_error && addr_digits != 3'd0) begin
                            op_addr <= addr_shift;
                            display_value <= addr_shift;
                            state <= S_DEL_START;
                        end else begin
                            start_msg(MSG_ERR);
                            state <= S_WAIT_ERR;
                        end
                    end else if (is_hex(uart_rx_data) && addr_digits < 3'd6) begin
                        addr_shift <= {addr_shift[19:0], hex_value(uart_rx_data)};
                        addr_digits <= addr_digits + 1'b1;
                    end else begin
                        input_error <= 1'b1;
                    end
                end
            end

            S_DEL_START: begin
                erase_sector_addr <= {op_addr[23:12], 12'h000};
                erase_sector_req <= 1'b1;
                state <= S_DEL_WAIT;
            end

            S_DEL_WAIT: begin
                if (erase_sector_end) begin
                    start_msg(MSG_DEL_OK);
                    state <= S_WAIT_DEL_OK;
                end
            end

            S_WAIT_DEL_OK: begin
                if (tx_done) begin
                    start_msg(MSG_PROMPT);
                    state <= S_WAIT_PROMPT;
                end
            end

            S_WAIT_ERR: begin
                if (tx_done) begin
                    start_msg(MSG_PROMPT);
                    state <= S_WAIT_PROMPT;
                end
            end

            default: state <= S_BOOT;
        endcase
    end
end

endmodule
