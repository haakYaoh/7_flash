#----------------------系统时钟---------------------------
set_property -dict {PACKAGE_PIN G11 IOSTANDARD LVCMOS33} [get_ports sys_clk]

#----------------------系统复位---------------------------
set_property -dict {PACKAGE_PIN B6 IOSTANDARD LVCMOS33} [get_ports rst]

# FLASH
set_property PACKAGE_PIN D13 [get_ports spi_clk]
set_property PACKAGE_PIN F14 [get_ports spi_miso]
set_property PACKAGE_PIN G14 [get_ports spi_mosi]
set_property PACKAGE_PIN D12 [get_ports spi_csn]

set_property IOSTANDARD LVCMOS33 [get_ports spi_clk]
set_property IOSTANDARD LVCMOS33 [get_ports spi_miso]
set_property IOSTANDARD LVCMOS33 [get_ports spi_mosi]
set_property IOSTANDARD LVCMOS33 [get_ports spi_csn]

# UART
set_property -dict {PACKAGE_PIN F12 IOSTANDARD LVCMOS33} [get_ports uart_txd]
set_property -dict {PACKAGE_PIN E12 IOSTANDARD LVCMOS33} [get_ports uart_rxd]

#SEG
set_property -dict {PACKAGE_PIN D3 IOSTANDARD LVCMOS33} [get_ports {seg[0]}]
set_property -dict {PACKAGE_PIN C3 IOSTANDARD LVCMOS33} [get_ports {seg[1]}]
set_property -dict {PACKAGE_PIN A4 IOSTANDARD LVCMOS33} [get_ports {seg[2]}]
set_property -dict {PACKAGE_PIN A3 IOSTANDARD LVCMOS33} [get_ports {seg[3]}]
set_property -dict {PACKAGE_PIN B3 IOSTANDARD LVCMOS33} [get_ports {seg[4]}]
set_property -dict {PACKAGE_PIN A2 IOSTANDARD LVCMOS33} [get_ports {seg[5]}]
set_property -dict {PACKAGE_PIN B5 IOSTANDARD LVCMOS33} [get_ports {seg[6]}]
set_property -dict {PACKAGE_PIN A5 IOSTANDARD LVCMOS33} [get_ports {seg[7]}]

set_property -dict {PACKAGE_PIN B2 IOSTANDARD LVCMOS33} [get_ports {sel[0]}]
set_property -dict {PACKAGE_PIN B1 IOSTANDARD LVCMOS33} [get_ports {sel[1]}]
set_property -dict {PACKAGE_PIN C5 IOSTANDARD LVCMOS33} [get_ports {sel[2]}]
set_property -dict {PACKAGE_PIN C4 IOSTANDARD LVCMOS33} [get_ports {sel[3]}]
set_property -dict {PACKAGE_PIN E4 IOSTANDARD LVCMOS33} [get_ports {sel[4]}]
set_property -dict {PACKAGE_PIN D4 IOSTANDARD LVCMOS33} [get_ports {sel[5]}]
set_property -dict {PACKAGE_PIN F3 IOSTANDARD LVCMOS33} [get_ports {sel[6]}]
set_property -dict {PACKAGE_PIN F2 IOSTANDARD LVCMOS33} [get_ports {sel[7]}]

set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]  

