## Pmod A - Header Pins 1 and 2 for Motor I2C
set_property -dict {PACKAGE_PIN Y18 IOSTANDARD LVCMOS33} [get_ports motor_i2c_scl_io]
set_property -dict {PACKAGE_PIN Y19 IOSTANDARD LVCMOS33} [get_ports motor_i2c_sda_io]

## Pmod A - Header Pins 3 and 4 for Sensor I2C
set_property -dict {PACKAGE_PIN Y16 IOSTANDARD LVCMOS33} [get_ports sensor_i2c_scl_io]
set_property -dict {PACKAGE_PIN Y17 IOSTANDARD LVCMOS33} [get_ports sensor_i2c_sda_io]

## Enable FPGA Internal Pull-ups
set_property PULLUP true [get_ports motor_i2c_scl_io]
set_property PULLUP true [get_ports motor_i2c_sda_io]
set_property PULLUP true [get_ports sensor_i2c_scl_io]
set_property PULLUP true [get_ports sensor_i2c_sda_io]