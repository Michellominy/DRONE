## =========================================================
## MPU6050 CUSTOM IP CONSTRAINTS (PYNQ-Z2)
## =========================================================

set_property PACKAGE_PIN W19 [get_ports {i2c_scl}]
set_property IOSTANDARD LVCMOS33 [get_ports {i2c_scl}]
set_property PULLUP true [get_ports {i2c_scl}]

set_property PACKAGE_PIN W18 [get_ports {i2c_sda}]
set_property IOSTANDARD LVCMOS33 [get_ports {i2c_sda}]
set_property PULLUP true [get_ports {i2c_sda}]

set_property PACKAGE_PIN Y18 [get_ports {mpu_int_0}]
set_property IOSTANDARD LVCMOS33 [get_ports {mpu_int_0}]
set_property PULLUP true [get_ports {mpu_int_0}]
## =========================================================