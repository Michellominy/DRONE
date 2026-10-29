## =========================================================
## IMU CUSTOM IP CONSTRAINTS (PYNQ-Z2)
## =========================================================

# RASPI Pin constraint
#set_property PACKAGE_PIN Y17 [get_ports {imu_scl}]
#set_property IOSTANDARD LVCMOS33 [get_ports {imu_scl}]
#set_property PULLUP true [get_ports {imu_scl}]

#set_property PACKAGE_PIN Y16 [get_ports {imu_sda}]
#set_property IOSTANDARD LVCMOS33 [get_ports {imu_sda}]
#set_property PULLUP true [get_ports {imu_sda}]

#set_property PACKAGE_PIN Y18 [get_ports {imu_int_0}]
#set_property IOSTANDARD LVCMOS33 [get_ports {imu_int_0}]
#set_property PULLUP true [get_ports {imu_int_0}]

# Arduino PIN Constraint
# Blue,SDA,I2C Data
# Yellow,SCL,I2C Clock

set_property PACKAGE_PIN Y11 [get_ports {imu_scl}]
set_property IOSTANDARD LVCMOS33 [get_ports {imu_scl}]
set_property PULLUP true [get_ports {imu_scl}]

set_property PACKAGE_PIN Y12 [get_ports {imu_sda}]
set_property IOSTANDARD LVCMOS33 [get_ports {imu_sda}]
set_property PULLUP true [get_ports {imu_sda}]

set_property PACKAGE_PIN W11 [get_ports {imu_int_0}]
set_property IOSTANDARD LVCMOS33 [get_ports {imu_int_0}]
set_property PULLUP true [get_ports {imu_int_0}]




#set_property PACKAGE_PIN W19 [get_ports {pca_scl}]
#set_property IOSTANDARD LVCMOS33 [get_ports {pca_scl}]
#set_property PULLUP true [get_ports {pca_scl}]

#set_property PACKAGE_PIN W18 [get_ports {pca_sda}]
#set_property IOSTANDARD LVCMOS33 [get_ports {pca_sda}]
#set_property PULLUP true [get_ports {pca_sda}]
set_property PACKAGE_PIN V6 [get_ports {motor_pins_0[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {motor_pins_0[0]}]

set_property PACKAGE_PIN Y6 [get_ports {motor_pins_0[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {motor_pins_0[1]}]

set_property PACKAGE_PIN C20 [get_ports {motor_pins_0[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {motor_pins_0[2]}]

set_property PACKAGE_PIN W6 [get_ports {motor_pins_0[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {motor_pins_0[3]}]
## =========================================================