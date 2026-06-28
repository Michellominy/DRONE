// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2025 Advanced Micro Devices, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2025.2 (win64) Build 6299465 Fri Nov 14 19:35:11 GMT 2025
// Date        : Sat Jun 27 21:00:05 2026
// Host        : Michel-PC running 64-bit major release  (build 9200)
// Command     : write_verilog -force -mode synth_stub
//               c:/Users/lomin/Documents/git/DRONE/rtl_controller/drone_overlay/drone_overlay.gen/sources_1/bd/drone_block_design/ip/drone_block_design_mpu6050_axis_ip_0_0/drone_block_design_mpu6050_axis_ip_0_0_stub.v
// Design      : drone_block_design_mpu6050_axis_ip_0_0
// Purpose     : Stub declaration of top-level module interface
// Device      : xc7z020clg400-1
// --------------------------------------------------------------------------------

// This empty module with port declaration file causes synthesis tools to infer a black box for IP.
// The synthesis directives are for Synopsys Synplify support to prevent IO buffer insertion.
// Please paste the declaration into a Verilog source file or add the file as an additional source.
(* CHECK_LICENSE_TYPE = "drone_block_design_mpu6050_axis_ip_0_0,mpu6050_axis,{}" *) (* core_generation_info = "drone_block_design_mpu6050_axis_ip_0_0,mpu6050_axis,{x_ipProduct=Vivado 2025.2,x_ipVendor=xilinx.com,x_ipLibrary=user,x_ipName=mpu6050_axis_ip,x_ipVersion=1.0,x_ipCoreRevision=39,x_ipLanguage=VHDL,x_ipSimLanguage=MIXED,CLK_FREQ=100000000,I2C_FREQ=400000}" *) (* downgradeipidentifiedwarnings = "yes" *) 
(* x_core_info = "mpu6050_axis,Vivado 2025.2" *) 
module drone_block_design_mpu6050_axis_ip_0_0(clk, reset_n, mpu_int, scl_i, scl_t, sda_i, sda_t, 
  m_axis_tdata, m_axis_tvalid, m_axis_tready, m_axis_tlast, m_axis_tkeep, debug_mpu_state, 
  debug_raw_byte)
/* synthesis syn_black_box black_box_pad_pin="reset_n,mpu_int,scl_i,scl_t,sda_i,sda_t,m_axis_tdata[127:0],m_axis_tvalid,m_axis_tready,m_axis_tlast,m_axis_tkeep[15:0],debug_mpu_state[3:0],debug_raw_byte[7:0]" */
/* synthesis syn_force_seq_prim="clk" */;
  (* x_interface_info = "xilinx.com:signal:clock:1.0 clk CLK" *) (* x_interface_mode = "slave clk" *) (* x_interface_parameter = "XIL_INTERFACENAME clk, ASSOCIATED_BUSIF m_axis, FREQ_HZ 100000000, FREQ_TOLERANCE_HZ 0, PHASE 0.0, CLK_DOMAIN drone_block_design_processing_system7_0_0_FCLK_CLK0, INSERT_VIP 0" *) input clk /* synthesis syn_isclock = 1 */;
  (* x_interface_info = "xilinx.com:signal:reset:1.0 reset_n RST" *) (* x_interface_mode = "slave reset_n" *) (* x_interface_parameter = "XIL_INTERFACENAME reset_n, POLARITY ACTIVE_LOW, INSERT_VIP 0" *) input reset_n;
  input mpu_int;
  input scl_i;
  output scl_t;
  input sda_i;
  output sda_t;
  (* x_interface_info = "xilinx.com:interface:axis:1.0 m_axis TDATA" *) (* x_interface_mode = "master m_axis" *) (* x_interface_parameter = "XIL_INTERFACENAME m_axis, TDATA_NUM_BYTES 16, TDEST_WIDTH 0, TID_WIDTH 0, TUSER_WIDTH 0, HAS_TREADY 1, HAS_TSTRB 0, HAS_TKEEP 1, HAS_TLAST 1, FREQ_HZ 100000000, PHASE 0.0, CLK_DOMAIN drone_block_design_processing_system7_0_0_FCLK_CLK0, LAYERED_METADATA undef, INSERT_VIP 0" *) output [127:0]m_axis_tdata;
  (* x_interface_info = "xilinx.com:interface:axis:1.0 m_axis TVALID" *) output m_axis_tvalid;
  (* x_interface_info = "xilinx.com:interface:axis:1.0 m_axis TREADY" *) input m_axis_tready;
  (* x_interface_info = "xilinx.com:interface:axis:1.0 m_axis TLAST" *) output m_axis_tlast;
  (* x_interface_info = "xilinx.com:interface:axis:1.0 m_axis TKEEP" *) output [15:0]m_axis_tkeep;
  output [3:0]debug_mpu_state;
  output [7:0]debug_raw_byte;
endmodule
