// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2025 Advanced Micro Devices, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2025.2 (win64) Build 6299465 Fri Nov 14 19:35:11 GMT 2025
// Date        : Sat Jun 13 14:26:21 2026
// Host        : Michel-PC running 64-bit major release  (build 9200)
// Command     : write_verilog -force -mode funcsim -rename_top decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix -prefix
//               decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_ drone_block_design_mpu6050_axis_ip_0_0_sim_netlist.v
// Design      : drone_block_design_mpu6050_axis_ip_0_0
// Purpose     : This verilog netlist is a functional simulation representation of the design and should not be modified
//               or synthesized. This netlist cannot be used for SDF annotated simulation.
// Device      : xc7z020clg400-1
// --------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

(* CHECK_LICENSE_TYPE = "drone_block_design_mpu6050_axis_ip_0_0,mpu6050_axis_ip,{}" *) (* downgradeipidentifiedwarnings = "yes" *) (* x_core_info = "mpu6050_axis_ip,Vivado 2025.2" *) 
(* NotValidForBitStream *)
module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix
   (clk,
    reset_n,
    i2c_scl,
    i2c_sda,
    mpu_int,
    m_axis_tdata,
    m_axis_tvalid,
    m_axis_tready);
  (* x_interface_info = "xilinx.com:signal:clock:1.0 clk CLK" *) (* x_interface_mode = "slave clk" *) (* x_interface_parameter = "XIL_INTERFACENAME clk, FREQ_HZ 100000000, FREQ_TOLERANCE_HZ 0, PHASE 0.0, CLK_DOMAIN drone_block_design_processing_system7_0_0_FCLK_CLK0, INSERT_VIP 0" *) input clk;
  (* x_interface_info = "xilinx.com:signal:reset:1.0 reset_n RST" *) (* x_interface_mode = "slave reset_n" *) (* x_interface_parameter = "XIL_INTERFACENAME reset_n, POLARITY ACTIVE_LOW, INSERT_VIP 0" *) input reset_n;
  inout i2c_scl;
  inout i2c_sda;
  input mpu_int;
  output [127:0]m_axis_tdata;
  output m_axis_tvalid;
  input m_axis_tready;

  wire \<const0> ;
  wire clk;
  wire i2c_scl;
  wire i2c_sda;
  wire [111:0]\^m_axis_tdata ;
  wire m_axis_tready;
  wire m_axis_tvalid;
  wire mpu_int;
  wire reset_n;

  assign m_axis_tdata[127] = \<const0> ;
  assign m_axis_tdata[126] = \<const0> ;
  assign m_axis_tdata[125] = \<const0> ;
  assign m_axis_tdata[124] = \<const0> ;
  assign m_axis_tdata[123] = \<const0> ;
  assign m_axis_tdata[122] = \<const0> ;
  assign m_axis_tdata[121] = \<const0> ;
  assign m_axis_tdata[120] = \<const0> ;
  assign m_axis_tdata[119] = \<const0> ;
  assign m_axis_tdata[118] = \<const0> ;
  assign m_axis_tdata[117] = \<const0> ;
  assign m_axis_tdata[116] = \<const0> ;
  assign m_axis_tdata[115] = \<const0> ;
  assign m_axis_tdata[114] = \<const0> ;
  assign m_axis_tdata[113] = \<const0> ;
  assign m_axis_tdata[112] = \<const0> ;
  assign m_axis_tdata[111:0] = \^m_axis_tdata [111:0];
  GND GND
       (.G(\<const0> ));
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_mpu6050_axis_ip U0
       (.clk(clk),
        .i2c_scl(i2c_scl),
        .i2c_sda(i2c_sda),
        .m_axis_tdata(\^m_axis_tdata ),
        .m_axis_tready(m_axis_tready),
        .m_axis_tvalid(m_axis_tvalid),
        .mpu_int(mpu_int),
        .reset_n(reset_n));
endmodule

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_i2c_master
   (clk,
    reset_n,
    ena,
    addr,
    rw,
    data_wr,
    busy,
    data_rd,
    ack_error,
    sda,
    scl);
  input clk;
  input reset_n;
  input ena;
  input [6:0]addr;
  input rw;
  input [7:0]data_wr;
  output busy;
  output [7:0]data_rd;
  output ack_error;
  inout sda;
  inout scl;


endmodule

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_mpu6050_axis_ip
   (m_axis_tdata,
    m_axis_tvalid,
    i2c_sda,
    i2c_scl,
    reset_n,
    clk,
    m_axis_tready,
    mpu_int);
  output [111:0]m_axis_tdata;
  output m_axis_tvalid;
  inout i2c_sda;
  inout i2c_scl;
  input reset_n;
  input clk;
  input m_axis_tready;
  input mpu_int;

  wire \FSM_sequential_state[0]_i_1_n_0 ;
  wire \FSM_sequential_state[1]_i_1_n_0 ;
  wire \FSM_sequential_state[1]_i_2_n_0 ;
  wire \FSM_sequential_state[1]_i_3_n_0 ;
  wire \FSM_sequential_state[1]_i_4_n_0 ;
  wire \FSM_sequential_state[1]_i_5_n_0 ;
  wire \FSM_sequential_state[2]_i_1_n_0 ;
  wire \FSM_sequential_state[2]_i_2_n_0 ;
  wire [3:0]byte_idx;
  wire \byte_idx[3]_i_1_n_0 ;
  wire \byte_idx[3]_i_3_n_0 ;
  wire \byte_idx_reg_n_0_[0] ;
  wire \byte_idx_reg_n_0_[1] ;
  wire \byte_idx_reg_n_0_[2] ;
  wire \byte_idx_reg_n_0_[3] ;
  wire clk;
  wire [6:0]data_wr;
  wire ena;
  wire i2c_busy;
  wire [7:0]i2c_data_rd;
  wire [6:1]i2c_data_wr;
  wire \i2c_data_wr[0]_i_1_n_0 ;
  wire \i2c_data_wr[1]_i_1_n_0 ;
  wire \i2c_data_wr[2]_i_1_n_0 ;
  wire \i2c_data_wr[2]_i_2_n_0 ;
  wire \i2c_data_wr[3]_i_1_n_0 ;
  wire \i2c_data_wr[4]_i_1_n_0 ;
  wire \i2c_data_wr[5]_i_1_n_0 ;
  wire \i2c_data_wr[5]_i_2_n_0 ;
  wire \i2c_data_wr[5]_i_4_n_0 ;
  wire \i2c_data_wr[5]_i_5_n_0 ;
  wire \i2c_data_wr[6]_i_1_n_0 ;
  wire \i2c_data_wr[6]_i_2_n_0 ;
  wire \i2c_data_wr[6]_i_3_n_0 ;
  wire \i2c_data_wr[6]_i_4_n_0 ;
  wire i2c_ena_i_1_n_0;
  wire i2c_rw_i_1_n_0;
  wire i2c_scl;
  wire i2c_sda;
  wire [2:2]init_step;
  wire \init_step[0]_i_1_n_0 ;
  wire \init_step[1]_i_1_n_0 ;
  wire \init_step[2]_i_1_n_0 ;
  wire \init_step[2]_i_3_n_0 ;
  wire \init_step_reg_n_0_[0] ;
  wire \init_step_reg_n_0_[1] ;
  wire \init_step_reg_n_0_[2] ;
  wire int_edge;
  wire int_edge0;
  wire int_q1;
  wire int_q2;
  wire [111:0]m_axis_tdata;
  wire [15:0]m_axis_tdata0;
  wire \m_axis_tdata[103]_i_2_n_0 ;
  wire \m_axis_tdata[103]_i_3_n_0 ;
  wire \m_axis_tdata[103]_i_4_n_0 ;
  wire \m_axis_tdata[103]_i_5_n_0 ;
  wire \m_axis_tdata[107]_i_2_n_0 ;
  wire \m_axis_tdata[107]_i_3_n_0 ;
  wire \m_axis_tdata[107]_i_4_n_0 ;
  wire \m_axis_tdata[107]_i_5_n_0 ;
  wire \m_axis_tdata[111]_i_1_n_0 ;
  wire \m_axis_tdata[111]_i_3_n_0 ;
  wire \m_axis_tdata[111]_i_4_n_0 ;
  wire \m_axis_tdata[111]_i_5_n_0 ;
  wire \m_axis_tdata[111]_i_6_n_0 ;
  wire \m_axis_tdata[19]_i_2_n_0 ;
  wire \m_axis_tdata[19]_i_3_n_0 ;
  wire \m_axis_tdata[19]_i_4_n_0 ;
  wire \m_axis_tdata[23]_i_2_n_0 ;
  wire \m_axis_tdata[23]_i_3_n_0 ;
  wire \m_axis_tdata[23]_i_4_n_0 ;
  wire \m_axis_tdata[23]_i_5_n_0 ;
  wire \m_axis_tdata[27]_i_2_n_0 ;
  wire \m_axis_tdata[27]_i_3_n_0 ;
  wire \m_axis_tdata[27]_i_4_n_0 ;
  wire \m_axis_tdata[27]_i_5_n_0 ;
  wire \m_axis_tdata[31]_i_2_n_0 ;
  wire \m_axis_tdata[31]_i_3_n_0 ;
  wire \m_axis_tdata[31]_i_4_n_0 ;
  wire \m_axis_tdata[31]_i_5_n_0 ;
  wire \m_axis_tdata[35]_i_2_n_0 ;
  wire \m_axis_tdata[35]_i_3_n_0 ;
  wire \m_axis_tdata[35]_i_4_n_0 ;
  wire \m_axis_tdata[39]_i_2_n_0 ;
  wire \m_axis_tdata[39]_i_3_n_0 ;
  wire \m_axis_tdata[39]_i_4_n_0 ;
  wire \m_axis_tdata[39]_i_5_n_0 ;
  wire \m_axis_tdata[43]_i_2_n_0 ;
  wire \m_axis_tdata[43]_i_3_n_0 ;
  wire \m_axis_tdata[43]_i_4_n_0 ;
  wire \m_axis_tdata[43]_i_5_n_0 ;
  wire \m_axis_tdata[47]_i_2_n_0 ;
  wire \m_axis_tdata[47]_i_3_n_0 ;
  wire \m_axis_tdata[47]_i_4_n_0 ;
  wire \m_axis_tdata[47]_i_5_n_0 ;
  wire \m_axis_tdata[83]_i_2_n_0 ;
  wire \m_axis_tdata[83]_i_3_n_0 ;
  wire \m_axis_tdata[83]_i_4_n_0 ;
  wire \m_axis_tdata[87]_i_2_n_0 ;
  wire \m_axis_tdata[87]_i_3_n_0 ;
  wire \m_axis_tdata[87]_i_4_n_0 ;
  wire \m_axis_tdata[87]_i_5_n_0 ;
  wire \m_axis_tdata[91]_i_2_n_0 ;
  wire \m_axis_tdata[91]_i_3_n_0 ;
  wire \m_axis_tdata[91]_i_4_n_0 ;
  wire \m_axis_tdata[91]_i_5_n_0 ;
  wire \m_axis_tdata[95]_i_2_n_0 ;
  wire \m_axis_tdata[95]_i_3_n_0 ;
  wire \m_axis_tdata[95]_i_4_n_0 ;
  wire \m_axis_tdata[95]_i_5_n_0 ;
  wire \m_axis_tdata[99]_i_2_n_0 ;
  wire \m_axis_tdata[99]_i_3_n_0 ;
  wire \m_axis_tdata[99]_i_4_n_0 ;
  wire \m_axis_tdata_reg[103]_i_1_n_0 ;
  wire \m_axis_tdata_reg[103]_i_1_n_1 ;
  wire \m_axis_tdata_reg[103]_i_1_n_2 ;
  wire \m_axis_tdata_reg[103]_i_1_n_3 ;
  wire \m_axis_tdata_reg[107]_i_1_n_0 ;
  wire \m_axis_tdata_reg[107]_i_1_n_1 ;
  wire \m_axis_tdata_reg[107]_i_1_n_2 ;
  wire \m_axis_tdata_reg[107]_i_1_n_3 ;
  wire \m_axis_tdata_reg[111]_i_2_n_1 ;
  wire \m_axis_tdata_reg[111]_i_2_n_2 ;
  wire \m_axis_tdata_reg[111]_i_2_n_3 ;
  wire \m_axis_tdata_reg[19]_i_1_n_0 ;
  wire \m_axis_tdata_reg[19]_i_1_n_1 ;
  wire \m_axis_tdata_reg[19]_i_1_n_2 ;
  wire \m_axis_tdata_reg[19]_i_1_n_3 ;
  wire \m_axis_tdata_reg[19]_i_1_n_4 ;
  wire \m_axis_tdata_reg[19]_i_1_n_5 ;
  wire \m_axis_tdata_reg[19]_i_1_n_6 ;
  wire \m_axis_tdata_reg[19]_i_1_n_7 ;
  wire \m_axis_tdata_reg[23]_i_1_n_0 ;
  wire \m_axis_tdata_reg[23]_i_1_n_1 ;
  wire \m_axis_tdata_reg[23]_i_1_n_2 ;
  wire \m_axis_tdata_reg[23]_i_1_n_3 ;
  wire \m_axis_tdata_reg[23]_i_1_n_4 ;
  wire \m_axis_tdata_reg[23]_i_1_n_5 ;
  wire \m_axis_tdata_reg[23]_i_1_n_6 ;
  wire \m_axis_tdata_reg[23]_i_1_n_7 ;
  wire \m_axis_tdata_reg[27]_i_1_n_0 ;
  wire \m_axis_tdata_reg[27]_i_1_n_1 ;
  wire \m_axis_tdata_reg[27]_i_1_n_2 ;
  wire \m_axis_tdata_reg[27]_i_1_n_3 ;
  wire \m_axis_tdata_reg[27]_i_1_n_4 ;
  wire \m_axis_tdata_reg[27]_i_1_n_5 ;
  wire \m_axis_tdata_reg[27]_i_1_n_6 ;
  wire \m_axis_tdata_reg[27]_i_1_n_7 ;
  wire \m_axis_tdata_reg[31]_i_1_n_1 ;
  wire \m_axis_tdata_reg[31]_i_1_n_2 ;
  wire \m_axis_tdata_reg[31]_i_1_n_3 ;
  wire \m_axis_tdata_reg[31]_i_1_n_4 ;
  wire \m_axis_tdata_reg[31]_i_1_n_5 ;
  wire \m_axis_tdata_reg[31]_i_1_n_6 ;
  wire \m_axis_tdata_reg[31]_i_1_n_7 ;
  wire \m_axis_tdata_reg[35]_i_1_n_0 ;
  wire \m_axis_tdata_reg[35]_i_1_n_1 ;
  wire \m_axis_tdata_reg[35]_i_1_n_2 ;
  wire \m_axis_tdata_reg[35]_i_1_n_3 ;
  wire \m_axis_tdata_reg[35]_i_1_n_4 ;
  wire \m_axis_tdata_reg[35]_i_1_n_5 ;
  wire \m_axis_tdata_reg[35]_i_1_n_6 ;
  wire \m_axis_tdata_reg[35]_i_1_n_7 ;
  wire \m_axis_tdata_reg[39]_i_1_n_0 ;
  wire \m_axis_tdata_reg[39]_i_1_n_1 ;
  wire \m_axis_tdata_reg[39]_i_1_n_2 ;
  wire \m_axis_tdata_reg[39]_i_1_n_3 ;
  wire \m_axis_tdata_reg[39]_i_1_n_4 ;
  wire \m_axis_tdata_reg[39]_i_1_n_5 ;
  wire \m_axis_tdata_reg[39]_i_1_n_6 ;
  wire \m_axis_tdata_reg[39]_i_1_n_7 ;
  wire \m_axis_tdata_reg[43]_i_1_n_0 ;
  wire \m_axis_tdata_reg[43]_i_1_n_1 ;
  wire \m_axis_tdata_reg[43]_i_1_n_2 ;
  wire \m_axis_tdata_reg[43]_i_1_n_3 ;
  wire \m_axis_tdata_reg[43]_i_1_n_4 ;
  wire \m_axis_tdata_reg[43]_i_1_n_5 ;
  wire \m_axis_tdata_reg[43]_i_1_n_6 ;
  wire \m_axis_tdata_reg[43]_i_1_n_7 ;
  wire \m_axis_tdata_reg[47]_i_1_n_1 ;
  wire \m_axis_tdata_reg[47]_i_1_n_2 ;
  wire \m_axis_tdata_reg[47]_i_1_n_3 ;
  wire \m_axis_tdata_reg[47]_i_1_n_4 ;
  wire \m_axis_tdata_reg[47]_i_1_n_5 ;
  wire \m_axis_tdata_reg[47]_i_1_n_6 ;
  wire \m_axis_tdata_reg[47]_i_1_n_7 ;
  wire \m_axis_tdata_reg[83]_i_1_n_0 ;
  wire \m_axis_tdata_reg[83]_i_1_n_1 ;
  wire \m_axis_tdata_reg[83]_i_1_n_2 ;
  wire \m_axis_tdata_reg[83]_i_1_n_3 ;
  wire \m_axis_tdata_reg[83]_i_1_n_4 ;
  wire \m_axis_tdata_reg[83]_i_1_n_5 ;
  wire \m_axis_tdata_reg[83]_i_1_n_6 ;
  wire \m_axis_tdata_reg[83]_i_1_n_7 ;
  wire \m_axis_tdata_reg[87]_i_1_n_0 ;
  wire \m_axis_tdata_reg[87]_i_1_n_1 ;
  wire \m_axis_tdata_reg[87]_i_1_n_2 ;
  wire \m_axis_tdata_reg[87]_i_1_n_3 ;
  wire \m_axis_tdata_reg[87]_i_1_n_4 ;
  wire \m_axis_tdata_reg[87]_i_1_n_5 ;
  wire \m_axis_tdata_reg[87]_i_1_n_6 ;
  wire \m_axis_tdata_reg[87]_i_1_n_7 ;
  wire \m_axis_tdata_reg[91]_i_1_n_0 ;
  wire \m_axis_tdata_reg[91]_i_1_n_1 ;
  wire \m_axis_tdata_reg[91]_i_1_n_2 ;
  wire \m_axis_tdata_reg[91]_i_1_n_3 ;
  wire \m_axis_tdata_reg[91]_i_1_n_4 ;
  wire \m_axis_tdata_reg[91]_i_1_n_5 ;
  wire \m_axis_tdata_reg[91]_i_1_n_6 ;
  wire \m_axis_tdata_reg[91]_i_1_n_7 ;
  wire \m_axis_tdata_reg[95]_i_1_n_1 ;
  wire \m_axis_tdata_reg[95]_i_1_n_2 ;
  wire \m_axis_tdata_reg[95]_i_1_n_3 ;
  wire \m_axis_tdata_reg[95]_i_1_n_4 ;
  wire \m_axis_tdata_reg[95]_i_1_n_5 ;
  wire \m_axis_tdata_reg[95]_i_1_n_6 ;
  wire \m_axis_tdata_reg[95]_i_1_n_7 ;
  wire \m_axis_tdata_reg[99]_i_1_n_0 ;
  wire \m_axis_tdata_reg[99]_i_1_n_1 ;
  wire \m_axis_tdata_reg[99]_i_1_n_2 ;
  wire \m_axis_tdata_reg[99]_i_1_n_3 ;
  wire m_axis_tready;
  wire m_axis_tvalid;
  wire m_axis_tvalid_i_1_n_0;
  wire mpu_int;
  wire \raw_bytes[0][7]_i_1_n_0 ;
  wire \raw_bytes[10][7]_i_1_n_0 ;
  wire \raw_bytes[11][7]_i_1_n_0 ;
  wire \raw_bytes[12][7]_i_1_n_0 ;
  wire \raw_bytes[13][7]_i_1_n_0 ;
  wire \raw_bytes[1][7]_i_1_n_0 ;
  wire \raw_bytes[1][7]_i_2_n_0 ;
  wire \raw_bytes[2][7]_i_1_n_0 ;
  wire \raw_bytes[3][7]_i_1_n_0 ;
  wire \raw_bytes[4][7]_i_1_n_0 ;
  wire \raw_bytes[5][7]_i_1_n_0 ;
  wire \raw_bytes[6][7]_i_1_n_0 ;
  wire \raw_bytes[7][7]_i_1_n_0 ;
  wire \raw_bytes[8][7]_i_1_n_0 ;
  wire \raw_bytes[9][7]_i_1_n_0 ;
  wire [7:0]\raw_bytes_reg[0] ;
  wire [7:0]\raw_bytes_reg[10] ;
  wire [7:0]\raw_bytes_reg[11] ;
  wire [7:0]\raw_bytes_reg[12] ;
  wire [7:0]\raw_bytes_reg[13] ;
  wire [7:0]\raw_bytes_reg[1] ;
  wire [7:0]\raw_bytes_reg[2] ;
  wire [7:0]\raw_bytes_reg[3] ;
  wire [7:0]\raw_bytes_reg[4] ;
  wire [7:0]\raw_bytes_reg[5] ;
  wire [7:0]\raw_bytes_reg[6] ;
  wire [7:0]\raw_bytes_reg[7] ;
  wire [7:0]\raw_bytes_reg[8] ;
  wire [7:0]\raw_bytes_reg[9] ;
  wire reset_n;
  wire rw;
  wire [2:0]state;
  wire NLW_Inst_i2c_master_ack_error_UNCONNECTED;
  wire [3:3]\NLW_m_axis_tdata_reg[111]_i_2_CO_UNCONNECTED ;
  wire [3:3]\NLW_m_axis_tdata_reg[31]_i_1_CO_UNCONNECTED ;
  wire [3:3]\NLW_m_axis_tdata_reg[47]_i_1_CO_UNCONNECTED ;
  wire [3:3]\NLW_m_axis_tdata_reg[95]_i_1_CO_UNCONNECTED ;

  LUT6 #(
    .INIT(64'hEE0CEE0CFF773377)) 
    \FSM_sequential_state[0]_i_1 
       (.I0(i2c_busy),
        .I1(state[1]),
        .I2(int_edge),
        .I3(state[2]),
        .I4(m_axis_tready),
        .I5(state[0]),
        .O(\FSM_sequential_state[0]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'hFFAAFFFFFFAE0000)) 
    \FSM_sequential_state[1]_i_1 
       (.I0(\FSM_sequential_state[1]_i_2_n_0 ),
        .I1(state[0]),
        .I2(\FSM_sequential_state[1]_i_3_n_0 ),
        .I3(\FSM_sequential_state[1]_i_4_n_0 ),
        .I4(\FSM_sequential_state[1]_i_5_n_0 ),
        .I5(state[1]),
        .O(\FSM_sequential_state[1]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair3" *) 
  LUT5 #(
    .INIT(32'h0055EA00)) 
    \FSM_sequential_state[1]_i_2 
       (.I0(state[2]),
        .I1(\init_step_reg_n_0_[2] ),
        .I2(\init_step_reg_n_0_[1] ),
        .I3(state[1]),
        .I4(state[0]),
        .O(\FSM_sequential_state[1]_i_2_n_0 ));
  LUT3 #(
    .INIT(8'h7F)) 
    \FSM_sequential_state[1]_i_3 
       (.I0(\byte_idx_reg_n_0_[3] ),
        .I1(\byte_idx_reg_n_0_[1] ),
        .I2(\byte_idx_reg_n_0_[2] ),
        .O(\FSM_sequential_state[1]_i_3_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair5" *) 
  LUT4 #(
    .INIT(16'h0800)) 
    \FSM_sequential_state[1]_i_4 
       (.I0(\init_step_reg_n_0_[2] ),
        .I1(\init_step_reg_n_0_[0] ),
        .I2(state[0]),
        .I3(state[1]),
        .O(\FSM_sequential_state[1]_i_4_n_0 ));
  LUT6 #(
    .INIT(64'h308833FF30BBFFFF)) 
    \FSM_sequential_state[1]_i_5 
       (.I0(m_axis_tready),
        .I1(state[2]),
        .I2(int_edge),
        .I3(state[0]),
        .I4(state[1]),
        .I5(i2c_busy),
        .O(\FSM_sequential_state[1]_i_5_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \FSM_sequential_state[2]_i_1 
       (.I0(reset_n),
        .O(\FSM_sequential_state[2]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'hFC44CCCC)) 
    \FSM_sequential_state[2]_i_2 
       (.I0(m_axis_tready),
        .I1(state[2]),
        .I2(int_edge),
        .I3(state[0]),
        .I4(state[1]),
        .O(\FSM_sequential_state[2]_i_2_n_0 ));
  (* FSM_ENCODED_STATES = "s_reset:000,s_init_wait:010,s_read_wait:101,s_wait_int:011,s_init_start:001,s_stream_out:110,s_read_start:100" *) 
  FDRE #(
    .INIT(1'b0)) 
    \FSM_sequential_state_reg[0] 
       (.C(clk),
        .CE(1'b1),
        .D(\FSM_sequential_state[0]_i_1_n_0 ),
        .Q(state[0]),
        .R(\FSM_sequential_state[2]_i_1_n_0 ));
  (* FSM_ENCODED_STATES = "s_reset:000,s_init_wait:010,s_read_wait:101,s_wait_int:011,s_init_start:001,s_stream_out:110,s_read_start:100" *) 
  FDRE #(
    .INIT(1'b0)) 
    \FSM_sequential_state_reg[1] 
       (.C(clk),
        .CE(1'b1),
        .D(\FSM_sequential_state[1]_i_1_n_0 ),
        .Q(state[1]),
        .R(\FSM_sequential_state[2]_i_1_n_0 ));
  (* FSM_ENCODED_STATES = "s_reset:000,s_init_wait:010,s_read_wait:101,s_wait_int:011,s_init_start:001,s_stream_out:110,s_read_start:100" *) 
  FDRE #(
    .INIT(1'b0)) 
    \FSM_sequential_state_reg[2] 
       (.C(clk),
        .CE(1'b1),
        .D(\FSM_sequential_state[2]_i_2_n_0 ),
        .Q(state[2]),
        .R(\FSM_sequential_state[2]_i_1_n_0 ));
  (* BUS_CLK = "400000" *) 
  (* INPUT_CLK = "100000000" *) 
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_i2c_master Inst_i2c_master
       (.ack_error(NLW_Inst_i2c_master_ack_error_UNCONNECTED),
        .addr({1'b1,1'b1,1'b0,1'b1,1'b0,1'b0,1'b0}),
        .busy(i2c_busy),
        .clk(clk),
        .data_rd(i2c_data_rd),
        .data_wr({1'b0,data_wr}),
        .ena(ena),
        .reset_n(reset_n),
        .rw(rw),
        .scl(i2c_scl),
        .sda(i2c_sda));
  LUT2 #(
    .INIT(4'h1)) 
    \byte_idx[0]_i_1 
       (.I0(state[1]),
        .I1(\byte_idx_reg_n_0_[0] ),
        .O(byte_idx[0]));
  (* SOFT_HLUTNM = "soft_lutpair6" *) 
  LUT3 #(
    .INIT(8'h14)) 
    \byte_idx[1]_i_1 
       (.I0(state[1]),
        .I1(\byte_idx_reg_n_0_[0] ),
        .I2(\byte_idx_reg_n_0_[1] ),
        .O(byte_idx[1]));
  (* SOFT_HLUTNM = "soft_lutpair6" *) 
  LUT4 #(
    .INIT(16'h0078)) 
    \byte_idx[2]_i_1 
       (.I0(\byte_idx_reg_n_0_[0] ),
        .I1(\byte_idx_reg_n_0_[1] ),
        .I2(\byte_idx_reg_n_0_[2] ),
        .I3(state[1]),
        .O(byte_idx[2]));
  LUT6 #(
    .INIT(64'h04F0000004000000)) 
    \byte_idx[3]_i_1 
       (.I0(i2c_busy),
        .I1(\FSM_sequential_state[1]_i_3_n_0 ),
        .I2(state[1]),
        .I3(state[2]),
        .I4(\byte_idx[3]_i_3_n_0 ),
        .I5(int_edge),
        .O(\byte_idx[3]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair4" *) 
  LUT5 #(
    .INIT(32'h00007F80)) 
    \byte_idx[3]_i_2 
       (.I0(\byte_idx_reg_n_0_[1] ),
        .I1(\byte_idx_reg_n_0_[0] ),
        .I2(\byte_idx_reg_n_0_[2] ),
        .I3(\byte_idx_reg_n_0_[3] ),
        .I4(state[1]),
        .O(byte_idx[3]));
  (* SOFT_HLUTNM = "soft_lutpair8" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \byte_idx[3]_i_3 
       (.I0(reset_n),
        .I1(state[0]),
        .O(\byte_idx[3]_i_3_n_0 ));
  FDRE #(
    .INIT(1'b0)) 
    \byte_idx_reg[0] 
       (.C(clk),
        .CE(\byte_idx[3]_i_1_n_0 ),
        .D(byte_idx[0]),
        .Q(\byte_idx_reg_n_0_[0] ),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \byte_idx_reg[1] 
       (.C(clk),
        .CE(\byte_idx[3]_i_1_n_0 ),
        .D(byte_idx[1]),
        .Q(\byte_idx_reg_n_0_[1] ),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \byte_idx_reg[2] 
       (.C(clk),
        .CE(\byte_idx[3]_i_1_n_0 ),
        .D(byte_idx[2]),
        .Q(\byte_idx_reg_n_0_[2] ),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \byte_idx_reg[3] 
       (.C(clk),
        .CE(\byte_idx[3]_i_1_n_0 ),
        .D(byte_idx[3]),
        .Q(\byte_idx_reg_n_0_[3] ),
        .R(1'b0));
  (* SOFT_HLUTNM = "soft_lutpair3" *) 
  LUT2 #(
    .INIT(4'h1)) 
    \i2c_data_wr[0]_i_1 
       (.I0(\init_step_reg_n_0_[2] ),
        .I1(state[2]),
        .O(\i2c_data_wr[0]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'hAAEFAA20)) 
    \i2c_data_wr[1]_i_1 
       (.I0(i2c_data_wr[1]),
        .I1(\i2c_data_wr[5]_i_4_n_0 ),
        .I2(\i2c_data_wr[6]_i_3_n_0 ),
        .I3(\i2c_data_wr[6]_i_4_n_0 ),
        .I4(data_wr[1]),
        .O(\i2c_data_wr[1]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair7" *) 
  LUT4 #(
    .INIT(16'h55D7)) 
    \i2c_data_wr[1]_i_2 
       (.I0(state[0]),
        .I1(\init_step_reg_n_0_[1] ),
        .I2(\init_step_reg_n_0_[0] ),
        .I3(state[2]),
        .O(i2c_data_wr[1]));
  LUT6 #(
    .INIT(64'h8888F8FF88880800)) 
    \i2c_data_wr[2]_i_1 
       (.I0(\i2c_data_wr[2]_i_2_n_0 ),
        .I1(\i2c_data_wr[6]_i_2_n_0 ),
        .I2(\i2c_data_wr[5]_i_4_n_0 ),
        .I3(\i2c_data_wr[6]_i_3_n_0 ),
        .I4(\i2c_data_wr[6]_i_4_n_0 ),
        .I5(data_wr[2]),
        .O(\i2c_data_wr[2]_i_1_n_0 ));
  LUT2 #(
    .INIT(4'h2)) 
    \i2c_data_wr[2]_i_2 
       (.I0(\init_step_reg_n_0_[0] ),
        .I1(state[2]),
        .O(\i2c_data_wr[2]_i_2_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair0" *) 
  LUT2 #(
    .INIT(4'h1)) 
    \i2c_data_wr[3]_i_1 
       (.I0(\init_step_reg_n_0_[0] ),
        .I1(state[2]),
        .O(\i2c_data_wr[3]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair1" *) 
  LUT4 #(
    .INIT(16'h000E)) 
    \i2c_data_wr[4]_i_1 
       (.I0(\init_step_reg_n_0_[1] ),
        .I1(\init_step_reg_n_0_[2] ),
        .I2(state[2]),
        .I3(\init_step_reg_n_0_[0] ),
        .O(\i2c_data_wr[4]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'h00000020)) 
    \i2c_data_wr[5]_i_1 
       (.I0(reset_n),
        .I1(state[1]),
        .I2(state[2]),
        .I3(\i2c_data_wr[5]_i_4_n_0 ),
        .I4(state[0]),
        .O(\i2c_data_wr[5]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'h000C005000000000)) 
    \i2c_data_wr[5]_i_2 
       (.I0(\i2c_data_wr[5]_i_4_n_0 ),
        .I1(\i2c_data_wr[5]_i_5_n_0 ),
        .I2(state[2]),
        .I3(state[1]),
        .I4(state[0]),
        .I5(reset_n),
        .O(\i2c_data_wr[5]_i_2_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair7" *) 
  LUT4 #(
    .INIT(16'h0001)) 
    \i2c_data_wr[5]_i_3 
       (.I0(state[2]),
        .I1(\init_step_reg_n_0_[0] ),
        .I2(\init_step_reg_n_0_[1] ),
        .I3(\init_step_reg_n_0_[2] ),
        .O(i2c_data_wr[6]));
  (* SOFT_HLUTNM = "soft_lutpair4" *) 
  LUT4 #(
    .INIT(16'hFFFE)) 
    \i2c_data_wr[5]_i_4 
       (.I0(\byte_idx_reg_n_0_[1] ),
        .I1(\byte_idx_reg_n_0_[0] ),
        .I2(\byte_idx_reg_n_0_[3] ),
        .I3(\byte_idx_reg_n_0_[2] ),
        .O(\i2c_data_wr[5]_i_4_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair5" *) 
  LUT2 #(
    .INIT(4'h7)) 
    \i2c_data_wr[5]_i_5 
       (.I0(\init_step_reg_n_0_[2] ),
        .I1(\init_step_reg_n_0_[1] ),
        .O(\i2c_data_wr[5]_i_5_n_0 ));
  LUT6 #(
    .INIT(64'h2222F2FF22220200)) 
    \i2c_data_wr[6]_i_1 
       (.I0(\i2c_data_wr[3]_i_1_n_0 ),
        .I1(\i2c_data_wr[6]_i_2_n_0 ),
        .I2(\i2c_data_wr[5]_i_4_n_0 ),
        .I3(\i2c_data_wr[6]_i_3_n_0 ),
        .I4(\i2c_data_wr[6]_i_4_n_0 ),
        .I5(data_wr[6]),
        .O(\i2c_data_wr[6]_i_1_n_0 ));
  LUT2 #(
    .INIT(4'hE)) 
    \i2c_data_wr[6]_i_2 
       (.I0(\init_step_reg_n_0_[2] ),
        .I1(\init_step_reg_n_0_[1] ),
        .O(\i2c_data_wr[6]_i_2_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair2" *) 
  LUT4 #(
    .INIT(16'h0400)) 
    \i2c_data_wr[6]_i_3 
       (.I0(state[0]),
        .I1(reset_n),
        .I2(state[1]),
        .I3(state[2]),
        .O(\i2c_data_wr[6]_i_3_n_0 ));
  LUT6 #(
    .INIT(64'h0000000800080008)) 
    \i2c_data_wr[6]_i_4 
       (.I0(reset_n),
        .I1(state[0]),
        .I2(state[1]),
        .I3(state[2]),
        .I4(\init_step_reg_n_0_[1] ),
        .I5(\init_step_reg_n_0_[2] ),
        .O(\i2c_data_wr[6]_i_4_n_0 ));
  FDSE #(
    .INIT(1'b0)) 
    \i2c_data_wr_reg[0] 
       (.C(clk),
        .CE(\i2c_data_wr[5]_i_2_n_0 ),
        .D(\i2c_data_wr[0]_i_1_n_0 ),
        .Q(data_wr[0]),
        .S(\i2c_data_wr[5]_i_1_n_0 ));
  FDRE #(
    .INIT(1'b0)) 
    \i2c_data_wr_reg[1] 
       (.C(clk),
        .CE(1'b1),
        .D(\i2c_data_wr[1]_i_1_n_0 ),
        .Q(data_wr[1]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \i2c_data_wr_reg[2] 
       (.C(clk),
        .CE(1'b1),
        .D(\i2c_data_wr[2]_i_1_n_0 ),
        .Q(data_wr[2]),
        .R(1'b0));
  FDSE #(
    .INIT(1'b0)) 
    \i2c_data_wr_reg[3] 
       (.C(clk),
        .CE(\i2c_data_wr[5]_i_2_n_0 ),
        .D(\i2c_data_wr[3]_i_1_n_0 ),
        .Q(data_wr[3]),
        .S(\i2c_data_wr[5]_i_1_n_0 ));
  FDSE #(
    .INIT(1'b0)) 
    \i2c_data_wr_reg[4] 
       (.C(clk),
        .CE(\i2c_data_wr[5]_i_2_n_0 ),
        .D(\i2c_data_wr[4]_i_1_n_0 ),
        .Q(data_wr[4]),
        .S(\i2c_data_wr[5]_i_1_n_0 ));
  FDSE #(
    .INIT(1'b0)) 
    \i2c_data_wr_reg[5] 
       (.C(clk),
        .CE(\i2c_data_wr[5]_i_2_n_0 ),
        .D(i2c_data_wr[6]),
        .Q(data_wr[5]),
        .S(\i2c_data_wr[5]_i_1_n_0 ));
  FDRE #(
    .INIT(1'b0)) 
    \i2c_data_wr_reg[6] 
       (.C(clk),
        .CE(1'b1),
        .D(\i2c_data_wr[6]_i_1_n_0 ),
        .Q(data_wr[6]),
        .R(1'b0));
  LUT6 #(
    .INIT(64'hAAA8B3EE00000000)) 
    i2c_ena_i_1
       (.I0(ena),
        .I1(state[0]),
        .I2(i2c_busy),
        .I3(state[2]),
        .I4(state[1]),
        .I5(reset_n),
        .O(i2c_ena_i_1_n_0));
  FDRE #(
    .INIT(1'b0)) 
    i2c_ena_reg
       (.C(clk),
        .CE(1'b1),
        .D(i2c_ena_i_1_n_0),
        .Q(ena),
        .R(1'b0));
  LUT6 #(
    .INIT(64'hFFFBFF3F00080000)) 
    i2c_rw_i_1
       (.I0(\i2c_data_wr[5]_i_4_n_0 ),
        .I1(reset_n),
        .I2(state[0]),
        .I3(state[1]),
        .I4(state[2]),
        .I5(rw),
        .O(i2c_rw_i_1_n_0));
  FDRE #(
    .INIT(1'b0)) 
    i2c_rw_reg
       (.C(clk),
        .CE(1'b1),
        .D(i2c_rw_i_1_n_0),
        .Q(rw),
        .R(1'b0));
  (* SOFT_HLUTNM = "soft_lutpair0" *) 
  LUT5 #(
    .INIT(32'h555400A0)) 
    \init_step[0]_i_1 
       (.I0(\init_step[2]_i_3_n_0 ),
        .I1(state[0]),
        .I2(state[1]),
        .I3(state[2]),
        .I4(\init_step_reg_n_0_[0] ),
        .O(\init_step[0]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'h3333773000008800)) 
    \init_step[1]_i_1 
       (.I0(\init_step_reg_n_0_[0] ),
        .I1(\init_step[2]_i_3_n_0 ),
        .I2(state[0]),
        .I3(state[1]),
        .I4(state[2]),
        .I5(\init_step_reg_n_0_[1] ),
        .O(\init_step[1]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'hBBBBBBBA8888888A)) 
    \init_step[2]_i_1 
       (.I0(init_step),
        .I1(\init_step[2]_i_3_n_0 ),
        .I2(state[0]),
        .I3(state[1]),
        .I4(state[2]),
        .I5(\init_step_reg_n_0_[2] ),
        .O(\init_step[2]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair1" *) 
  LUT5 #(
    .INIT(32'h04080C00)) 
    \init_step[2]_i_2 
       (.I0(\init_step_reg_n_0_[0] ),
        .I1(state[1]),
        .I2(state[2]),
        .I3(\init_step_reg_n_0_[2] ),
        .I4(\init_step_reg_n_0_[1] ),
        .O(init_step));
  LUT6 #(
    .INIT(64'h0001000100010011)) 
    \init_step[2]_i_3 
       (.I0(i2c_busy),
        .I1(state[0]),
        .I2(\init_step_reg_n_0_[2] ),
        .I3(state[2]),
        .I4(\init_step_reg_n_0_[0] ),
        .I5(\init_step_reg_n_0_[1] ),
        .O(\init_step[2]_i_3_n_0 ));
  FDRE #(
    .INIT(1'b0)) 
    \init_step_reg[0] 
       (.C(clk),
        .CE(1'b1),
        .D(\init_step[0]_i_1_n_0 ),
        .Q(\init_step_reg_n_0_[0] ),
        .R(\FSM_sequential_state[2]_i_1_n_0 ));
  FDRE #(
    .INIT(1'b0)) 
    \init_step_reg[1] 
       (.C(clk),
        .CE(1'b1),
        .D(\init_step[1]_i_1_n_0 ),
        .Q(\init_step_reg_n_0_[1] ),
        .R(\FSM_sequential_state[2]_i_1_n_0 ));
  FDRE #(
    .INIT(1'b0)) 
    \init_step_reg[2] 
       (.C(clk),
        .CE(1'b1),
        .D(\init_step[2]_i_1_n_0 ),
        .Q(\init_step_reg_n_0_[2] ),
        .R(\FSM_sequential_state[2]_i_1_n_0 ));
  LUT2 #(
    .INIT(4'h2)) 
    int_edge_i_1
       (.I0(int_q1),
        .I1(int_q2),
        .O(int_edge0));
  FDRE #(
    .INIT(1'b0)) 
    int_edge_reg
       (.C(clk),
        .CE(1'b1),
        .D(int_edge0),
        .Q(int_edge),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    int_q1_reg
       (.C(clk),
        .CE(1'b1),
        .D(mpu_int),
        .Q(int_q1),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    int_q2_reg
       (.C(clk),
        .CE(1'b1),
        .D(int_q1),
        .Q(int_q2),
        .R(1'b0));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[103]_i_2 
       (.I0(\raw_bytes_reg[13] [7]),
        .O(\m_axis_tdata[103]_i_2_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[103]_i_3 
       (.I0(\raw_bytes_reg[13] [6]),
        .O(\m_axis_tdata[103]_i_3_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[103]_i_4 
       (.I0(\raw_bytes_reg[13] [5]),
        .O(\m_axis_tdata[103]_i_4_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[103]_i_5 
       (.I0(\raw_bytes_reg[13] [4]),
        .O(\m_axis_tdata[103]_i_5_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[107]_i_2 
       (.I0(\raw_bytes_reg[12] [3]),
        .O(\m_axis_tdata[107]_i_2_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[107]_i_3 
       (.I0(\raw_bytes_reg[12] [2]),
        .O(\m_axis_tdata[107]_i_3_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[107]_i_4 
       (.I0(\raw_bytes_reg[12] [1]),
        .O(\m_axis_tdata[107]_i_4_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[107]_i_5 
       (.I0(\raw_bytes_reg[12] [0]),
        .O(\m_axis_tdata[107]_i_5_n_0 ));
  LUT4 #(
    .INIT(16'h0800)) 
    \m_axis_tdata[111]_i_1 
       (.I0(reset_n),
        .I1(state[2]),
        .I2(state[0]),
        .I3(state[1]),
        .O(\m_axis_tdata[111]_i_1_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[111]_i_3 
       (.I0(\raw_bytes_reg[12] [7]),
        .O(\m_axis_tdata[111]_i_3_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[111]_i_4 
       (.I0(\raw_bytes_reg[12] [6]),
        .O(\m_axis_tdata[111]_i_4_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[111]_i_5 
       (.I0(\raw_bytes_reg[12] [5]),
        .O(\m_axis_tdata[111]_i_5_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[111]_i_6 
       (.I0(\raw_bytes_reg[12] [4]),
        .O(\m_axis_tdata[111]_i_6_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[19]_i_2 
       (.I0(\raw_bytes_reg[3] [3]),
        .O(\m_axis_tdata[19]_i_2_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[19]_i_3 
       (.I0(\raw_bytes_reg[3] [2]),
        .O(\m_axis_tdata[19]_i_3_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[19]_i_4 
       (.I0(\raw_bytes_reg[3] [1]),
        .O(\m_axis_tdata[19]_i_4_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[23]_i_2 
       (.I0(\raw_bytes_reg[3] [7]),
        .O(\m_axis_tdata[23]_i_2_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[23]_i_3 
       (.I0(\raw_bytes_reg[3] [6]),
        .O(\m_axis_tdata[23]_i_3_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[23]_i_4 
       (.I0(\raw_bytes_reg[3] [5]),
        .O(\m_axis_tdata[23]_i_4_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[23]_i_5 
       (.I0(\raw_bytes_reg[3] [4]),
        .O(\m_axis_tdata[23]_i_5_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[27]_i_2 
       (.I0(\raw_bytes_reg[2] [3]),
        .O(\m_axis_tdata[27]_i_2_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[27]_i_3 
       (.I0(\raw_bytes_reg[2] [2]),
        .O(\m_axis_tdata[27]_i_3_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[27]_i_4 
       (.I0(\raw_bytes_reg[2] [1]),
        .O(\m_axis_tdata[27]_i_4_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[27]_i_5 
       (.I0(\raw_bytes_reg[2] [0]),
        .O(\m_axis_tdata[27]_i_5_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[31]_i_2 
       (.I0(\raw_bytes_reg[2] [7]),
        .O(\m_axis_tdata[31]_i_2_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[31]_i_3 
       (.I0(\raw_bytes_reg[2] [6]),
        .O(\m_axis_tdata[31]_i_3_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[31]_i_4 
       (.I0(\raw_bytes_reg[2] [5]),
        .O(\m_axis_tdata[31]_i_4_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[31]_i_5 
       (.I0(\raw_bytes_reg[2] [4]),
        .O(\m_axis_tdata[31]_i_5_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[35]_i_2 
       (.I0(\raw_bytes_reg[5] [3]),
        .O(\m_axis_tdata[35]_i_2_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[35]_i_3 
       (.I0(\raw_bytes_reg[5] [2]),
        .O(\m_axis_tdata[35]_i_3_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[35]_i_4 
       (.I0(\raw_bytes_reg[5] [1]),
        .O(\m_axis_tdata[35]_i_4_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[39]_i_2 
       (.I0(\raw_bytes_reg[5] [7]),
        .O(\m_axis_tdata[39]_i_2_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[39]_i_3 
       (.I0(\raw_bytes_reg[5] [6]),
        .O(\m_axis_tdata[39]_i_3_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[39]_i_4 
       (.I0(\raw_bytes_reg[5] [5]),
        .O(\m_axis_tdata[39]_i_4_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[39]_i_5 
       (.I0(\raw_bytes_reg[5] [4]),
        .O(\m_axis_tdata[39]_i_5_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[43]_i_2 
       (.I0(\raw_bytes_reg[4] [3]),
        .O(\m_axis_tdata[43]_i_2_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[43]_i_3 
       (.I0(\raw_bytes_reg[4] [2]),
        .O(\m_axis_tdata[43]_i_3_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[43]_i_4 
       (.I0(\raw_bytes_reg[4] [1]),
        .O(\m_axis_tdata[43]_i_4_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[43]_i_5 
       (.I0(\raw_bytes_reg[4] [0]),
        .O(\m_axis_tdata[43]_i_5_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[47]_i_2 
       (.I0(\raw_bytes_reg[4] [7]),
        .O(\m_axis_tdata[47]_i_2_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[47]_i_3 
       (.I0(\raw_bytes_reg[4] [6]),
        .O(\m_axis_tdata[47]_i_3_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[47]_i_4 
       (.I0(\raw_bytes_reg[4] [5]),
        .O(\m_axis_tdata[47]_i_4_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[47]_i_5 
       (.I0(\raw_bytes_reg[4] [4]),
        .O(\m_axis_tdata[47]_i_5_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[83]_i_2 
       (.I0(\raw_bytes_reg[11] [3]),
        .O(\m_axis_tdata[83]_i_2_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[83]_i_3 
       (.I0(\raw_bytes_reg[11] [2]),
        .O(\m_axis_tdata[83]_i_3_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[83]_i_4 
       (.I0(\raw_bytes_reg[11] [1]),
        .O(\m_axis_tdata[83]_i_4_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[87]_i_2 
       (.I0(\raw_bytes_reg[11] [7]),
        .O(\m_axis_tdata[87]_i_2_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[87]_i_3 
       (.I0(\raw_bytes_reg[11] [6]),
        .O(\m_axis_tdata[87]_i_3_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[87]_i_4 
       (.I0(\raw_bytes_reg[11] [5]),
        .O(\m_axis_tdata[87]_i_4_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[87]_i_5 
       (.I0(\raw_bytes_reg[11] [4]),
        .O(\m_axis_tdata[87]_i_5_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[91]_i_2 
       (.I0(\raw_bytes_reg[10] [3]),
        .O(\m_axis_tdata[91]_i_2_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[91]_i_3 
       (.I0(\raw_bytes_reg[10] [2]),
        .O(\m_axis_tdata[91]_i_3_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[91]_i_4 
       (.I0(\raw_bytes_reg[10] [1]),
        .O(\m_axis_tdata[91]_i_4_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[91]_i_5 
       (.I0(\raw_bytes_reg[10] [0]),
        .O(\m_axis_tdata[91]_i_5_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[95]_i_2 
       (.I0(\raw_bytes_reg[10] [7]),
        .O(\m_axis_tdata[95]_i_2_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[95]_i_3 
       (.I0(\raw_bytes_reg[10] [6]),
        .O(\m_axis_tdata[95]_i_3_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[95]_i_4 
       (.I0(\raw_bytes_reg[10] [5]),
        .O(\m_axis_tdata[95]_i_4_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[95]_i_5 
       (.I0(\raw_bytes_reg[10] [4]),
        .O(\m_axis_tdata[95]_i_5_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[99]_i_2 
       (.I0(\raw_bytes_reg[13] [3]),
        .O(\m_axis_tdata[99]_i_2_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[99]_i_3 
       (.I0(\raw_bytes_reg[13] [2]),
        .O(\m_axis_tdata[99]_i_3_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[99]_i_4 
       (.I0(\raw_bytes_reg[13] [1]),
        .O(\m_axis_tdata[99]_i_4_n_0 ));
  FDRE \m_axis_tdata_reg[0] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[1] [0]),
        .Q(m_axis_tdata[0]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[100] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(m_axis_tdata0[4]),
        .Q(m_axis_tdata[100]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[101] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(m_axis_tdata0[5]),
        .Q(m_axis_tdata[101]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[102] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(m_axis_tdata0[6]),
        .Q(m_axis_tdata[102]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[103] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(m_axis_tdata0[7]),
        .Q(m_axis_tdata[103]),
        .R(1'b0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \m_axis_tdata_reg[103]_i_1 
       (.CI(\m_axis_tdata_reg[99]_i_1_n_0 ),
        .CO({\m_axis_tdata_reg[103]_i_1_n_0 ,\m_axis_tdata_reg[103]_i_1_n_1 ,\m_axis_tdata_reg[103]_i_1_n_2 ,\m_axis_tdata_reg[103]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(m_axis_tdata0[7:4]),
        .S({\m_axis_tdata[103]_i_2_n_0 ,\m_axis_tdata[103]_i_3_n_0 ,\m_axis_tdata[103]_i_4_n_0 ,\m_axis_tdata[103]_i_5_n_0 }));
  FDRE \m_axis_tdata_reg[104] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(m_axis_tdata0[8]),
        .Q(m_axis_tdata[104]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[105] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(m_axis_tdata0[9]),
        .Q(m_axis_tdata[105]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[106] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(m_axis_tdata0[10]),
        .Q(m_axis_tdata[106]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[107] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(m_axis_tdata0[11]),
        .Q(m_axis_tdata[107]),
        .R(1'b0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \m_axis_tdata_reg[107]_i_1 
       (.CI(\m_axis_tdata_reg[103]_i_1_n_0 ),
        .CO({\m_axis_tdata_reg[107]_i_1_n_0 ,\m_axis_tdata_reg[107]_i_1_n_1 ,\m_axis_tdata_reg[107]_i_1_n_2 ,\m_axis_tdata_reg[107]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(m_axis_tdata0[11:8]),
        .S({\m_axis_tdata[107]_i_2_n_0 ,\m_axis_tdata[107]_i_3_n_0 ,\m_axis_tdata[107]_i_4_n_0 ,\m_axis_tdata[107]_i_5_n_0 }));
  FDRE \m_axis_tdata_reg[108] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(m_axis_tdata0[12]),
        .Q(m_axis_tdata[108]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[109] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(m_axis_tdata0[13]),
        .Q(m_axis_tdata[109]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[10] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[0] [2]),
        .Q(m_axis_tdata[10]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[110] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(m_axis_tdata0[14]),
        .Q(m_axis_tdata[110]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[111] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(m_axis_tdata0[15]),
        .Q(m_axis_tdata[111]),
        .R(1'b0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \m_axis_tdata_reg[111]_i_2 
       (.CI(\m_axis_tdata_reg[107]_i_1_n_0 ),
        .CO({\NLW_m_axis_tdata_reg[111]_i_2_CO_UNCONNECTED [3],\m_axis_tdata_reg[111]_i_2_n_1 ,\m_axis_tdata_reg[111]_i_2_n_2 ,\m_axis_tdata_reg[111]_i_2_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(m_axis_tdata0[15:12]),
        .S({\m_axis_tdata[111]_i_3_n_0 ,\m_axis_tdata[111]_i_4_n_0 ,\m_axis_tdata[111]_i_5_n_0 ,\m_axis_tdata[111]_i_6_n_0 }));
  FDRE \m_axis_tdata_reg[11] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[0] [3]),
        .Q(m_axis_tdata[11]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[12] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[0] [4]),
        .Q(m_axis_tdata[12]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[13] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[0] [5]),
        .Q(m_axis_tdata[13]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[14] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[0] [6]),
        .Q(m_axis_tdata[14]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[15] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[0] [7]),
        .Q(m_axis_tdata[15]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[16] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[19]_i_1_n_7 ),
        .Q(m_axis_tdata[16]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[17] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[19]_i_1_n_6 ),
        .Q(m_axis_tdata[17]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[18] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[19]_i_1_n_5 ),
        .Q(m_axis_tdata[18]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[19] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[19]_i_1_n_4 ),
        .Q(m_axis_tdata[19]),
        .R(1'b0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \m_axis_tdata_reg[19]_i_1 
       (.CI(1'b0),
        .CO({\m_axis_tdata_reg[19]_i_1_n_0 ,\m_axis_tdata_reg[19]_i_1_n_1 ,\m_axis_tdata_reg[19]_i_1_n_2 ,\m_axis_tdata_reg[19]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b1}),
        .O({\m_axis_tdata_reg[19]_i_1_n_4 ,\m_axis_tdata_reg[19]_i_1_n_5 ,\m_axis_tdata_reg[19]_i_1_n_6 ,\m_axis_tdata_reg[19]_i_1_n_7 }),
        .S({\m_axis_tdata[19]_i_2_n_0 ,\m_axis_tdata[19]_i_3_n_0 ,\m_axis_tdata[19]_i_4_n_0 ,\raw_bytes_reg[3] [0]}));
  FDRE \m_axis_tdata_reg[1] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[1] [1]),
        .Q(m_axis_tdata[1]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[20] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[23]_i_1_n_7 ),
        .Q(m_axis_tdata[20]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[21] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[23]_i_1_n_6 ),
        .Q(m_axis_tdata[21]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[22] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[23]_i_1_n_5 ),
        .Q(m_axis_tdata[22]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[23] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[23]_i_1_n_4 ),
        .Q(m_axis_tdata[23]),
        .R(1'b0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \m_axis_tdata_reg[23]_i_1 
       (.CI(\m_axis_tdata_reg[19]_i_1_n_0 ),
        .CO({\m_axis_tdata_reg[23]_i_1_n_0 ,\m_axis_tdata_reg[23]_i_1_n_1 ,\m_axis_tdata_reg[23]_i_1_n_2 ,\m_axis_tdata_reg[23]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({\m_axis_tdata_reg[23]_i_1_n_4 ,\m_axis_tdata_reg[23]_i_1_n_5 ,\m_axis_tdata_reg[23]_i_1_n_6 ,\m_axis_tdata_reg[23]_i_1_n_7 }),
        .S({\m_axis_tdata[23]_i_2_n_0 ,\m_axis_tdata[23]_i_3_n_0 ,\m_axis_tdata[23]_i_4_n_0 ,\m_axis_tdata[23]_i_5_n_0 }));
  FDRE \m_axis_tdata_reg[24] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[27]_i_1_n_7 ),
        .Q(m_axis_tdata[24]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[25] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[27]_i_1_n_6 ),
        .Q(m_axis_tdata[25]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[26] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[27]_i_1_n_5 ),
        .Q(m_axis_tdata[26]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[27] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[27]_i_1_n_4 ),
        .Q(m_axis_tdata[27]),
        .R(1'b0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \m_axis_tdata_reg[27]_i_1 
       (.CI(\m_axis_tdata_reg[23]_i_1_n_0 ),
        .CO({\m_axis_tdata_reg[27]_i_1_n_0 ,\m_axis_tdata_reg[27]_i_1_n_1 ,\m_axis_tdata_reg[27]_i_1_n_2 ,\m_axis_tdata_reg[27]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({\m_axis_tdata_reg[27]_i_1_n_4 ,\m_axis_tdata_reg[27]_i_1_n_5 ,\m_axis_tdata_reg[27]_i_1_n_6 ,\m_axis_tdata_reg[27]_i_1_n_7 }),
        .S({\m_axis_tdata[27]_i_2_n_0 ,\m_axis_tdata[27]_i_3_n_0 ,\m_axis_tdata[27]_i_4_n_0 ,\m_axis_tdata[27]_i_5_n_0 }));
  FDRE \m_axis_tdata_reg[28] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[31]_i_1_n_7 ),
        .Q(m_axis_tdata[28]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[29] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[31]_i_1_n_6 ),
        .Q(m_axis_tdata[29]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[2] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[1] [2]),
        .Q(m_axis_tdata[2]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[30] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[31]_i_1_n_5 ),
        .Q(m_axis_tdata[30]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[31] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[31]_i_1_n_4 ),
        .Q(m_axis_tdata[31]),
        .R(1'b0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \m_axis_tdata_reg[31]_i_1 
       (.CI(\m_axis_tdata_reg[27]_i_1_n_0 ),
        .CO({\NLW_m_axis_tdata_reg[31]_i_1_CO_UNCONNECTED [3],\m_axis_tdata_reg[31]_i_1_n_1 ,\m_axis_tdata_reg[31]_i_1_n_2 ,\m_axis_tdata_reg[31]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({\m_axis_tdata_reg[31]_i_1_n_4 ,\m_axis_tdata_reg[31]_i_1_n_5 ,\m_axis_tdata_reg[31]_i_1_n_6 ,\m_axis_tdata_reg[31]_i_1_n_7 }),
        .S({\m_axis_tdata[31]_i_2_n_0 ,\m_axis_tdata[31]_i_3_n_0 ,\m_axis_tdata[31]_i_4_n_0 ,\m_axis_tdata[31]_i_5_n_0 }));
  FDRE \m_axis_tdata_reg[32] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[35]_i_1_n_7 ),
        .Q(m_axis_tdata[32]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[33] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[35]_i_1_n_6 ),
        .Q(m_axis_tdata[33]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[34] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[35]_i_1_n_5 ),
        .Q(m_axis_tdata[34]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[35] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[35]_i_1_n_4 ),
        .Q(m_axis_tdata[35]),
        .R(1'b0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \m_axis_tdata_reg[35]_i_1 
       (.CI(1'b0),
        .CO({\m_axis_tdata_reg[35]_i_1_n_0 ,\m_axis_tdata_reg[35]_i_1_n_1 ,\m_axis_tdata_reg[35]_i_1_n_2 ,\m_axis_tdata_reg[35]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b1}),
        .O({\m_axis_tdata_reg[35]_i_1_n_4 ,\m_axis_tdata_reg[35]_i_1_n_5 ,\m_axis_tdata_reg[35]_i_1_n_6 ,\m_axis_tdata_reg[35]_i_1_n_7 }),
        .S({\m_axis_tdata[35]_i_2_n_0 ,\m_axis_tdata[35]_i_3_n_0 ,\m_axis_tdata[35]_i_4_n_0 ,\raw_bytes_reg[5] [0]}));
  FDRE \m_axis_tdata_reg[36] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[39]_i_1_n_7 ),
        .Q(m_axis_tdata[36]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[37] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[39]_i_1_n_6 ),
        .Q(m_axis_tdata[37]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[38] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[39]_i_1_n_5 ),
        .Q(m_axis_tdata[38]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[39] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[39]_i_1_n_4 ),
        .Q(m_axis_tdata[39]),
        .R(1'b0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \m_axis_tdata_reg[39]_i_1 
       (.CI(\m_axis_tdata_reg[35]_i_1_n_0 ),
        .CO({\m_axis_tdata_reg[39]_i_1_n_0 ,\m_axis_tdata_reg[39]_i_1_n_1 ,\m_axis_tdata_reg[39]_i_1_n_2 ,\m_axis_tdata_reg[39]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({\m_axis_tdata_reg[39]_i_1_n_4 ,\m_axis_tdata_reg[39]_i_1_n_5 ,\m_axis_tdata_reg[39]_i_1_n_6 ,\m_axis_tdata_reg[39]_i_1_n_7 }),
        .S({\m_axis_tdata[39]_i_2_n_0 ,\m_axis_tdata[39]_i_3_n_0 ,\m_axis_tdata[39]_i_4_n_0 ,\m_axis_tdata[39]_i_5_n_0 }));
  FDRE \m_axis_tdata_reg[3] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[1] [3]),
        .Q(m_axis_tdata[3]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[40] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[43]_i_1_n_7 ),
        .Q(m_axis_tdata[40]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[41] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[43]_i_1_n_6 ),
        .Q(m_axis_tdata[41]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[42] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[43]_i_1_n_5 ),
        .Q(m_axis_tdata[42]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[43] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[43]_i_1_n_4 ),
        .Q(m_axis_tdata[43]),
        .R(1'b0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \m_axis_tdata_reg[43]_i_1 
       (.CI(\m_axis_tdata_reg[39]_i_1_n_0 ),
        .CO({\m_axis_tdata_reg[43]_i_1_n_0 ,\m_axis_tdata_reg[43]_i_1_n_1 ,\m_axis_tdata_reg[43]_i_1_n_2 ,\m_axis_tdata_reg[43]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({\m_axis_tdata_reg[43]_i_1_n_4 ,\m_axis_tdata_reg[43]_i_1_n_5 ,\m_axis_tdata_reg[43]_i_1_n_6 ,\m_axis_tdata_reg[43]_i_1_n_7 }),
        .S({\m_axis_tdata[43]_i_2_n_0 ,\m_axis_tdata[43]_i_3_n_0 ,\m_axis_tdata[43]_i_4_n_0 ,\m_axis_tdata[43]_i_5_n_0 }));
  FDRE \m_axis_tdata_reg[44] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[47]_i_1_n_7 ),
        .Q(m_axis_tdata[44]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[45] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[47]_i_1_n_6 ),
        .Q(m_axis_tdata[45]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[46] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[47]_i_1_n_5 ),
        .Q(m_axis_tdata[46]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[47] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[47]_i_1_n_4 ),
        .Q(m_axis_tdata[47]),
        .R(1'b0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \m_axis_tdata_reg[47]_i_1 
       (.CI(\m_axis_tdata_reg[43]_i_1_n_0 ),
        .CO({\NLW_m_axis_tdata_reg[47]_i_1_CO_UNCONNECTED [3],\m_axis_tdata_reg[47]_i_1_n_1 ,\m_axis_tdata_reg[47]_i_1_n_2 ,\m_axis_tdata_reg[47]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({\m_axis_tdata_reg[47]_i_1_n_4 ,\m_axis_tdata_reg[47]_i_1_n_5 ,\m_axis_tdata_reg[47]_i_1_n_6 ,\m_axis_tdata_reg[47]_i_1_n_7 }),
        .S({\m_axis_tdata[47]_i_2_n_0 ,\m_axis_tdata[47]_i_3_n_0 ,\m_axis_tdata[47]_i_4_n_0 ,\m_axis_tdata[47]_i_5_n_0 }));
  FDRE \m_axis_tdata_reg[48] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[7] [0]),
        .Q(m_axis_tdata[48]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[49] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[7] [1]),
        .Q(m_axis_tdata[49]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[4] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[1] [4]),
        .Q(m_axis_tdata[4]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[50] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[7] [2]),
        .Q(m_axis_tdata[50]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[51] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[7] [3]),
        .Q(m_axis_tdata[51]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[52] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[7] [4]),
        .Q(m_axis_tdata[52]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[53] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[7] [5]),
        .Q(m_axis_tdata[53]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[54] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[7] [6]),
        .Q(m_axis_tdata[54]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[55] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[7] [7]),
        .Q(m_axis_tdata[55]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[56] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[6] [0]),
        .Q(m_axis_tdata[56]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[57] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[6] [1]),
        .Q(m_axis_tdata[57]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[58] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[6] [2]),
        .Q(m_axis_tdata[58]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[59] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[6] [3]),
        .Q(m_axis_tdata[59]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[5] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[1] [5]),
        .Q(m_axis_tdata[5]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[60] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[6] [4]),
        .Q(m_axis_tdata[60]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[61] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[6] [5]),
        .Q(m_axis_tdata[61]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[62] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[6] [6]),
        .Q(m_axis_tdata[62]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[63] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[6] [7]),
        .Q(m_axis_tdata[63]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[64] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[9] [0]),
        .Q(m_axis_tdata[64]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[65] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[9] [1]),
        .Q(m_axis_tdata[65]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[66] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[9] [2]),
        .Q(m_axis_tdata[66]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[67] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[9] [3]),
        .Q(m_axis_tdata[67]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[68] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[9] [4]),
        .Q(m_axis_tdata[68]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[69] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[9] [5]),
        .Q(m_axis_tdata[69]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[6] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[1] [6]),
        .Q(m_axis_tdata[6]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[70] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[9] [6]),
        .Q(m_axis_tdata[70]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[71] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[9] [7]),
        .Q(m_axis_tdata[71]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[72] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[8] [0]),
        .Q(m_axis_tdata[72]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[73] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[8] [1]),
        .Q(m_axis_tdata[73]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[74] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[8] [2]),
        .Q(m_axis_tdata[74]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[75] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[8] [3]),
        .Q(m_axis_tdata[75]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[76] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[8] [4]),
        .Q(m_axis_tdata[76]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[77] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[8] [5]),
        .Q(m_axis_tdata[77]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[78] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[8] [6]),
        .Q(m_axis_tdata[78]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[79] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[8] [7]),
        .Q(m_axis_tdata[79]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[7] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[1] [7]),
        .Q(m_axis_tdata[7]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[80] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[83]_i_1_n_7 ),
        .Q(m_axis_tdata[80]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[81] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[83]_i_1_n_6 ),
        .Q(m_axis_tdata[81]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[82] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[83]_i_1_n_5 ),
        .Q(m_axis_tdata[82]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[83] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[83]_i_1_n_4 ),
        .Q(m_axis_tdata[83]),
        .R(1'b0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \m_axis_tdata_reg[83]_i_1 
       (.CI(1'b0),
        .CO({\m_axis_tdata_reg[83]_i_1_n_0 ,\m_axis_tdata_reg[83]_i_1_n_1 ,\m_axis_tdata_reg[83]_i_1_n_2 ,\m_axis_tdata_reg[83]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b1}),
        .O({\m_axis_tdata_reg[83]_i_1_n_4 ,\m_axis_tdata_reg[83]_i_1_n_5 ,\m_axis_tdata_reg[83]_i_1_n_6 ,\m_axis_tdata_reg[83]_i_1_n_7 }),
        .S({\m_axis_tdata[83]_i_2_n_0 ,\m_axis_tdata[83]_i_3_n_0 ,\m_axis_tdata[83]_i_4_n_0 ,\raw_bytes_reg[11] [0]}));
  FDRE \m_axis_tdata_reg[84] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[87]_i_1_n_7 ),
        .Q(m_axis_tdata[84]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[85] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[87]_i_1_n_6 ),
        .Q(m_axis_tdata[85]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[86] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[87]_i_1_n_5 ),
        .Q(m_axis_tdata[86]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[87] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[87]_i_1_n_4 ),
        .Q(m_axis_tdata[87]),
        .R(1'b0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \m_axis_tdata_reg[87]_i_1 
       (.CI(\m_axis_tdata_reg[83]_i_1_n_0 ),
        .CO({\m_axis_tdata_reg[87]_i_1_n_0 ,\m_axis_tdata_reg[87]_i_1_n_1 ,\m_axis_tdata_reg[87]_i_1_n_2 ,\m_axis_tdata_reg[87]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({\m_axis_tdata_reg[87]_i_1_n_4 ,\m_axis_tdata_reg[87]_i_1_n_5 ,\m_axis_tdata_reg[87]_i_1_n_6 ,\m_axis_tdata_reg[87]_i_1_n_7 }),
        .S({\m_axis_tdata[87]_i_2_n_0 ,\m_axis_tdata[87]_i_3_n_0 ,\m_axis_tdata[87]_i_4_n_0 ,\m_axis_tdata[87]_i_5_n_0 }));
  FDRE \m_axis_tdata_reg[88] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[91]_i_1_n_7 ),
        .Q(m_axis_tdata[88]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[89] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[91]_i_1_n_6 ),
        .Q(m_axis_tdata[89]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[8] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[0] [0]),
        .Q(m_axis_tdata[8]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[90] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[91]_i_1_n_5 ),
        .Q(m_axis_tdata[90]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[91] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[91]_i_1_n_4 ),
        .Q(m_axis_tdata[91]),
        .R(1'b0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \m_axis_tdata_reg[91]_i_1 
       (.CI(\m_axis_tdata_reg[87]_i_1_n_0 ),
        .CO({\m_axis_tdata_reg[91]_i_1_n_0 ,\m_axis_tdata_reg[91]_i_1_n_1 ,\m_axis_tdata_reg[91]_i_1_n_2 ,\m_axis_tdata_reg[91]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({\m_axis_tdata_reg[91]_i_1_n_4 ,\m_axis_tdata_reg[91]_i_1_n_5 ,\m_axis_tdata_reg[91]_i_1_n_6 ,\m_axis_tdata_reg[91]_i_1_n_7 }),
        .S({\m_axis_tdata[91]_i_2_n_0 ,\m_axis_tdata[91]_i_3_n_0 ,\m_axis_tdata[91]_i_4_n_0 ,\m_axis_tdata[91]_i_5_n_0 }));
  FDRE \m_axis_tdata_reg[92] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[95]_i_1_n_7 ),
        .Q(m_axis_tdata[92]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[93] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[95]_i_1_n_6 ),
        .Q(m_axis_tdata[93]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[94] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[95]_i_1_n_5 ),
        .Q(m_axis_tdata[94]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[95] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\m_axis_tdata_reg[95]_i_1_n_4 ),
        .Q(m_axis_tdata[95]),
        .R(1'b0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \m_axis_tdata_reg[95]_i_1 
       (.CI(\m_axis_tdata_reg[91]_i_1_n_0 ),
        .CO({\NLW_m_axis_tdata_reg[95]_i_1_CO_UNCONNECTED [3],\m_axis_tdata_reg[95]_i_1_n_1 ,\m_axis_tdata_reg[95]_i_1_n_2 ,\m_axis_tdata_reg[95]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({\m_axis_tdata_reg[95]_i_1_n_4 ,\m_axis_tdata_reg[95]_i_1_n_5 ,\m_axis_tdata_reg[95]_i_1_n_6 ,\m_axis_tdata_reg[95]_i_1_n_7 }),
        .S({\m_axis_tdata[95]_i_2_n_0 ,\m_axis_tdata[95]_i_3_n_0 ,\m_axis_tdata[95]_i_4_n_0 ,\m_axis_tdata[95]_i_5_n_0 }));
  FDRE \m_axis_tdata_reg[96] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(m_axis_tdata0[0]),
        .Q(m_axis_tdata[96]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[97] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(m_axis_tdata0[1]),
        .Q(m_axis_tdata[97]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[98] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(m_axis_tdata0[2]),
        .Q(m_axis_tdata[98]),
        .R(1'b0));
  FDRE \m_axis_tdata_reg[99] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(m_axis_tdata0[3]),
        .Q(m_axis_tdata[99]),
        .R(1'b0));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \m_axis_tdata_reg[99]_i_1 
       (.CI(1'b0),
        .CO({\m_axis_tdata_reg[99]_i_1_n_0 ,\m_axis_tdata_reg[99]_i_1_n_1 ,\m_axis_tdata_reg[99]_i_1_n_2 ,\m_axis_tdata_reg[99]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b1}),
        .O(m_axis_tdata0[3:0]),
        .S({\m_axis_tdata[99]_i_2_n_0 ,\m_axis_tdata[99]_i_3_n_0 ,\m_axis_tdata[99]_i_4_n_0 ,\raw_bytes_reg[13] [0]}));
  FDRE \m_axis_tdata_reg[9] 
       (.C(clk),
        .CE(\m_axis_tdata[111]_i_1_n_0 ),
        .D(\raw_bytes_reg[0] [1]),
        .Q(m_axis_tdata[9]),
        .R(1'b0));
  (* SOFT_HLUTNM = "soft_lutpair8" *) 
  LUT4 #(
    .INIT(16'h0800)) 
    m_axis_tvalid_i_1
       (.I0(state[2]),
        .I1(state[1]),
        .I2(state[0]),
        .I3(reset_n),
        .O(m_axis_tvalid_i_1_n_0));
  FDRE m_axis_tvalid_reg
       (.C(clk),
        .CE(1'b1),
        .D(m_axis_tvalid_i_1_n_0),
        .Q(m_axis_tvalid),
        .R(1'b0));
  LUT5 #(
    .INIT(32'h00040000)) 
    \raw_bytes[0][7]_i_1 
       (.I0(\byte_idx_reg_n_0_[1] ),
        .I1(\byte_idx_reg_n_0_[0] ),
        .I2(\byte_idx_reg_n_0_[3] ),
        .I3(\byte_idx_reg_n_0_[2] ),
        .I4(\raw_bytes[1][7]_i_2_n_0 ),
        .O(\raw_bytes[0][7]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'h40000000)) 
    \raw_bytes[10][7]_i_1 
       (.I0(\byte_idx_reg_n_0_[2] ),
        .I1(\raw_bytes[1][7]_i_2_n_0 ),
        .I2(\byte_idx_reg_n_0_[3] ),
        .I3(\byte_idx_reg_n_0_[0] ),
        .I4(\byte_idx_reg_n_0_[1] ),
        .O(\raw_bytes[10][7]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'h00000080)) 
    \raw_bytes[11][7]_i_1 
       (.I0(\byte_idx_reg_n_0_[2] ),
        .I1(\raw_bytes[1][7]_i_2_n_0 ),
        .I2(\byte_idx_reg_n_0_[3] ),
        .I3(\byte_idx_reg_n_0_[0] ),
        .I4(\byte_idx_reg_n_0_[1] ),
        .O(\raw_bytes[11][7]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'h00008000)) 
    \raw_bytes[12][7]_i_1 
       (.I0(\byte_idx_reg_n_0_[2] ),
        .I1(\raw_bytes[1][7]_i_2_n_0 ),
        .I2(\byte_idx_reg_n_0_[3] ),
        .I3(\byte_idx_reg_n_0_[0] ),
        .I4(\byte_idx_reg_n_0_[1] ),
        .O(\raw_bytes[12][7]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'h00800000)) 
    \raw_bytes[13][7]_i_1 
       (.I0(\byte_idx_reg_n_0_[2] ),
        .I1(\raw_bytes[1][7]_i_2_n_0 ),
        .I2(\byte_idx_reg_n_0_[3] ),
        .I3(\byte_idx_reg_n_0_[0] ),
        .I4(\byte_idx_reg_n_0_[1] ),
        .O(\raw_bytes[13][7]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'h00020000)) 
    \raw_bytes[1][7]_i_1 
       (.I0(\byte_idx_reg_n_0_[1] ),
        .I1(\byte_idx_reg_n_0_[0] ),
        .I2(\byte_idx_reg_n_0_[3] ),
        .I3(\byte_idx_reg_n_0_[2] ),
        .I4(\raw_bytes[1][7]_i_2_n_0 ),
        .O(\raw_bytes[1][7]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair2" *) 
  LUT5 #(
    .INIT(32'h04000000)) 
    \raw_bytes[1][7]_i_2 
       (.I0(i2c_busy),
        .I1(state[2]),
        .I2(state[1]),
        .I3(state[0]),
        .I4(reset_n),
        .O(\raw_bytes[1][7]_i_2_n_0 ));
  LUT5 #(
    .INIT(32'h00080000)) 
    \raw_bytes[2][7]_i_1 
       (.I0(\byte_idx_reg_n_0_[1] ),
        .I1(\byte_idx_reg_n_0_[0] ),
        .I2(\byte_idx_reg_n_0_[3] ),
        .I3(\byte_idx_reg_n_0_[2] ),
        .I4(\raw_bytes[1][7]_i_2_n_0 ),
        .O(\raw_bytes[2][7]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'h00000040)) 
    \raw_bytes[3][7]_i_1 
       (.I0(\byte_idx_reg_n_0_[3] ),
        .I1(\raw_bytes[1][7]_i_2_n_0 ),
        .I2(\byte_idx_reg_n_0_[2] ),
        .I3(\byte_idx_reg_n_0_[1] ),
        .I4(\byte_idx_reg_n_0_[0] ),
        .O(\raw_bytes[3][7]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'h00400000)) 
    \raw_bytes[4][7]_i_1 
       (.I0(\byte_idx_reg_n_0_[3] ),
        .I1(\raw_bytes[1][7]_i_2_n_0 ),
        .I2(\byte_idx_reg_n_0_[2] ),
        .I3(\byte_idx_reg_n_0_[1] ),
        .I4(\byte_idx_reg_n_0_[0] ),
        .O(\raw_bytes[4][7]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'h04000000)) 
    \raw_bytes[5][7]_i_1 
       (.I0(\byte_idx_reg_n_0_[3] ),
        .I1(\byte_idx_reg_n_0_[1] ),
        .I2(\byte_idx_reg_n_0_[0] ),
        .I3(\raw_bytes[1][7]_i_2_n_0 ),
        .I4(\byte_idx_reg_n_0_[2] ),
        .O(\raw_bytes[5][7]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'h40000000)) 
    \raw_bytes[6][7]_i_1 
       (.I0(\byte_idx_reg_n_0_[3] ),
        .I1(\raw_bytes[1][7]_i_2_n_0 ),
        .I2(\byte_idx_reg_n_0_[2] ),
        .I3(\byte_idx_reg_n_0_[1] ),
        .I4(\byte_idx_reg_n_0_[0] ),
        .O(\raw_bytes[6][7]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'h00000040)) 
    \raw_bytes[7][7]_i_1 
       (.I0(\byte_idx_reg_n_0_[2] ),
        .I1(\raw_bytes[1][7]_i_2_n_0 ),
        .I2(\byte_idx_reg_n_0_[3] ),
        .I3(\byte_idx_reg_n_0_[0] ),
        .I4(\byte_idx_reg_n_0_[1] ),
        .O(\raw_bytes[7][7]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'h00004000)) 
    \raw_bytes[8][7]_i_1 
       (.I0(\byte_idx_reg_n_0_[2] ),
        .I1(\raw_bytes[1][7]_i_2_n_0 ),
        .I2(\byte_idx_reg_n_0_[3] ),
        .I3(\byte_idx_reg_n_0_[0] ),
        .I4(\byte_idx_reg_n_0_[1] ),
        .O(\raw_bytes[8][7]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'h00400000)) 
    \raw_bytes[9][7]_i_1 
       (.I0(\byte_idx_reg_n_0_[2] ),
        .I1(\raw_bytes[1][7]_i_2_n_0 ),
        .I2(\byte_idx_reg_n_0_[3] ),
        .I3(\byte_idx_reg_n_0_[0] ),
        .I4(\byte_idx_reg_n_0_[1] ),
        .O(\raw_bytes[9][7]_i_1_n_0 ));
  FDRE \raw_bytes_reg[0][0] 
       (.C(clk),
        .CE(\raw_bytes[0][7]_i_1_n_0 ),
        .D(i2c_data_rd[0]),
        .Q(\raw_bytes_reg[0] [0]),
        .R(1'b0));
  FDRE \raw_bytes_reg[0][1] 
       (.C(clk),
        .CE(\raw_bytes[0][7]_i_1_n_0 ),
        .D(i2c_data_rd[1]),
        .Q(\raw_bytes_reg[0] [1]),
        .R(1'b0));
  FDRE \raw_bytes_reg[0][2] 
       (.C(clk),
        .CE(\raw_bytes[0][7]_i_1_n_0 ),
        .D(i2c_data_rd[2]),
        .Q(\raw_bytes_reg[0] [2]),
        .R(1'b0));
  FDRE \raw_bytes_reg[0][3] 
       (.C(clk),
        .CE(\raw_bytes[0][7]_i_1_n_0 ),
        .D(i2c_data_rd[3]),
        .Q(\raw_bytes_reg[0] [3]),
        .R(1'b0));
  FDRE \raw_bytes_reg[0][4] 
       (.C(clk),
        .CE(\raw_bytes[0][7]_i_1_n_0 ),
        .D(i2c_data_rd[4]),
        .Q(\raw_bytes_reg[0] [4]),
        .R(1'b0));
  FDRE \raw_bytes_reg[0][5] 
       (.C(clk),
        .CE(\raw_bytes[0][7]_i_1_n_0 ),
        .D(i2c_data_rd[5]),
        .Q(\raw_bytes_reg[0] [5]),
        .R(1'b0));
  FDRE \raw_bytes_reg[0][6] 
       (.C(clk),
        .CE(\raw_bytes[0][7]_i_1_n_0 ),
        .D(i2c_data_rd[6]),
        .Q(\raw_bytes_reg[0] [6]),
        .R(1'b0));
  FDRE \raw_bytes_reg[0][7] 
       (.C(clk),
        .CE(\raw_bytes[0][7]_i_1_n_0 ),
        .D(i2c_data_rd[7]),
        .Q(\raw_bytes_reg[0] [7]),
        .R(1'b0));
  FDRE \raw_bytes_reg[10][0] 
       (.C(clk),
        .CE(\raw_bytes[10][7]_i_1_n_0 ),
        .D(i2c_data_rd[0]),
        .Q(\raw_bytes_reg[10] [0]),
        .R(1'b0));
  FDRE \raw_bytes_reg[10][1] 
       (.C(clk),
        .CE(\raw_bytes[10][7]_i_1_n_0 ),
        .D(i2c_data_rd[1]),
        .Q(\raw_bytes_reg[10] [1]),
        .R(1'b0));
  FDRE \raw_bytes_reg[10][2] 
       (.C(clk),
        .CE(\raw_bytes[10][7]_i_1_n_0 ),
        .D(i2c_data_rd[2]),
        .Q(\raw_bytes_reg[10] [2]),
        .R(1'b0));
  FDRE \raw_bytes_reg[10][3] 
       (.C(clk),
        .CE(\raw_bytes[10][7]_i_1_n_0 ),
        .D(i2c_data_rd[3]),
        .Q(\raw_bytes_reg[10] [3]),
        .R(1'b0));
  FDRE \raw_bytes_reg[10][4] 
       (.C(clk),
        .CE(\raw_bytes[10][7]_i_1_n_0 ),
        .D(i2c_data_rd[4]),
        .Q(\raw_bytes_reg[10] [4]),
        .R(1'b0));
  FDRE \raw_bytes_reg[10][5] 
       (.C(clk),
        .CE(\raw_bytes[10][7]_i_1_n_0 ),
        .D(i2c_data_rd[5]),
        .Q(\raw_bytes_reg[10] [5]),
        .R(1'b0));
  FDRE \raw_bytes_reg[10][6] 
       (.C(clk),
        .CE(\raw_bytes[10][7]_i_1_n_0 ),
        .D(i2c_data_rd[6]),
        .Q(\raw_bytes_reg[10] [6]),
        .R(1'b0));
  FDRE \raw_bytes_reg[10][7] 
       (.C(clk),
        .CE(\raw_bytes[10][7]_i_1_n_0 ),
        .D(i2c_data_rd[7]),
        .Q(\raw_bytes_reg[10] [7]),
        .R(1'b0));
  FDRE \raw_bytes_reg[11][0] 
       (.C(clk),
        .CE(\raw_bytes[11][7]_i_1_n_0 ),
        .D(i2c_data_rd[0]),
        .Q(\raw_bytes_reg[11] [0]),
        .R(1'b0));
  FDRE \raw_bytes_reg[11][1] 
       (.C(clk),
        .CE(\raw_bytes[11][7]_i_1_n_0 ),
        .D(i2c_data_rd[1]),
        .Q(\raw_bytes_reg[11] [1]),
        .R(1'b0));
  FDRE \raw_bytes_reg[11][2] 
       (.C(clk),
        .CE(\raw_bytes[11][7]_i_1_n_0 ),
        .D(i2c_data_rd[2]),
        .Q(\raw_bytes_reg[11] [2]),
        .R(1'b0));
  FDRE \raw_bytes_reg[11][3] 
       (.C(clk),
        .CE(\raw_bytes[11][7]_i_1_n_0 ),
        .D(i2c_data_rd[3]),
        .Q(\raw_bytes_reg[11] [3]),
        .R(1'b0));
  FDRE \raw_bytes_reg[11][4] 
       (.C(clk),
        .CE(\raw_bytes[11][7]_i_1_n_0 ),
        .D(i2c_data_rd[4]),
        .Q(\raw_bytes_reg[11] [4]),
        .R(1'b0));
  FDRE \raw_bytes_reg[11][5] 
       (.C(clk),
        .CE(\raw_bytes[11][7]_i_1_n_0 ),
        .D(i2c_data_rd[5]),
        .Q(\raw_bytes_reg[11] [5]),
        .R(1'b0));
  FDRE \raw_bytes_reg[11][6] 
       (.C(clk),
        .CE(\raw_bytes[11][7]_i_1_n_0 ),
        .D(i2c_data_rd[6]),
        .Q(\raw_bytes_reg[11] [6]),
        .R(1'b0));
  FDRE \raw_bytes_reg[11][7] 
       (.C(clk),
        .CE(\raw_bytes[11][7]_i_1_n_0 ),
        .D(i2c_data_rd[7]),
        .Q(\raw_bytes_reg[11] [7]),
        .R(1'b0));
  FDRE \raw_bytes_reg[12][0] 
       (.C(clk),
        .CE(\raw_bytes[12][7]_i_1_n_0 ),
        .D(i2c_data_rd[0]),
        .Q(\raw_bytes_reg[12] [0]),
        .R(1'b0));
  FDRE \raw_bytes_reg[12][1] 
       (.C(clk),
        .CE(\raw_bytes[12][7]_i_1_n_0 ),
        .D(i2c_data_rd[1]),
        .Q(\raw_bytes_reg[12] [1]),
        .R(1'b0));
  FDRE \raw_bytes_reg[12][2] 
       (.C(clk),
        .CE(\raw_bytes[12][7]_i_1_n_0 ),
        .D(i2c_data_rd[2]),
        .Q(\raw_bytes_reg[12] [2]),
        .R(1'b0));
  FDRE \raw_bytes_reg[12][3] 
       (.C(clk),
        .CE(\raw_bytes[12][7]_i_1_n_0 ),
        .D(i2c_data_rd[3]),
        .Q(\raw_bytes_reg[12] [3]),
        .R(1'b0));
  FDRE \raw_bytes_reg[12][4] 
       (.C(clk),
        .CE(\raw_bytes[12][7]_i_1_n_0 ),
        .D(i2c_data_rd[4]),
        .Q(\raw_bytes_reg[12] [4]),
        .R(1'b0));
  FDRE \raw_bytes_reg[12][5] 
       (.C(clk),
        .CE(\raw_bytes[12][7]_i_1_n_0 ),
        .D(i2c_data_rd[5]),
        .Q(\raw_bytes_reg[12] [5]),
        .R(1'b0));
  FDRE \raw_bytes_reg[12][6] 
       (.C(clk),
        .CE(\raw_bytes[12][7]_i_1_n_0 ),
        .D(i2c_data_rd[6]),
        .Q(\raw_bytes_reg[12] [6]),
        .R(1'b0));
  FDRE \raw_bytes_reg[12][7] 
       (.C(clk),
        .CE(\raw_bytes[12][7]_i_1_n_0 ),
        .D(i2c_data_rd[7]),
        .Q(\raw_bytes_reg[12] [7]),
        .R(1'b0));
  FDRE \raw_bytes_reg[13][0] 
       (.C(clk),
        .CE(\raw_bytes[13][7]_i_1_n_0 ),
        .D(i2c_data_rd[0]),
        .Q(\raw_bytes_reg[13] [0]),
        .R(1'b0));
  FDRE \raw_bytes_reg[13][1] 
       (.C(clk),
        .CE(\raw_bytes[13][7]_i_1_n_0 ),
        .D(i2c_data_rd[1]),
        .Q(\raw_bytes_reg[13] [1]),
        .R(1'b0));
  FDRE \raw_bytes_reg[13][2] 
       (.C(clk),
        .CE(\raw_bytes[13][7]_i_1_n_0 ),
        .D(i2c_data_rd[2]),
        .Q(\raw_bytes_reg[13] [2]),
        .R(1'b0));
  FDRE \raw_bytes_reg[13][3] 
       (.C(clk),
        .CE(\raw_bytes[13][7]_i_1_n_0 ),
        .D(i2c_data_rd[3]),
        .Q(\raw_bytes_reg[13] [3]),
        .R(1'b0));
  FDRE \raw_bytes_reg[13][4] 
       (.C(clk),
        .CE(\raw_bytes[13][7]_i_1_n_0 ),
        .D(i2c_data_rd[4]),
        .Q(\raw_bytes_reg[13] [4]),
        .R(1'b0));
  FDRE \raw_bytes_reg[13][5] 
       (.C(clk),
        .CE(\raw_bytes[13][7]_i_1_n_0 ),
        .D(i2c_data_rd[5]),
        .Q(\raw_bytes_reg[13] [5]),
        .R(1'b0));
  FDRE \raw_bytes_reg[13][6] 
       (.C(clk),
        .CE(\raw_bytes[13][7]_i_1_n_0 ),
        .D(i2c_data_rd[6]),
        .Q(\raw_bytes_reg[13] [6]),
        .R(1'b0));
  FDRE \raw_bytes_reg[13][7] 
       (.C(clk),
        .CE(\raw_bytes[13][7]_i_1_n_0 ),
        .D(i2c_data_rd[7]),
        .Q(\raw_bytes_reg[13] [7]),
        .R(1'b0));
  FDRE \raw_bytes_reg[1][0] 
       (.C(clk),
        .CE(\raw_bytes[1][7]_i_1_n_0 ),
        .D(i2c_data_rd[0]),
        .Q(\raw_bytes_reg[1] [0]),
        .R(1'b0));
  FDRE \raw_bytes_reg[1][1] 
       (.C(clk),
        .CE(\raw_bytes[1][7]_i_1_n_0 ),
        .D(i2c_data_rd[1]),
        .Q(\raw_bytes_reg[1] [1]),
        .R(1'b0));
  FDRE \raw_bytes_reg[1][2] 
       (.C(clk),
        .CE(\raw_bytes[1][7]_i_1_n_0 ),
        .D(i2c_data_rd[2]),
        .Q(\raw_bytes_reg[1] [2]),
        .R(1'b0));
  FDRE \raw_bytes_reg[1][3] 
       (.C(clk),
        .CE(\raw_bytes[1][7]_i_1_n_0 ),
        .D(i2c_data_rd[3]),
        .Q(\raw_bytes_reg[1] [3]),
        .R(1'b0));
  FDRE \raw_bytes_reg[1][4] 
       (.C(clk),
        .CE(\raw_bytes[1][7]_i_1_n_0 ),
        .D(i2c_data_rd[4]),
        .Q(\raw_bytes_reg[1] [4]),
        .R(1'b0));
  FDRE \raw_bytes_reg[1][5] 
       (.C(clk),
        .CE(\raw_bytes[1][7]_i_1_n_0 ),
        .D(i2c_data_rd[5]),
        .Q(\raw_bytes_reg[1] [5]),
        .R(1'b0));
  FDRE \raw_bytes_reg[1][6] 
       (.C(clk),
        .CE(\raw_bytes[1][7]_i_1_n_0 ),
        .D(i2c_data_rd[6]),
        .Q(\raw_bytes_reg[1] [6]),
        .R(1'b0));
  FDRE \raw_bytes_reg[1][7] 
       (.C(clk),
        .CE(\raw_bytes[1][7]_i_1_n_0 ),
        .D(i2c_data_rd[7]),
        .Q(\raw_bytes_reg[1] [7]),
        .R(1'b0));
  FDRE \raw_bytes_reg[2][0] 
       (.C(clk),
        .CE(\raw_bytes[2][7]_i_1_n_0 ),
        .D(i2c_data_rd[0]),
        .Q(\raw_bytes_reg[2] [0]),
        .R(1'b0));
  FDRE \raw_bytes_reg[2][1] 
       (.C(clk),
        .CE(\raw_bytes[2][7]_i_1_n_0 ),
        .D(i2c_data_rd[1]),
        .Q(\raw_bytes_reg[2] [1]),
        .R(1'b0));
  FDRE \raw_bytes_reg[2][2] 
       (.C(clk),
        .CE(\raw_bytes[2][7]_i_1_n_0 ),
        .D(i2c_data_rd[2]),
        .Q(\raw_bytes_reg[2] [2]),
        .R(1'b0));
  FDRE \raw_bytes_reg[2][3] 
       (.C(clk),
        .CE(\raw_bytes[2][7]_i_1_n_0 ),
        .D(i2c_data_rd[3]),
        .Q(\raw_bytes_reg[2] [3]),
        .R(1'b0));
  FDRE \raw_bytes_reg[2][4] 
       (.C(clk),
        .CE(\raw_bytes[2][7]_i_1_n_0 ),
        .D(i2c_data_rd[4]),
        .Q(\raw_bytes_reg[2] [4]),
        .R(1'b0));
  FDRE \raw_bytes_reg[2][5] 
       (.C(clk),
        .CE(\raw_bytes[2][7]_i_1_n_0 ),
        .D(i2c_data_rd[5]),
        .Q(\raw_bytes_reg[2] [5]),
        .R(1'b0));
  FDRE \raw_bytes_reg[2][6] 
       (.C(clk),
        .CE(\raw_bytes[2][7]_i_1_n_0 ),
        .D(i2c_data_rd[6]),
        .Q(\raw_bytes_reg[2] [6]),
        .R(1'b0));
  FDRE \raw_bytes_reg[2][7] 
       (.C(clk),
        .CE(\raw_bytes[2][7]_i_1_n_0 ),
        .D(i2c_data_rd[7]),
        .Q(\raw_bytes_reg[2] [7]),
        .R(1'b0));
  FDRE \raw_bytes_reg[3][0] 
       (.C(clk),
        .CE(\raw_bytes[3][7]_i_1_n_0 ),
        .D(i2c_data_rd[0]),
        .Q(\raw_bytes_reg[3] [0]),
        .R(1'b0));
  FDRE \raw_bytes_reg[3][1] 
       (.C(clk),
        .CE(\raw_bytes[3][7]_i_1_n_0 ),
        .D(i2c_data_rd[1]),
        .Q(\raw_bytes_reg[3] [1]),
        .R(1'b0));
  FDRE \raw_bytes_reg[3][2] 
       (.C(clk),
        .CE(\raw_bytes[3][7]_i_1_n_0 ),
        .D(i2c_data_rd[2]),
        .Q(\raw_bytes_reg[3] [2]),
        .R(1'b0));
  FDRE \raw_bytes_reg[3][3] 
       (.C(clk),
        .CE(\raw_bytes[3][7]_i_1_n_0 ),
        .D(i2c_data_rd[3]),
        .Q(\raw_bytes_reg[3] [3]),
        .R(1'b0));
  FDRE \raw_bytes_reg[3][4] 
       (.C(clk),
        .CE(\raw_bytes[3][7]_i_1_n_0 ),
        .D(i2c_data_rd[4]),
        .Q(\raw_bytes_reg[3] [4]),
        .R(1'b0));
  FDRE \raw_bytes_reg[3][5] 
       (.C(clk),
        .CE(\raw_bytes[3][7]_i_1_n_0 ),
        .D(i2c_data_rd[5]),
        .Q(\raw_bytes_reg[3] [5]),
        .R(1'b0));
  FDRE \raw_bytes_reg[3][6] 
       (.C(clk),
        .CE(\raw_bytes[3][7]_i_1_n_0 ),
        .D(i2c_data_rd[6]),
        .Q(\raw_bytes_reg[3] [6]),
        .R(1'b0));
  FDRE \raw_bytes_reg[3][7] 
       (.C(clk),
        .CE(\raw_bytes[3][7]_i_1_n_0 ),
        .D(i2c_data_rd[7]),
        .Q(\raw_bytes_reg[3] [7]),
        .R(1'b0));
  FDRE \raw_bytes_reg[4][0] 
       (.C(clk),
        .CE(\raw_bytes[4][7]_i_1_n_0 ),
        .D(i2c_data_rd[0]),
        .Q(\raw_bytes_reg[4] [0]),
        .R(1'b0));
  FDRE \raw_bytes_reg[4][1] 
       (.C(clk),
        .CE(\raw_bytes[4][7]_i_1_n_0 ),
        .D(i2c_data_rd[1]),
        .Q(\raw_bytes_reg[4] [1]),
        .R(1'b0));
  FDRE \raw_bytes_reg[4][2] 
       (.C(clk),
        .CE(\raw_bytes[4][7]_i_1_n_0 ),
        .D(i2c_data_rd[2]),
        .Q(\raw_bytes_reg[4] [2]),
        .R(1'b0));
  FDRE \raw_bytes_reg[4][3] 
       (.C(clk),
        .CE(\raw_bytes[4][7]_i_1_n_0 ),
        .D(i2c_data_rd[3]),
        .Q(\raw_bytes_reg[4] [3]),
        .R(1'b0));
  FDRE \raw_bytes_reg[4][4] 
       (.C(clk),
        .CE(\raw_bytes[4][7]_i_1_n_0 ),
        .D(i2c_data_rd[4]),
        .Q(\raw_bytes_reg[4] [4]),
        .R(1'b0));
  FDRE \raw_bytes_reg[4][5] 
       (.C(clk),
        .CE(\raw_bytes[4][7]_i_1_n_0 ),
        .D(i2c_data_rd[5]),
        .Q(\raw_bytes_reg[4] [5]),
        .R(1'b0));
  FDRE \raw_bytes_reg[4][6] 
       (.C(clk),
        .CE(\raw_bytes[4][7]_i_1_n_0 ),
        .D(i2c_data_rd[6]),
        .Q(\raw_bytes_reg[4] [6]),
        .R(1'b0));
  FDRE \raw_bytes_reg[4][7] 
       (.C(clk),
        .CE(\raw_bytes[4][7]_i_1_n_0 ),
        .D(i2c_data_rd[7]),
        .Q(\raw_bytes_reg[4] [7]),
        .R(1'b0));
  FDRE \raw_bytes_reg[5][0] 
       (.C(clk),
        .CE(\raw_bytes[5][7]_i_1_n_0 ),
        .D(i2c_data_rd[0]),
        .Q(\raw_bytes_reg[5] [0]),
        .R(1'b0));
  FDRE \raw_bytes_reg[5][1] 
       (.C(clk),
        .CE(\raw_bytes[5][7]_i_1_n_0 ),
        .D(i2c_data_rd[1]),
        .Q(\raw_bytes_reg[5] [1]),
        .R(1'b0));
  FDRE \raw_bytes_reg[5][2] 
       (.C(clk),
        .CE(\raw_bytes[5][7]_i_1_n_0 ),
        .D(i2c_data_rd[2]),
        .Q(\raw_bytes_reg[5] [2]),
        .R(1'b0));
  FDRE \raw_bytes_reg[5][3] 
       (.C(clk),
        .CE(\raw_bytes[5][7]_i_1_n_0 ),
        .D(i2c_data_rd[3]),
        .Q(\raw_bytes_reg[5] [3]),
        .R(1'b0));
  FDRE \raw_bytes_reg[5][4] 
       (.C(clk),
        .CE(\raw_bytes[5][7]_i_1_n_0 ),
        .D(i2c_data_rd[4]),
        .Q(\raw_bytes_reg[5] [4]),
        .R(1'b0));
  FDRE \raw_bytes_reg[5][5] 
       (.C(clk),
        .CE(\raw_bytes[5][7]_i_1_n_0 ),
        .D(i2c_data_rd[5]),
        .Q(\raw_bytes_reg[5] [5]),
        .R(1'b0));
  FDRE \raw_bytes_reg[5][6] 
       (.C(clk),
        .CE(\raw_bytes[5][7]_i_1_n_0 ),
        .D(i2c_data_rd[6]),
        .Q(\raw_bytes_reg[5] [6]),
        .R(1'b0));
  FDRE \raw_bytes_reg[5][7] 
       (.C(clk),
        .CE(\raw_bytes[5][7]_i_1_n_0 ),
        .D(i2c_data_rd[7]),
        .Q(\raw_bytes_reg[5] [7]),
        .R(1'b0));
  FDRE \raw_bytes_reg[6][0] 
       (.C(clk),
        .CE(\raw_bytes[6][7]_i_1_n_0 ),
        .D(i2c_data_rd[0]),
        .Q(\raw_bytes_reg[6] [0]),
        .R(1'b0));
  FDRE \raw_bytes_reg[6][1] 
       (.C(clk),
        .CE(\raw_bytes[6][7]_i_1_n_0 ),
        .D(i2c_data_rd[1]),
        .Q(\raw_bytes_reg[6] [1]),
        .R(1'b0));
  FDRE \raw_bytes_reg[6][2] 
       (.C(clk),
        .CE(\raw_bytes[6][7]_i_1_n_0 ),
        .D(i2c_data_rd[2]),
        .Q(\raw_bytes_reg[6] [2]),
        .R(1'b0));
  FDRE \raw_bytes_reg[6][3] 
       (.C(clk),
        .CE(\raw_bytes[6][7]_i_1_n_0 ),
        .D(i2c_data_rd[3]),
        .Q(\raw_bytes_reg[6] [3]),
        .R(1'b0));
  FDRE \raw_bytes_reg[6][4] 
       (.C(clk),
        .CE(\raw_bytes[6][7]_i_1_n_0 ),
        .D(i2c_data_rd[4]),
        .Q(\raw_bytes_reg[6] [4]),
        .R(1'b0));
  FDRE \raw_bytes_reg[6][5] 
       (.C(clk),
        .CE(\raw_bytes[6][7]_i_1_n_0 ),
        .D(i2c_data_rd[5]),
        .Q(\raw_bytes_reg[6] [5]),
        .R(1'b0));
  FDRE \raw_bytes_reg[6][6] 
       (.C(clk),
        .CE(\raw_bytes[6][7]_i_1_n_0 ),
        .D(i2c_data_rd[6]),
        .Q(\raw_bytes_reg[6] [6]),
        .R(1'b0));
  FDRE \raw_bytes_reg[6][7] 
       (.C(clk),
        .CE(\raw_bytes[6][7]_i_1_n_0 ),
        .D(i2c_data_rd[7]),
        .Q(\raw_bytes_reg[6] [7]),
        .R(1'b0));
  FDRE \raw_bytes_reg[7][0] 
       (.C(clk),
        .CE(\raw_bytes[7][7]_i_1_n_0 ),
        .D(i2c_data_rd[0]),
        .Q(\raw_bytes_reg[7] [0]),
        .R(1'b0));
  FDRE \raw_bytes_reg[7][1] 
       (.C(clk),
        .CE(\raw_bytes[7][7]_i_1_n_0 ),
        .D(i2c_data_rd[1]),
        .Q(\raw_bytes_reg[7] [1]),
        .R(1'b0));
  FDRE \raw_bytes_reg[7][2] 
       (.C(clk),
        .CE(\raw_bytes[7][7]_i_1_n_0 ),
        .D(i2c_data_rd[2]),
        .Q(\raw_bytes_reg[7] [2]),
        .R(1'b0));
  FDRE \raw_bytes_reg[7][3] 
       (.C(clk),
        .CE(\raw_bytes[7][7]_i_1_n_0 ),
        .D(i2c_data_rd[3]),
        .Q(\raw_bytes_reg[7] [3]),
        .R(1'b0));
  FDRE \raw_bytes_reg[7][4] 
       (.C(clk),
        .CE(\raw_bytes[7][7]_i_1_n_0 ),
        .D(i2c_data_rd[4]),
        .Q(\raw_bytes_reg[7] [4]),
        .R(1'b0));
  FDRE \raw_bytes_reg[7][5] 
       (.C(clk),
        .CE(\raw_bytes[7][7]_i_1_n_0 ),
        .D(i2c_data_rd[5]),
        .Q(\raw_bytes_reg[7] [5]),
        .R(1'b0));
  FDRE \raw_bytes_reg[7][6] 
       (.C(clk),
        .CE(\raw_bytes[7][7]_i_1_n_0 ),
        .D(i2c_data_rd[6]),
        .Q(\raw_bytes_reg[7] [6]),
        .R(1'b0));
  FDRE \raw_bytes_reg[7][7] 
       (.C(clk),
        .CE(\raw_bytes[7][7]_i_1_n_0 ),
        .D(i2c_data_rd[7]),
        .Q(\raw_bytes_reg[7] [7]),
        .R(1'b0));
  FDRE \raw_bytes_reg[8][0] 
       (.C(clk),
        .CE(\raw_bytes[8][7]_i_1_n_0 ),
        .D(i2c_data_rd[0]),
        .Q(\raw_bytes_reg[8] [0]),
        .R(1'b0));
  FDRE \raw_bytes_reg[8][1] 
       (.C(clk),
        .CE(\raw_bytes[8][7]_i_1_n_0 ),
        .D(i2c_data_rd[1]),
        .Q(\raw_bytes_reg[8] [1]),
        .R(1'b0));
  FDRE \raw_bytes_reg[8][2] 
       (.C(clk),
        .CE(\raw_bytes[8][7]_i_1_n_0 ),
        .D(i2c_data_rd[2]),
        .Q(\raw_bytes_reg[8] [2]),
        .R(1'b0));
  FDRE \raw_bytes_reg[8][3] 
       (.C(clk),
        .CE(\raw_bytes[8][7]_i_1_n_0 ),
        .D(i2c_data_rd[3]),
        .Q(\raw_bytes_reg[8] [3]),
        .R(1'b0));
  FDRE \raw_bytes_reg[8][4] 
       (.C(clk),
        .CE(\raw_bytes[8][7]_i_1_n_0 ),
        .D(i2c_data_rd[4]),
        .Q(\raw_bytes_reg[8] [4]),
        .R(1'b0));
  FDRE \raw_bytes_reg[8][5] 
       (.C(clk),
        .CE(\raw_bytes[8][7]_i_1_n_0 ),
        .D(i2c_data_rd[5]),
        .Q(\raw_bytes_reg[8] [5]),
        .R(1'b0));
  FDRE \raw_bytes_reg[8][6] 
       (.C(clk),
        .CE(\raw_bytes[8][7]_i_1_n_0 ),
        .D(i2c_data_rd[6]),
        .Q(\raw_bytes_reg[8] [6]),
        .R(1'b0));
  FDRE \raw_bytes_reg[8][7] 
       (.C(clk),
        .CE(\raw_bytes[8][7]_i_1_n_0 ),
        .D(i2c_data_rd[7]),
        .Q(\raw_bytes_reg[8] [7]),
        .R(1'b0));
  FDRE \raw_bytes_reg[9][0] 
       (.C(clk),
        .CE(\raw_bytes[9][7]_i_1_n_0 ),
        .D(i2c_data_rd[0]),
        .Q(\raw_bytes_reg[9] [0]),
        .R(1'b0));
  FDRE \raw_bytes_reg[9][1] 
       (.C(clk),
        .CE(\raw_bytes[9][7]_i_1_n_0 ),
        .D(i2c_data_rd[1]),
        .Q(\raw_bytes_reg[9] [1]),
        .R(1'b0));
  FDRE \raw_bytes_reg[9][2] 
       (.C(clk),
        .CE(\raw_bytes[9][7]_i_1_n_0 ),
        .D(i2c_data_rd[2]),
        .Q(\raw_bytes_reg[9] [2]),
        .R(1'b0));
  FDRE \raw_bytes_reg[9][3] 
       (.C(clk),
        .CE(\raw_bytes[9][7]_i_1_n_0 ),
        .D(i2c_data_rd[3]),
        .Q(\raw_bytes_reg[9] [3]),
        .R(1'b0));
  FDRE \raw_bytes_reg[9][4] 
       (.C(clk),
        .CE(\raw_bytes[9][7]_i_1_n_0 ),
        .D(i2c_data_rd[4]),
        .Q(\raw_bytes_reg[9] [4]),
        .R(1'b0));
  FDRE \raw_bytes_reg[9][5] 
       (.C(clk),
        .CE(\raw_bytes[9][7]_i_1_n_0 ),
        .D(i2c_data_rd[5]),
        .Q(\raw_bytes_reg[9] [5]),
        .R(1'b0));
  FDRE \raw_bytes_reg[9][6] 
       (.C(clk),
        .CE(\raw_bytes[9][7]_i_1_n_0 ),
        .D(i2c_data_rd[6]),
        .Q(\raw_bytes_reg[9] [6]),
        .R(1'b0));
  FDRE \raw_bytes_reg[9][7] 
       (.C(clk),
        .CE(\raw_bytes[9][7]_i_1_n_0 ),
        .D(i2c_data_rd[7]),
        .Q(\raw_bytes_reg[9] [7]),
        .R(1'b0));
endmodule
`ifndef GLBL
`define GLBL
`timescale  1 ps / 1 ps

module glbl ();

    parameter ROC_WIDTH = 100000;
    parameter TOC_WIDTH = 0;
    parameter GRES_WIDTH = 10000;
    parameter GRES_START = 10000;

//--------   STARTUP Globals --------------
    wire GSR;
    wire GTS;
    wire GWE;
    wire PRLD;
    wire GRESTORE;
    tri1 p_up_tmp;
    tri (weak1, strong0) PLL_LOCKG = p_up_tmp;

    wire PROGB_GLBL;
    wire CCLKO_GLBL;
    wire FCSBO_GLBL;
    wire [3:0] DO_GLBL;
    wire [3:0] DI_GLBL;
   
    reg GSR_int;
    reg GTS_int;
    reg PRLD_int;
    reg GRESTORE_int;

//--------   JTAG Globals --------------
    wire JTAG_TDO_GLBL;
    wire JTAG_TCK_GLBL;
    wire JTAG_TDI_GLBL;
    wire JTAG_TMS_GLBL;
    wire JTAG_TRST_GLBL;

    reg JTAG_CAPTURE_GLBL;
    reg JTAG_RESET_GLBL;
    reg JTAG_SHIFT_GLBL;
    reg JTAG_UPDATE_GLBL;
    reg JTAG_RUNTEST_GLBL;

    reg JTAG_SEL1_GLBL = 0;
    reg JTAG_SEL2_GLBL = 0 ;
    reg JTAG_SEL3_GLBL = 0;
    reg JTAG_SEL4_GLBL = 0;

    reg JTAG_USER_TDO1_GLBL = 1'bz;
    reg JTAG_USER_TDO2_GLBL = 1'bz;
    reg JTAG_USER_TDO3_GLBL = 1'bz;
    reg JTAG_USER_TDO4_GLBL = 1'bz;

    assign (strong1, weak0) GSR = GSR_int;
    assign (strong1, weak0) GTS = GTS_int;
    assign (weak1, weak0) PRLD = PRLD_int;
    assign (strong1, weak0) GRESTORE = GRESTORE_int;

    initial begin
	GSR_int = 1'b1;
	PRLD_int = 1'b1;
	#(ROC_WIDTH)
	GSR_int = 1'b0;
	PRLD_int = 1'b0;
    end

    initial begin
	GTS_int = 1'b1;
	#(TOC_WIDTH)
	GTS_int = 1'b0;
    end

    initial begin 
	GRESTORE_int = 1'b0;
	#(GRES_START);
	GRESTORE_int = 1'b1;
	#(GRES_WIDTH);
	GRESTORE_int = 1'b0;
    end

endmodule
`endif
