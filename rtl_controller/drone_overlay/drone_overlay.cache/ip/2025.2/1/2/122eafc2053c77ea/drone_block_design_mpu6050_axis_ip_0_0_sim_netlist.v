// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2025 Advanced Micro Devices, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2025.2 (win64) Build 6299465 Fri Nov 14 19:35:11 GMT 2025
// Date        : Sun Jun 21 21:51:02 2026
// Host        : Michel-PC running 64-bit major release  (build 9200)
// Command     : write_verilog -force -mode funcsim -rename_top decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix -prefix
//               decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_ drone_block_design_mpu6050_axis_ip_0_0_sim_netlist.v
// Design      : drone_block_design_mpu6050_axis_ip_0_0
// Purpose     : This verilog netlist is a functional simulation representation of the design and should not be modified
//               or synthesized. This netlist cannot be used for SDF annotated simulation.
// Device      : xc7z020clg400-1
// --------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

(* CHECK_LICENSE_TYPE = "drone_block_design_mpu6050_axis_ip_0_0,mpu6050_axis,{}" *) (* downgradeipidentifiedwarnings = "yes" *) (* x_core_info = "mpu6050_axis,Vivado 2025.2" *) 
(* NotValidForBitStream *)
module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix
   (clk,
    reset_n,
    mpu_int,
    scl_i,
    scl_t,
    sda_i,
    sda_t,
    m_axis_tdata,
    m_axis_tvalid,
    m_axis_tready,
    m_axis_tlast,
    m_axis_tkeep,
    debug_mpu_state,
    debug_byte_valid,
    debug_byte_idx,
    debug_raw_byte,
    debug_mpu_int,
    debug_i2c_tick,
    debug_i2c_state,
    debug_i2c_phase,
    debug_cmd_valid_in,
    debug_cmd_ready_out,
    debug_i2c_busy,
    debug_i2c_done,
    debug_i2c_error,
    debug_scl_out,
    debug_sda_out,
    debug_scl_in,
    debug_sda_in,
    debug_bit_idx,
    debug_q_cnt);
  (* x_interface_info = "xilinx.com:signal:clock:1.0 clk CLK" *) (* x_interface_mode = "slave clk" *) (* x_interface_parameter = "XIL_INTERFACENAME clk, ASSOCIATED_BUSIF m_axis, FREQ_HZ 100000000, FREQ_TOLERANCE_HZ 0, PHASE 0.0, CLK_DOMAIN drone_block_design_processing_system7_0_0_FCLK_CLK0, INSERT_VIP 0" *) input clk;
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
  output debug_byte_valid;
  output [3:0]debug_byte_idx;
  output [7:0]debug_raw_byte;
  output debug_mpu_int;
  output debug_i2c_tick;
  output [3:0]debug_i2c_state;
  output [2:0]debug_i2c_phase;
  output debug_cmd_valid_in;
  output debug_cmd_ready_out;
  output debug_i2c_busy;
  output debug_i2c_done;
  output debug_i2c_error;
  output debug_scl_out;
  output debug_sda_out;
  output debug_scl_in;
  output debug_sda_in;
  output [2:0]debug_bit_idx;
  output [1:0]debug_q_cnt;

  wire \<const0> ;
  wire clk;
  wire [2:0]debug_bit_idx;
  wire [3:0]debug_byte_idx;
  wire debug_byte_valid;
  wire debug_cmd_ready_out;
  wire debug_cmd_valid_in;
  wire debug_i2c_busy;
  wire debug_i2c_error;
  wire [2:0]debug_i2c_phase;
  wire [3:0]debug_i2c_state;
  wire debug_i2c_tick;
  wire [3:0]debug_mpu_state;
  wire [1:0]debug_q_cnt;
  wire [7:0]debug_raw_byte;
  wire [111:0]\^m_axis_tdata ;
  wire [15:15]\^m_axis_tkeep ;
  wire m_axis_tready;
  wire m_axis_tvalid;
  wire mpu_int;
  wire reset_n;
  wire scl_i;
  wire scl_t;
  wire sda_i;
  wire sda_t;

  assign debug_i2c_done = \<const0> ;
  assign debug_mpu_int = mpu_int;
  assign debug_scl_in = scl_i;
  assign debug_scl_out = scl_t;
  assign debug_sda_in = sda_i;
  assign debug_sda_out = sda_t;
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
  assign m_axis_tkeep[15] = \^m_axis_tkeep [15];
  assign m_axis_tkeep[14] = \^m_axis_tkeep [15];
  assign m_axis_tkeep[13] = \^m_axis_tkeep [15];
  assign m_axis_tkeep[12] = \^m_axis_tkeep [15];
  assign m_axis_tkeep[11] = \^m_axis_tkeep [15];
  assign m_axis_tkeep[10] = \^m_axis_tkeep [15];
  assign m_axis_tkeep[9] = \^m_axis_tkeep [15];
  assign m_axis_tkeep[8] = \^m_axis_tkeep [15];
  assign m_axis_tkeep[7] = \^m_axis_tkeep [15];
  assign m_axis_tkeep[6] = \^m_axis_tkeep [15];
  assign m_axis_tkeep[5] = \^m_axis_tkeep [15];
  assign m_axis_tkeep[4] = \^m_axis_tkeep [15];
  assign m_axis_tkeep[3] = \^m_axis_tkeep [15];
  assign m_axis_tkeep[2] = \^m_axis_tkeep [15];
  assign m_axis_tkeep[1] = \^m_axis_tkeep [15];
  assign m_axis_tkeep[0] = \^m_axis_tkeep [15];
  assign m_axis_tlast = m_axis_tvalid;
  GND GND
       (.G(\<const0> ));
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_mpu6050_axis U0
       (.\FSM_sequential_state_reg[1] (debug_i2c_state[1]),
        .\FSM_sequential_state_reg[2] (debug_i2c_state[2]),
        .Q(debug_i2c_state[3]),
        .\bit_idx_reg[0] (debug_bit_idx[0]),
        .\bit_idx_reg[1] (debug_bit_idx[1]),
        .\bit_idx_reg[2] (debug_bit_idx[2]),
        .\byte_idx_reg[3]_0 (debug_byte_idx),
        .byte_valid_reg(debug_byte_valid),
        .clk(clk),
        .cmd_ready_reg_reg(debug_cmd_ready_out),
        .debug_i2c_busy(debug_i2c_busy),
        .debug_i2c_error(debug_i2c_error),
        .debug_i2c_state(debug_i2c_state[0]),
        .debug_i2c_tick(debug_i2c_tick),
        .debug_mpu_state(debug_mpu_state),
        .i2c_cmd_valid_r_reg_0(debug_cmd_valid_in),
        .m_axis_tdata(\^m_axis_tdata ),
        .m_axis_tkeep(\^m_axis_tkeep ),
        .m_axis_tready(m_axis_tready),
        .m_axis_tvalid(m_axis_tvalid),
        .mpu_int(mpu_int),
        .\phase_state_reg[0] (debug_i2c_phase[0]),
        .\phase_state_reg[1] (debug_i2c_phase[1]),
        .\phase_state_reg[2] (debug_i2c_phase[2]),
        .\q_cnt_reg[0] (debug_q_cnt[0]),
        .\q_cnt_reg[1] (debug_q_cnt[1]),
        .\rd_data_reg[7] (debug_raw_byte),
        .reset_n(reset_n),
        .scl_t(scl_t),
        .sda_i(sda_i),
        .sda_t(sda_t));
endmodule

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_i2c_master
   (i2c_tick_reg_0,
    SR,
    cmd_ready_reg_reg_0,
    byte_valid_reg_0,
    i2c_done,
    debug_i2c_busy,
    debug_i2c_error,
    scl_t,
    sda_t,
    Q,
    \phase_state_reg[0]_0 ,
    \phase_state_reg[2]_0 ,
    \phase_state_reg[1]_0 ,
    done_reg_0,
    \FSM_onehot_mpu_state_reg[5] ,
    \FSM_onehot_mpu_state_reg[5]_0 ,
    \FSM_onehot_mpu_state_reg[5]_1 ,
    \FSM_sequential_state_reg[2]_0 ,
    \q_cnt_reg[1]_0 ,
    \q_cnt_reg[0]_0 ,
    \FSM_sequential_state_reg[1]_0 ,
    \bit_idx_reg[2]_0 ,
    \bit_idx_reg[1]_0 ,
    \bit_idx_reg[0]_0 ,
    E,
    \FSM_onehot_mpu_state_reg[6] ,
    \byte_idx_reg[0] ,
    \byte_idx_reg[2] ,
    \byte_idx_reg[2]_0 ,
    \byte_idx_reg[2]_1 ,
    \byte_idx_reg[2]_2 ,
    \byte_idx_reg[3] ,
    \byte_idx_reg[3]_0 ,
    \byte_idx_reg[3]_1 ,
    \byte_idx_reg[3]_2 ,
    \byte_idx_reg[2]_3 ,
    \byte_idx_reg[2]_4 ,
    \byte_idx_reg[3]_3 ,
    \byte_idx_reg[2]_5 ,
    \byte_idx_reg[2]_6 ,
    done_reg_1,
    debug_i2c_state,
    done_reg_2,
    cmd_ready_reg_reg_1,
    \rd_data_reg[7]_0 ,
    clk,
    \cmd_type_reg_reg[1]_0 ,
    \burst_len_reg_reg[3]_0 ,
    \m_axis_tkeep_reg[0] ,
    \byte_idx_reg[0]_0 ,
    \FSM_onehot_mpu_state_reg[0] ,
    m_axis_tvalid,
    \init_idx_reg[2] ,
    \init_idx_reg[2]_0 ,
    \init_idx_reg[2]_1 ,
    reset_n,
    sda_i,
    \wr_data_reg_reg[0]_0 ,
    \FSM_onehot_mpu_state_reg[0]_0 ,
    \FSM_onehot_mpu_state_reg[0]_1 ,
    D,
    i2c_cmd_valid_r,
    \FSM_onehot_mpu_state_reg[0]_2 ,
    mpu_int_sync,
    \byte_idx_reg[0]_1 ,
    \init_idx_reg[0] ,
    \raw_bytes_reg[0][0] ,
    \timeout_cnt_reg[0] ,
    O,
    \timeout_cnt_reg[8] ,
    \timeout_cnt_reg[12] ,
    \timeout_cnt_reg[16] ,
    \timeout_cnt_reg[20] ,
    \timeout_cnt_reg[23] ,
    \reg_addr_reg_reg[6]_0 ,
    \wr_data_reg_reg[7]_0 );
  output i2c_tick_reg_0;
  output [0:0]SR;
  output cmd_ready_reg_reg_0;
  output byte_valid_reg_0;
  output i2c_done;
  output debug_i2c_busy;
  output debug_i2c_error;
  output scl_t;
  output sda_t;
  output [0:0]Q;
  output \phase_state_reg[0]_0 ;
  output \phase_state_reg[2]_0 ;
  output \phase_state_reg[1]_0 ;
  output done_reg_0;
  output \FSM_onehot_mpu_state_reg[5] ;
  output \FSM_onehot_mpu_state_reg[5]_0 ;
  output \FSM_onehot_mpu_state_reg[5]_1 ;
  output \FSM_sequential_state_reg[2]_0 ;
  output \q_cnt_reg[1]_0 ;
  output \q_cnt_reg[0]_0 ;
  output \FSM_sequential_state_reg[1]_0 ;
  output \bit_idx_reg[2]_0 ;
  output \bit_idx_reg[1]_0 ;
  output \bit_idx_reg[0]_0 ;
  output [0:0]E;
  output [0:0]\FSM_onehot_mpu_state_reg[6] ;
  output [0:0]\byte_idx_reg[0] ;
  output [0:0]\byte_idx_reg[2] ;
  output [0:0]\byte_idx_reg[2]_0 ;
  output [0:0]\byte_idx_reg[2]_1 ;
  output [0:0]\byte_idx_reg[2]_2 ;
  output [0:0]\byte_idx_reg[3] ;
  output [0:0]\byte_idx_reg[3]_0 ;
  output [0:0]\byte_idx_reg[3]_1 ;
  output [0:0]\byte_idx_reg[3]_2 ;
  output [0:0]\byte_idx_reg[2]_3 ;
  output [0:0]\byte_idx_reg[2]_4 ;
  output [0:0]\byte_idx_reg[3]_3 ;
  output [0:0]\byte_idx_reg[2]_5 ;
  output [0:0]\byte_idx_reg[2]_6 ;
  output [0:0]done_reg_1;
  output [0:0]debug_i2c_state;
  output [23:0]done_reg_2;
  output cmd_ready_reg_reg_1;
  output [7:0]\rd_data_reg[7]_0 ;
  input clk;
  input \cmd_type_reg_reg[1]_0 ;
  input \burst_len_reg_reg[3]_0 ;
  input [8:0]\m_axis_tkeep_reg[0] ;
  input \byte_idx_reg[0]_0 ;
  input \FSM_onehot_mpu_state_reg[0] ;
  input m_axis_tvalid;
  input \init_idx_reg[2] ;
  input \init_idx_reg[2]_0 ;
  input \init_idx_reg[2]_1 ;
  input reset_n;
  input sda_i;
  input \wr_data_reg_reg[0]_0 ;
  input [0:0]\FSM_onehot_mpu_state_reg[0]_0 ;
  input \FSM_onehot_mpu_state_reg[0]_1 ;
  input [0:0]D;
  input i2c_cmd_valid_r;
  input \FSM_onehot_mpu_state_reg[0]_2 ;
  input mpu_int_sync;
  input \byte_idx_reg[0]_1 ;
  input \init_idx_reg[0] ;
  input [3:0]\raw_bytes_reg[0][0] ;
  input [0:0]\timeout_cnt_reg[0] ;
  input [3:0]O;
  input [3:0]\timeout_cnt_reg[8] ;
  input [3:0]\timeout_cnt_reg[12] ;
  input [3:0]\timeout_cnt_reg[16] ;
  input [3:0]\timeout_cnt_reg[20] ;
  input [2:0]\timeout_cnt_reg[23] ;
  input [6:0]\reg_addr_reg_reg[6]_0 ;
  input [6:0]\wr_data_reg_reg[7]_0 ;

  wire [0:0]D;
  wire [0:0]E;
  wire \FSM_onehot_mpu_state[9]_i_4_n_0 ;
  wire \FSM_onehot_mpu_state_reg[0] ;
  wire [0:0]\FSM_onehot_mpu_state_reg[0]_0 ;
  wire \FSM_onehot_mpu_state_reg[0]_1 ;
  wire \FSM_onehot_mpu_state_reg[0]_2 ;
  wire \FSM_onehot_mpu_state_reg[5] ;
  wire \FSM_onehot_mpu_state_reg[5]_0 ;
  wire \FSM_onehot_mpu_state_reg[5]_1 ;
  wire [0:0]\FSM_onehot_mpu_state_reg[6] ;
  wire \FSM_sequential_state[0]_i_2_n_0 ;
  wire \FSM_sequential_state[1]_i_2_n_0 ;
  wire \FSM_sequential_state[1]_i_3_n_0 ;
  wire \FSM_sequential_state[2]_i_2_n_0 ;
  wire \FSM_sequential_state[3]_i_1_n_0 ;
  wire \FSM_sequential_state[3]_i_3_n_0 ;
  wire \FSM_sequential_state[3]_i_4_n_0 ;
  wire \FSM_sequential_state[3]_i_5_n_0 ;
  wire \FSM_sequential_state[3]_i_6_n_0 ;
  wire \FSM_sequential_state[3]_i_7_n_0 ;
  wire \FSM_sequential_state[3]_i_8_n_0 ;
  wire \FSM_sequential_state[3]_i_9_n_0 ;
  wire \FSM_sequential_state_reg[1]_0 ;
  wire \FSM_sequential_state_reg[2]_0 ;
  wire [3:0]O;
  wire [0:0]Q;
  wire [0:0]SR;
  wire \bit_idx[0]_i_1_n_0 ;
  wire \bit_idx[1]_i_1_n_0 ;
  wire \bit_idx[2]_i_1_n_0 ;
  wire \bit_idx[2]_i_2_n_0 ;
  wire \bit_idx[2]_i_3_n_0 ;
  wire \bit_idx[2]_i_4_n_0 ;
  wire \bit_idx[2]_i_5_n_0 ;
  wire \bit_idx_reg[0]_0 ;
  wire \bit_idx_reg[1]_0 ;
  wire \bit_idx_reg[2]_0 ;
  wire [3:3]burst_len_reg;
  wire \burst_len_reg_reg[3]_0 ;
  wire busy_reg_i_1_n_0;
  wire busy_reg_i_2_n_0;
  wire \byte_cnt[0]_i_1_n_0 ;
  wire \byte_cnt[1]_i_1_n_0 ;
  wire \byte_cnt[2]_i_1_n_0 ;
  wire \byte_cnt[3]_i_1_n_0 ;
  wire \byte_cnt[6]_i_2_n_0 ;
  wire \byte_cnt[7]_i_1_n_0 ;
  wire \byte_cnt[7]_i_2_n_0 ;
  wire \byte_cnt[7]_i_4_n_0 ;
  wire \byte_cnt[7]_i_5_n_0 ;
  wire \byte_cnt_reg_n_0_[0] ;
  wire \byte_cnt_reg_n_0_[1] ;
  wire \byte_cnt_reg_n_0_[2] ;
  wire \byte_cnt_reg_n_0_[3] ;
  wire \byte_cnt_reg_n_0_[4] ;
  wire \byte_cnt_reg_n_0_[5] ;
  wire \byte_cnt_reg_n_0_[6] ;
  wire \byte_cnt_reg_n_0_[7] ;
  wire \byte_idx[3]_i_3_n_0 ;
  wire \byte_idx[3]_i_5_n_0 ;
  wire [0:0]\byte_idx_reg[0] ;
  wire \byte_idx_reg[0]_0 ;
  wire \byte_idx_reg[0]_1 ;
  wire [0:0]\byte_idx_reg[2] ;
  wire [0:0]\byte_idx_reg[2]_0 ;
  wire [0:0]\byte_idx_reg[2]_1 ;
  wire [0:0]\byte_idx_reg[2]_2 ;
  wire [0:0]\byte_idx_reg[2]_3 ;
  wire [0:0]\byte_idx_reg[2]_4 ;
  wire [0:0]\byte_idx_reg[2]_5 ;
  wire [0:0]\byte_idx_reg[2]_6 ;
  wire [0:0]\byte_idx_reg[3] ;
  wire [0:0]\byte_idx_reg[3]_0 ;
  wire [0:0]\byte_idx_reg[3]_1 ;
  wire [0:0]\byte_idx_reg[3]_2 ;
  wire [0:0]\byte_idx_reg[3]_3 ;
  wire byte_valid_i_1_n_0;
  wire byte_valid_reg_0;
  wire clk;
  wire cmd_latched_i_1_n_0;
  wire cmd_latched_reg_n_0;
  wire cmd_ready_reg_i_1_n_0;
  wire cmd_ready_reg_i_2_n_0;
  wire cmd_ready_reg_i_3_n_0;
  wire cmd_ready_reg_reg_0;
  wire cmd_ready_reg_reg_1;
  wire [1:1]cmd_type_reg;
  wire \cmd_type_reg_reg[1]_0 ;
  wire debug_i2c_busy;
  wire debug_i2c_error;
  wire [0:0]debug_i2c_state;
  wire [0:0]dev_addr_reg;
  wire done_i_1_n_0;
  wire done_reg_0;
  wire [0:0]done_reg_1;
  wire [23:0]done_reg_2;
  wire error_reg;
  wire error_reg_i_1_n_0;
  wire error_reg_i_3_n_0;
  wire error_reg_i_4_n_0;
  wire i2c_cmd_valid_r;
  wire i2c_done;
  wire i2c_tick_i_1_n_0;
  wire i2c_tick_reg_0;
  wire [7:4]in38;
  wire \init_idx[2]_i_2_n_0 ;
  wire \init_idx_reg[0] ;
  wire \init_idx_reg[2] ;
  wire \init_idx_reg[2]_0 ;
  wire \init_idx_reg[2]_1 ;
  wire [8:0]\m_axis_tkeep_reg[0] ;
  wire m_axis_tvalid;
  wire mpu_int_sync;
  wire p_0_in;
  wire [6:6]p_0_in_0;
  wire \phase_state[0]_i_1_n_0 ;
  wire \phase_state[1]_i_1_n_0 ;
  wire \phase_state[1]_i_2_n_0 ;
  wire \phase_state[2]_i_1_n_0 ;
  wire \phase_state[2]_i_2_n_0 ;
  wire \phase_state[2]_i_3_n_0 ;
  wire \phase_state_reg[0]_0 ;
  wire \phase_state_reg[1]_0 ;
  wire \phase_state_reg[2]_0 ;
  wire \q_cnt[0]_i_1_n_0 ;
  wire \q_cnt[1]_i_1_n_0 ;
  wire \q_cnt_reg[0]_0 ;
  wire \q_cnt_reg[1]_0 ;
  wire \raw_bytes[1][7]_i_2_n_0 ;
  wire \raw_bytes[3][7]_i_2_n_0 ;
  wire [3:0]\raw_bytes_reg[0][0] ;
  wire \rd_data[7]_i_1_n_0 ;
  wire \rd_data[7]_i_2_n_0 ;
  wire [7:0]\rd_data_reg[7]_0 ;
  wire [6:0]reg_addr_reg;
  wire [6:0]\reg_addr_reg_reg[6]_0 ;
  wire reset_n;
  wire scl_out;
  wire scl_out1_out;
  wire scl_out_i_2_n_0;
  wire scl_out_i_5_n_0;
  wire scl_t;
  wire sda_i;
  wire sda_out0_out;
  wire sda_out_i_10_n_0;
  wire sda_out_i_11_n_0;
  wire sda_out_i_1_n_0;
  wire sda_out_i_2_n_0;
  wire sda_out_i_3_n_0;
  wire sda_out_i_4_n_0;
  wire sda_out_i_6_n_0;
  wire sda_out_i_7_n_0;
  wire sda_out_i_8_n_0;
  wire sda_out_i_9_n_0;
  wire sda_t;
  wire \shift_reg[0]_i_1_n_0 ;
  wire \shift_reg[0]_i_2_n_0 ;
  wire \shift_reg[0]_i_3_n_0 ;
  wire \shift_reg[1]_i_1_n_0 ;
  wire \shift_reg[1]_i_2_n_0 ;
  wire \shift_reg[2]_i_1_n_0 ;
  wire \shift_reg[2]_i_2_n_0 ;
  wire \shift_reg[2]_i_3_n_0 ;
  wire \shift_reg[3]_i_1_n_0 ;
  wire \shift_reg[3]_i_2_n_0 ;
  wire \shift_reg[4]_i_1_n_0 ;
  wire \shift_reg[4]_i_2_n_0 ;
  wire \shift_reg[4]_i_3_n_0 ;
  wire \shift_reg[5]_i_1_n_0 ;
  wire \shift_reg[5]_i_2_n_0 ;
  wire \shift_reg[5]_i_3_n_0 ;
  wire \shift_reg[5]_i_4_n_0 ;
  wire \shift_reg[6]_i_1_n_0 ;
  wire \shift_reg[6]_i_2_n_0 ;
  wire \shift_reg[6]_i_3_n_0 ;
  wire \shift_reg[7]_i_1_n_0 ;
  wire \shift_reg[7]_i_2_n_0 ;
  wire \shift_reg[7]_i_3_n_0 ;
  wire \shift_reg[7]_i_4_n_0 ;
  wire \shift_reg[7]_i_5_n_0 ;
  wire \shift_reg_reg_n_0_[0] ;
  wire \shift_reg_reg_n_0_[1] ;
  wire \shift_reg_reg_n_0_[2] ;
  wire \shift_reg_reg_n_0_[3] ;
  wire \shift_reg_reg_n_0_[4] ;
  wire \shift_reg_reg_n_0_[5] ;
  wire \shift_reg_reg_n_0_[6] ;
  wire \shift_reg_reg_n_0_[7] ;
  wire state1_carry_i_1_n_0;
  wire state1_carry_i_2_n_0;
  wire state1_carry_i_3_n_0;
  wire state1_carry_i_4_n_0;
  wire state1_carry_i_5_n_0;
  wire state1_carry_i_6_n_0;
  wire state1_carry_i_7_n_0;
  wire state1_carry_i_8_n_0;
  wire state1_carry_n_0;
  wire state1_carry_n_1;
  wire state1_carry_n_2;
  wire state1_carry_n_3;
  wire [2:0]state__0;
  wire [3:0]state__1;
  wire \tick_cnt[0]_i_1_n_0 ;
  wire \tick_cnt[1]_i_1_n_0 ;
  wire \tick_cnt[2]_i_1_n_0 ;
  wire \tick_cnt[3]_i_1_n_0 ;
  wire \tick_cnt[4]_i_1_n_0 ;
  wire \tick_cnt[5]_i_1_n_0 ;
  wire \tick_cnt[5]_i_2_n_0 ;
  wire \tick_cnt[5]_i_3_n_0 ;
  wire [5:0]tick_cnt_reg;
  wire [0:0]\timeout_cnt_reg[0] ;
  wire [3:0]\timeout_cnt_reg[12] ;
  wire [3:0]\timeout_cnt_reg[16] ;
  wire [3:0]\timeout_cnt_reg[20] ;
  wire [2:0]\timeout_cnt_reg[23] ;
  wire [3:0]\timeout_cnt_reg[8] ;
  wire [7:0]wr_data_reg;
  wire \wr_data_reg_reg[0]_0 ;
  wire [6:0]\wr_data_reg_reg[7]_0 ;
  wire [3:0]NLW_state1_carry_O_UNCONNECTED;

  LUT6 #(
    .INIT(64'hFFFFFFFFFFFFFF02)) 
    \FSM_onehot_mpu_state[9]_i_1 
       (.I0(\m_axis_tkeep_reg[0] [3]),
        .I1(\FSM_onehot_mpu_state_reg[0]_0 ),
        .I2(\FSM_onehot_mpu_state_reg[0]_1 ),
        .I3(D),
        .I4(\FSM_onehot_mpu_state_reg[0] ),
        .I5(\FSM_onehot_mpu_state[9]_i_4_n_0 ),
        .O(E));
  LUT6 #(
    .INIT(64'hFFFFF222F222F222)) 
    \FSM_onehot_mpu_state[9]_i_4 
       (.I0(i2c_cmd_valid_r),
        .I1(p_0_in),
        .I2(\FSM_onehot_mpu_state_reg[0]_2 ),
        .I3(i2c_done),
        .I4(\m_axis_tkeep_reg[0] [6]),
        .I5(mpu_int_sync),
        .O(\FSM_onehot_mpu_state[9]_i_4_n_0 ));
  LUT6 #(
    .INIT(64'h000000000000AA7F)) 
    \FSM_sequential_state[0]_i_1 
       (.I0(state__0[1]),
        .I1(\FSM_sequential_state[0]_i_2_n_0 ),
        .I2(\phase_state_reg[2]_0 ),
        .I3(state__0[2]),
        .I4(state__0[0]),
        .I5(Q),
        .O(state__1[0]));
  (* SOFT_HLUTNM = "soft_lutpair33" *) 
  LUT2 #(
    .INIT(4'hE)) 
    \FSM_sequential_state[0]_i_2 
       (.I0(\phase_state_reg[0]_0 ),
        .I1(\phase_state_reg[1]_0 ),
        .O(\FSM_sequential_state[0]_i_2_n_0 ));
  LUT6 #(
    .INIT(64'h00FE00FE003C000C)) 
    \FSM_sequential_state[1]_i_1 
       (.I0(\FSM_sequential_state[1]_i_2_n_0 ),
        .I1(state__0[0]),
        .I2(state__0[1]),
        .I3(Q),
        .I4(\FSM_sequential_state[1]_i_3_n_0 ),
        .I5(state__0[2]),
        .O(state__1[1]));
  (* SOFT_HLUTNM = "soft_lutpair22" *) 
  LUT3 #(
    .INIT(8'h17)) 
    \FSM_sequential_state[1]_i_2 
       (.I0(\phase_state_reg[0]_0 ),
        .I1(\phase_state_reg[2]_0 ),
        .I2(\phase_state_reg[1]_0 ),
        .O(\FSM_sequential_state[1]_i_2_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair22" *) 
  LUT3 #(
    .INIT(8'h1F)) 
    \FSM_sequential_state[1]_i_3 
       (.I0(\phase_state_reg[2]_0 ),
        .I1(\phase_state_reg[0]_0 ),
        .I2(\phase_state_reg[1]_0 ),
        .O(\FSM_sequential_state[1]_i_3_n_0 ));
  LUT6 #(
    .INIT(64'hFFEA000055EA0000)) 
    \FSM_sequential_state[2]_i_1 
       (.I0(state__0[0]),
        .I1(\phase_state_reg[0]_0 ),
        .I2(\FSM_sequential_state[2]_i_2_n_0 ),
        .I3(state__0[2]),
        .I4(\FSM_sequential_state_reg[1]_0 ),
        .I5(state1_carry_n_0),
        .O(state__1[2]));
  (* SOFT_HLUTNM = "soft_lutpair33" *) 
  LUT2 #(
    .INIT(4'hE)) 
    \FSM_sequential_state[2]_i_2 
       (.I0(\phase_state_reg[1]_0 ),
        .I1(\phase_state_reg[2]_0 ),
        .O(\FSM_sequential_state[2]_i_2_n_0 ));
  LUT6 #(
    .INIT(64'hFFFFEEEEEEFEEEEE)) 
    \FSM_sequential_state[3]_i_1 
       (.I0(\FSM_sequential_state[3]_i_3_n_0 ),
        .I1(\FSM_sequential_state[3]_i_4_n_0 ),
        .I2(\FSM_sequential_state_reg[1]_0 ),
        .I3(\FSM_sequential_state[3]_i_5_n_0 ),
        .I4(\FSM_sequential_state[3]_i_6_n_0 ),
        .I5(\FSM_sequential_state[3]_i_7_n_0 ),
        .O(\FSM_sequential_state[3]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'h00000000000B0300)) 
    \FSM_sequential_state[3]_i_2 
       (.I0(\FSM_sequential_state[3]_i_8_n_0 ),
        .I1(\FSM_sequential_state[3]_i_9_n_0 ),
        .I2(Q),
        .I3(state__0[1]),
        .I4(state__0[2]),
        .I5(state__0[0]),
        .O(state__1[3]));
  LUT6 #(
    .INIT(64'h040C040404080400)) 
    \FSM_sequential_state[3]_i_3 
       (.I0(busy_reg_i_2_n_0),
        .I1(i2c_tick_reg_0),
        .I2(Q),
        .I3(state__0[1]),
        .I4(cmd_ready_reg_i_2_n_0),
        .I5(cmd_latched_reg_n_0),
        .O(\FSM_sequential_state[3]_i_3_n_0 ));
  LUT6 #(
    .INIT(64'h0200000000000000)) 
    \FSM_sequential_state[3]_i_4 
       (.I0(Q),
        .I1(busy_reg_i_2_n_0),
        .I2(state__0[1]),
        .I3(i2c_tick_reg_0),
        .I4(\q_cnt_reg[0]_0 ),
        .I5(\q_cnt_reg[1]_0 ),
        .O(\FSM_sequential_state[3]_i_4_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair5" *) 
  LUT3 #(
    .INIT(8'hFE)) 
    \FSM_sequential_state[3]_i_5 
       (.I0(\bit_idx_reg[1]_0 ),
        .I1(\bit_idx_reg[0]_0 ),
        .I2(\bit_idx_reg[2]_0 ),
        .O(\FSM_sequential_state[3]_i_5_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair4" *) 
  LUT3 #(
    .INIT(8'h80)) 
    \FSM_sequential_state[3]_i_6 
       (.I0(\q_cnt_reg[1]_0 ),
        .I1(\q_cnt_reg[0]_0 ),
        .I2(i2c_tick_reg_0),
        .O(\FSM_sequential_state[3]_i_6_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair16" *) 
  LUT3 #(
    .INIT(8'h40)) 
    \FSM_sequential_state[3]_i_7 
       (.I0(Q),
        .I1(state__0[2]),
        .I2(state__0[0]),
        .O(\FSM_sequential_state[3]_i_7_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair29" *) 
  LUT3 #(
    .INIT(8'hE0)) 
    \FSM_sequential_state[3]_i_8 
       (.I0(\phase_state_reg[2]_0 ),
        .I1(\phase_state_reg[1]_0 ),
        .I2(\phase_state_reg[0]_0 ),
        .O(\FSM_sequential_state[3]_i_8_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair29" *) 
  LUT2 #(
    .INIT(4'h7)) 
    \FSM_sequential_state[3]_i_9 
       (.I0(\phase_state_reg[1]_0 ),
        .I1(\phase_state_reg[2]_0 ),
        .O(\FSM_sequential_state[3]_i_9_n_0 ));
  (* FSM_ENCODED_STATES = "write_bit:0011,get_ack:0100,chk_phase:0010,start:0001,idle:0000,send_ack:0111,read_bit:0110,stop:1000,restart:0101" *) 
  FDRE #(
    .INIT(1'b0)) 
    \FSM_sequential_state_reg[0] 
       (.C(clk),
        .CE(\FSM_sequential_state[3]_i_1_n_0 ),
        .D(state__1[0]),
        .Q(state__0[0]),
        .R(SR));
  (* FSM_ENCODED_STATES = "write_bit:0011,get_ack:0100,chk_phase:0010,start:0001,idle:0000,send_ack:0111,read_bit:0110,stop:1000,restart:0101" *) 
  FDRE #(
    .INIT(1'b0)) 
    \FSM_sequential_state_reg[1] 
       (.C(clk),
        .CE(\FSM_sequential_state[3]_i_1_n_0 ),
        .D(state__1[1]),
        .Q(state__0[1]),
        .R(SR));
  (* FSM_ENCODED_STATES = "write_bit:0011,get_ack:0100,chk_phase:0010,start:0001,idle:0000,send_ack:0111,read_bit:0110,stop:1000,restart:0101" *) 
  FDRE #(
    .INIT(1'b0)) 
    \FSM_sequential_state_reg[2] 
       (.C(clk),
        .CE(\FSM_sequential_state[3]_i_1_n_0 ),
        .D(state__1[2]),
        .Q(state__0[2]),
        .R(SR));
  (* FSM_ENCODED_STATES = "write_bit:0011,get_ack:0100,chk_phase:0010,start:0001,idle:0000,send_ack:0111,read_bit:0110,stop:1000,restart:0101" *) 
  FDRE #(
    .INIT(1'b0)) 
    \FSM_sequential_state_reg[3] 
       (.C(clk),
        .CE(\FSM_sequential_state[3]_i_1_n_0 ),
        .D(state__1[3]),
        .Q(Q),
        .R(SR));
  (* SOFT_HLUTNM = "soft_lutpair16" *) 
  LUT5 #(
    .INIT(32'h09FF0F00)) 
    \bit_idx[0]_i_1 
       (.I0(state__0[0]),
        .I1(state__0[2]),
        .I2(Q),
        .I3(\bit_idx[2]_i_3_n_0 ),
        .I4(\bit_idx_reg[0]_0 ),
        .O(\bit_idx[0]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'h3223FFFF31130000)) 
    \bit_idx[1]_i_1 
       (.I0(\bit_idx_reg[0]_0 ),
        .I1(Q),
        .I2(state__0[0]),
        .I3(state__0[2]),
        .I4(\bit_idx[2]_i_3_n_0 ),
        .I5(\bit_idx_reg[1]_0 ),
        .O(\bit_idx[1]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'h3223FFFF31130000)) 
    \bit_idx[2]_i_1 
       (.I0(\bit_idx[2]_i_2_n_0 ),
        .I1(Q),
        .I2(state__0[0]),
        .I3(state__0[2]),
        .I4(\bit_idx[2]_i_3_n_0 ),
        .I5(\bit_idx_reg[2]_0 ),
        .O(\bit_idx[2]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair14" *) 
  LUT2 #(
    .INIT(4'hE)) 
    \bit_idx[2]_i_2 
       (.I0(\bit_idx_reg[0]_0 ),
        .I1(\bit_idx_reg[1]_0 ),
        .O(\bit_idx[2]_i_2_n_0 ));
  LUT6 #(
    .INIT(64'hFFFFFFFFEAAAAAAA)) 
    \bit_idx[2]_i_3 
       (.I0(\bit_idx[2]_i_4_n_0 ),
        .I1(cmd_ready_reg_i_2_n_0),
        .I2(state__0[0]),
        .I3(state1_carry_n_0),
        .I4(\rd_data[7]_i_2_n_0 ),
        .I5(\bit_idx[2]_i_5_n_0 ),
        .O(\bit_idx[2]_i_3_n_0 ));
  LUT6 #(
    .INIT(64'h2800000000000000)) 
    \bit_idx[2]_i_4 
       (.I0(\FSM_sequential_state[3]_i_5_n_0 ),
        .I1(state__0[2]),
        .I2(state__0[0]),
        .I3(\q_cnt_reg[1]_0 ),
        .I4(\q_cnt_reg[0]_0 ),
        .I5(\shift_reg[7]_i_5_n_0 ),
        .O(\bit_idx[2]_i_4_n_0 ));
  LUT6 #(
    .INIT(64'h0000005700000000)) 
    \bit_idx[2]_i_5 
       (.I0(\phase_state_reg[1]_0 ),
        .I1(\phase_state_reg[0]_0 ),
        .I2(\phase_state_reg[2]_0 ),
        .I3(state__0[0]),
        .I4(state__0[2]),
        .I5(\shift_reg[7]_i_5_n_0 ),
        .O(\bit_idx[2]_i_5_n_0 ));
  FDRE #(
    .INIT(1'b0)) 
    \bit_idx_reg[0] 
       (.C(clk),
        .CE(1'b1),
        .D(\bit_idx[0]_i_1_n_0 ),
        .Q(\bit_idx_reg[0]_0 ),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \bit_idx_reg[1] 
       (.C(clk),
        .CE(1'b1),
        .D(\bit_idx[1]_i_1_n_0 ),
        .Q(\bit_idx_reg[1]_0 ),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \bit_idx_reg[2] 
       (.C(clk),
        .CE(1'b1),
        .D(\bit_idx[2]_i_1_n_0 ),
        .Q(\bit_idx_reg[2]_0 ),
        .R(1'b0));
  FDRE \burst_len_reg_reg[3] 
       (.C(clk),
        .CE(dev_addr_reg),
        .D(\burst_len_reg_reg[3]_0 ),
        .Q(burst_len_reg),
        .R(1'b0));
  LUT6 #(
    .INIT(64'hFFFFFFEF00000020)) 
    busy_reg_i_1
       (.I0(cmd_latched_reg_n_0),
        .I1(Q),
        .I2(i2c_tick_reg_0),
        .I3(state__0[1]),
        .I4(busy_reg_i_2_n_0),
        .I5(debug_i2c_busy),
        .O(busy_reg_i_1_n_0));
  (* SOFT_HLUTNM = "soft_lutpair25" *) 
  LUT2 #(
    .INIT(4'hE)) 
    busy_reg_i_2
       (.I0(state__0[0]),
        .I1(state__0[2]),
        .O(busy_reg_i_2_n_0));
  FDRE #(
    .INIT(1'b0)) 
    busy_reg_reg
       (.C(clk),
        .CE(1'b1),
        .D(busy_reg_i_1_n_0),
        .Q(debug_i2c_busy),
        .R(SR));
  (* SOFT_HLUTNM = "soft_lutpair23" *) 
  LUT3 #(
    .INIT(8'h04)) 
    \byte_cnt[0]_i_1 
       (.I0(Q),
        .I1(state__0[1]),
        .I2(\byte_cnt_reg_n_0_[0] ),
        .O(\byte_cnt[0]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair1" *) 
  LUT4 #(
    .INIT(16'h0440)) 
    \byte_cnt[1]_i_1 
       (.I0(Q),
        .I1(state__0[1]),
        .I2(\byte_cnt_reg_n_0_[0] ),
        .I3(\byte_cnt_reg_n_0_[1] ),
        .O(\byte_cnt[1]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair1" *) 
  LUT5 #(
    .INIT(32'h04444000)) 
    \byte_cnt[2]_i_1 
       (.I0(Q),
        .I1(state__0[1]),
        .I2(\byte_cnt_reg_n_0_[1] ),
        .I3(\byte_cnt_reg_n_0_[0] ),
        .I4(\byte_cnt_reg_n_0_[2] ),
        .O(\byte_cnt[2]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'h0444444440000000)) 
    \byte_cnt[3]_i_1 
       (.I0(Q),
        .I1(state__0[1]),
        .I2(\byte_cnt_reg_n_0_[0] ),
        .I3(\byte_cnt_reg_n_0_[1] ),
        .I4(\byte_cnt_reg_n_0_[2] ),
        .I5(\byte_cnt_reg_n_0_[3] ),
        .O(\byte_cnt[3]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair7" *) 
  LUT5 #(
    .INIT(32'h7FFF8000)) 
    \byte_cnt[4]_i_1 
       (.I0(\byte_cnt_reg_n_0_[0] ),
        .I1(\byte_cnt_reg_n_0_[1] ),
        .I2(\byte_cnt_reg_n_0_[2] ),
        .I3(\byte_cnt_reg_n_0_[3] ),
        .I4(\byte_cnt_reg_n_0_[4] ),
        .O(in38[4]));
  LUT6 #(
    .INIT(64'h7FFFFFFF80000000)) 
    \byte_cnt[5]_i_1 
       (.I0(\byte_cnt_reg_n_0_[4] ),
        .I1(\byte_cnt_reg_n_0_[3] ),
        .I2(\byte_cnt_reg_n_0_[2] ),
        .I3(\byte_cnt_reg_n_0_[1] ),
        .I4(\byte_cnt_reg_n_0_[0] ),
        .I5(\byte_cnt_reg_n_0_[5] ),
        .O(in38[5]));
  LUT6 #(
    .INIT(64'hFFFF7FFF00008000)) 
    \byte_cnt[6]_i_1 
       (.I0(\byte_cnt_reg_n_0_[4] ),
        .I1(\byte_cnt_reg_n_0_[5] ),
        .I2(\byte_cnt_reg_n_0_[3] ),
        .I3(\byte_cnt_reg_n_0_[2] ),
        .I4(\byte_cnt[6]_i_2_n_0 ),
        .I5(\byte_cnt_reg_n_0_[6] ),
        .O(in38[6]));
  LUT2 #(
    .INIT(4'h7)) 
    \byte_cnt[6]_i_2 
       (.I0(\byte_cnt_reg_n_0_[0] ),
        .I1(\byte_cnt_reg_n_0_[1] ),
        .O(\byte_cnt[6]_i_2_n_0 ));
  LUT3 #(
    .INIT(8'h8A)) 
    \byte_cnt[7]_i_1 
       (.I0(\byte_cnt[7]_i_2_n_0 ),
        .I1(Q),
        .I2(state__0[1]),
        .O(\byte_cnt[7]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'h8080FF0080800000)) 
    \byte_cnt[7]_i_2 
       (.I0(\rd_data[7]_i_2_n_0 ),
        .I1(state1_carry_n_0),
        .I2(cmd_ready_reg_i_2_n_0),
        .I3(\FSM_sequential_state_reg[2]_0 ),
        .I4(state__0[0]),
        .I5(\byte_cnt[7]_i_4_n_0 ),
        .O(\byte_cnt[7]_i_2_n_0 ));
  LUT5 #(
    .INIT(32'h7FFF8000)) 
    \byte_cnt[7]_i_3 
       (.I0(\byte_cnt[7]_i_5_n_0 ),
        .I1(\byte_cnt_reg_n_0_[6] ),
        .I2(\byte_cnt_reg_n_0_[5] ),
        .I3(\byte_cnt_reg_n_0_[4] ),
        .I4(\byte_cnt_reg_n_0_[7] ),
        .O(in38[7]));
  LUT6 #(
    .INIT(64'h0000000008000000)) 
    \byte_cnt[7]_i_4 
       (.I0(reset_n),
        .I1(i2c_tick_reg_0),
        .I2(\FSM_sequential_state[0]_i_2_n_0 ),
        .I3(cmd_ready_reg_i_2_n_0),
        .I4(\phase_state_reg[2]_0 ),
        .I5(state__0[1]),
        .O(\byte_cnt[7]_i_4_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair7" *) 
  LUT4 #(
    .INIT(16'h8000)) 
    \byte_cnt[7]_i_5 
       (.I0(\byte_cnt_reg_n_0_[3] ),
        .I1(\byte_cnt_reg_n_0_[2] ),
        .I2(\byte_cnt_reg_n_0_[1] ),
        .I3(\byte_cnt_reg_n_0_[0] ),
        .O(\byte_cnt[7]_i_5_n_0 ));
  FDRE #(
    .INIT(1'b0)) 
    \byte_cnt_reg[0] 
       (.C(clk),
        .CE(\byte_cnt[7]_i_2_n_0 ),
        .D(\byte_cnt[0]_i_1_n_0 ),
        .Q(\byte_cnt_reg_n_0_[0] ),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \byte_cnt_reg[1] 
       (.C(clk),
        .CE(\byte_cnt[7]_i_2_n_0 ),
        .D(\byte_cnt[1]_i_1_n_0 ),
        .Q(\byte_cnt_reg_n_0_[1] ),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \byte_cnt_reg[2] 
       (.C(clk),
        .CE(\byte_cnt[7]_i_2_n_0 ),
        .D(\byte_cnt[2]_i_1_n_0 ),
        .Q(\byte_cnt_reg_n_0_[2] ),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \byte_cnt_reg[3] 
       (.C(clk),
        .CE(\byte_cnt[7]_i_2_n_0 ),
        .D(\byte_cnt[3]_i_1_n_0 ),
        .Q(\byte_cnt_reg_n_0_[3] ),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \byte_cnt_reg[4] 
       (.C(clk),
        .CE(\byte_cnt[7]_i_2_n_0 ),
        .D(in38[4]),
        .Q(\byte_cnt_reg_n_0_[4] ),
        .R(\byte_cnt[7]_i_1_n_0 ));
  FDRE #(
    .INIT(1'b0)) 
    \byte_cnt_reg[5] 
       (.C(clk),
        .CE(\byte_cnt[7]_i_2_n_0 ),
        .D(in38[5]),
        .Q(\byte_cnt_reg_n_0_[5] ),
        .R(\byte_cnt[7]_i_1_n_0 ));
  FDRE #(
    .INIT(1'b0)) 
    \byte_cnt_reg[6] 
       (.C(clk),
        .CE(\byte_cnt[7]_i_2_n_0 ),
        .D(in38[6]),
        .Q(\byte_cnt_reg_n_0_[6] ),
        .R(\byte_cnt[7]_i_1_n_0 ));
  FDRE #(
    .INIT(1'b0)) 
    \byte_cnt_reg[7] 
       (.C(clk),
        .CE(\byte_cnt[7]_i_2_n_0 ),
        .D(in38[7]),
        .Q(\byte_cnt_reg_n_0_[7] ),
        .R(\byte_cnt[7]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'hFFFFFFFFFFFFF080)) 
    \byte_idx[3]_i_1 
       (.I0(\byte_idx[3]_i_3_n_0 ),
        .I1(\byte_idx_reg[0]_1 ),
        .I2(\byte_idx_reg[0]_0 ),
        .I3(\byte_idx[3]_i_5_n_0 ),
        .I4(\m_axis_tkeep_reg[0] [6]),
        .I5(\m_axis_tkeep_reg[0] [0]),
        .O(\FSM_onehot_mpu_state_reg[6] ));
  (* SOFT_HLUTNM = "soft_lutpair28" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \byte_idx[3]_i_3 
       (.I0(\m_axis_tkeep_reg[0] [8]),
        .I1(byte_valid_reg_0),
        .O(\byte_idx[3]_i_3_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair21" *) 
  LUT4 #(
    .INIT(16'h0040)) 
    \byte_idx[3]_i_5 
       (.I0(\raw_bytes_reg[0][0] [1]),
        .I1(\m_axis_tkeep_reg[0] [8]),
        .I2(byte_valid_reg_0),
        .I3(\raw_bytes_reg[0][0] [0]),
        .O(\byte_idx[3]_i_5_n_0 ));
  LUT6 #(
    .INIT(64'h2000000000000000)) 
    byte_valid_i_1
       (.I0(sda_out_i_3_n_0),
        .I1(\q_cnt_reg[0]_0 ),
        .I2(\q_cnt_reg[1]_0 ),
        .I3(state__0[0]),
        .I4(i2c_tick_reg_0),
        .I5(reset_n),
        .O(byte_valid_i_1_n_0));
  FDRE byte_valid_reg
       (.C(clk),
        .CE(1'b1),
        .D(byte_valid_i_1_n_0),
        .Q(byte_valid_reg_0),
        .R(1'b0));
  LUT6 #(
    .INIT(64'hCDDDCDDDCDDDD1DD)) 
    cmd_latched_i_1
       (.I0(p_0_in),
        .I1(cmd_latched_reg_n_0),
        .I2(Q),
        .I3(i2c_tick_reg_0),
        .I4(state__0[1]),
        .I5(busy_reg_i_2_n_0),
        .O(cmd_latched_i_1_n_0));
  (* SOFT_HLUTNM = "soft_lutpair15" *) 
  LUT2 #(
    .INIT(4'h7)) 
    cmd_latched_i_2
       (.I0(\wr_data_reg_reg[0]_0 ),
        .I1(cmd_ready_reg_reg_0),
        .O(p_0_in));
  FDRE #(
    .INIT(1'b0)) 
    cmd_latched_reg
       (.C(clk),
        .CE(1'b1),
        .D(cmd_latched_i_1_n_0),
        .Q(cmd_latched_reg_n_0),
        .R(SR));
  LUT6 #(
    .INIT(64'hF444CC4444444444)) 
    cmd_ready_reg_i_1
       (.I0(\wr_data_reg_reg[0]_0 ),
        .I1(cmd_ready_reg_reg_0),
        .I2(cmd_ready_reg_i_2_n_0),
        .I3(i2c_tick_reg_0),
        .I4(cmd_ready_reg_i_3_n_0),
        .I5(Q),
        .O(cmd_ready_reg_i_1_n_0));
  (* SOFT_HLUTNM = "soft_lutpair31" *) 
  LUT2 #(
    .INIT(4'h8)) 
    cmd_ready_reg_i_2
       (.I0(\q_cnt_reg[0]_0 ),
        .I1(\q_cnt_reg[1]_0 ),
        .O(cmd_ready_reg_i_2_n_0));
  (* SOFT_HLUTNM = "soft_lutpair25" *) 
  LUT3 #(
    .INIT(8'h01)) 
    cmd_ready_reg_i_3
       (.I0(state__0[2]),
        .I1(state__0[0]),
        .I2(state__0[1]),
        .O(cmd_ready_reg_i_3_n_0));
  FDSE #(
    .INIT(1'b1)) 
    cmd_ready_reg_reg
       (.C(clk),
        .CE(1'b1),
        .D(cmd_ready_reg_i_1_n_0),
        .Q(cmd_ready_reg_reg_0),
        .S(SR));
  LUT3 #(
    .INIT(8'h80)) 
    \cmd_type_reg[1]_i_1 
       (.I0(reset_n),
        .I1(cmd_ready_reg_reg_0),
        .I2(\wr_data_reg_reg[0]_0 ),
        .O(dev_addr_reg));
  FDRE \cmd_type_reg_reg[1] 
       (.C(clk),
        .CE(dev_addr_reg),
        .D(\cmd_type_reg_reg[1]_0 ),
        .Q(cmd_type_reg),
        .R(1'b0));
  (* SOFT_HLUTNM = "soft_lutpair32" *) 
  LUT2 #(
    .INIT(4'h2)) 
    \debug_i2c_state[0]_INST_0 
       (.I0(state__0[0]),
        .I1(Q),
        .O(debug_i2c_state));
  (* SOFT_HLUTNM = "soft_lutpair27" *) 
  LUT2 #(
    .INIT(4'h2)) 
    \debug_i2c_state[1]_INST_0 
       (.I0(state__0[1]),
        .I1(Q),
        .O(\FSM_sequential_state_reg[1]_0 ));
  (* SOFT_HLUTNM = "soft_lutpair26" *) 
  LUT2 #(
    .INIT(4'h2)) 
    \debug_i2c_state[2]_INST_0 
       (.I0(state__0[2]),
        .I1(Q),
        .O(\FSM_sequential_state_reg[2]_0 ));
  LUT6 #(
    .INIT(64'h8000000000000000)) 
    done_i_1
       (.I0(\q_cnt_reg[0]_0 ),
        .I1(\q_cnt_reg[1]_0 ),
        .I2(cmd_ready_reg_i_3_n_0),
        .I3(Q),
        .I4(i2c_tick_reg_0),
        .I5(reset_n),
        .O(done_i_1_n_0));
  FDRE done_reg
       (.C(clk),
        .CE(1'b1),
        .D(done_i_1_n_0),
        .Q(i2c_done),
        .R(1'b0));
  LUT6 #(
    .INIT(64'hAAAAFFFFAAAA2000)) 
    error_reg_i_1
       (.I0(error_reg),
        .I1(state__0[1]),
        .I2(i2c_tick_reg_0),
        .I3(error_reg_i_3_n_0),
        .I4(error_reg_i_4_n_0),
        .I5(debug_i2c_error),
        .O(error_reg_i_1_n_0));
  (* SOFT_HLUTNM = "soft_lutpair17" *) 
  LUT3 #(
    .INIT(8'h40)) 
    error_reg_i_2
       (.I0(Q),
        .I1(state__0[2]),
        .I2(sda_i),
        .O(error_reg));
  (* SOFT_HLUTNM = "soft_lutpair18" *) 
  LUT3 #(
    .INIT(8'h04)) 
    error_reg_i_3
       (.I0(\q_cnt_reg[0]_0 ),
        .I1(\q_cnt_reg[1]_0 ),
        .I2(state__0[0]),
        .O(error_reg_i_3_n_0));
  LUT6 #(
    .INIT(64'h0000010000000000)) 
    error_reg_i_4
       (.I0(state__0[2]),
        .I1(state__0[0]),
        .I2(state__0[1]),
        .I3(i2c_tick_reg_0),
        .I4(Q),
        .I5(cmd_latched_reg_n_0),
        .O(error_reg_i_4_n_0));
  FDRE #(
    .INIT(1'b0)) 
    error_reg_reg
       (.C(clk),
        .CE(1'b1),
        .D(error_reg_i_1_n_0),
        .Q(debug_i2c_error),
        .R(SR));
  (* SOFT_HLUTNM = "soft_lutpair15" *) 
  LUT5 #(
    .INIT(32'h5557FFFC)) 
    i2c_cmd_valid_r_i_1
       (.I0(cmd_ready_reg_reg_0),
        .I1(\m_axis_tkeep_reg[0] [7]),
        .I2(\m_axis_tkeep_reg[0] [1]),
        .I3(\m_axis_tkeep_reg[0] [4]),
        .I4(\wr_data_reg_reg[0]_0 ),
        .O(cmd_ready_reg_reg_1));
  LUT6 #(
    .INIT(64'h4000000000000000)) 
    i2c_tick_i_1
       (.I0(tick_cnt_reg[1]),
        .I1(tick_cnt_reg[4]),
        .I2(tick_cnt_reg[5]),
        .I3(tick_cnt_reg[3]),
        .I4(tick_cnt_reg[0]),
        .I5(tick_cnt_reg[2]),
        .O(i2c_tick_i_1_n_0));
  FDRE #(
    .INIT(1'b0)) 
    i2c_tick_reg
       (.C(clk),
        .CE(1'b1),
        .D(i2c_tick_i_1_n_0),
        .Q(i2c_tick_reg_0),
        .R(SR));
  LUT3 #(
    .INIT(8'h38)) 
    \init_idx[0]_i_1 
       (.I0(\m_axis_tkeep_reg[0] [5]),
        .I1(\init_idx[2]_i_2_n_0 ),
        .I2(\init_idx_reg[2] ),
        .O(\FSM_onehot_mpu_state_reg[5]_1 ));
  (* SOFT_HLUTNM = "soft_lutpair2" *) 
  LUT4 #(
    .INIT(16'h2F80)) 
    \init_idx[1]_i_1 
       (.I0(\m_axis_tkeep_reg[0] [5]),
        .I1(\init_idx_reg[2] ),
        .I2(\init_idx[2]_i_2_n_0 ),
        .I3(\init_idx_reg[2]_0 ),
        .O(\FSM_onehot_mpu_state_reg[5]_0 ));
  (* SOFT_HLUTNM = "soft_lutpair2" *) 
  LUT5 #(
    .INIT(32'h2AFF8000)) 
    \init_idx[2]_i_1 
       (.I0(\m_axis_tkeep_reg[0] [5]),
        .I1(\init_idx_reg[2] ),
        .I2(\init_idx_reg[2]_0 ),
        .I3(\init_idx[2]_i_2_n_0 ),
        .I4(\init_idx_reg[2]_1 ),
        .O(\FSM_onehot_mpu_state_reg[5] ));
  LUT5 #(
    .INIT(32'hFFFF8000)) 
    \init_idx[2]_i_2 
       (.I0(\byte_idx_reg[0]_0 ),
        .I1(i2c_done),
        .I2(\m_axis_tkeep_reg[0] [5]),
        .I3(\init_idx_reg[0] ),
        .I4(\m_axis_tkeep_reg[0] [0]),
        .O(\init_idx[2]_i_2_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair0" *) 
  LUT3 #(
    .INIT(8'h80)) 
    \m_axis_tdata[111]_i_1 
       (.I0(i2c_done),
        .I1(\m_axis_tkeep_reg[0] [8]),
        .I2(\byte_idx_reg[0]_0 ),
        .O(done_reg_1));
  (* SOFT_HLUTNM = "soft_lutpair0" *) 
  LUT5 #(
    .INIT(32'h88FF8880)) 
    m_valid_reg_i_1
       (.I0(i2c_done),
        .I1(\m_axis_tkeep_reg[0] [8]),
        .I2(\byte_idx_reg[0]_0 ),
        .I3(\FSM_onehot_mpu_state_reg[0] ),
        .I4(m_axis_tvalid),
        .O(done_reg_0));
  LUT6 #(
    .INIT(64'h0020FFFF00300000)) 
    \phase_state[0]_i_1 
       (.I0(cmd_type_reg),
        .I1(\phase_state_reg[1]_0 ),
        .I2(\FSM_sequential_state_reg[2]_0 ),
        .I3(state__0[0]),
        .I4(\phase_state[2]_i_2_n_0 ),
        .I5(\phase_state_reg[0]_0 ),
        .O(\phase_state[0]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'hFCCCFFFFF8880000)) 
    \phase_state[1]_i_1 
       (.I0(\phase_state_reg[0]_0 ),
        .I1(\phase_state[1]_i_2_n_0 ),
        .I2(state__0[2]),
        .I3(\FSM_sequential_state_reg[1]_0 ),
        .I4(\phase_state[2]_i_2_n_0 ),
        .I5(\phase_state_reg[1]_0 ),
        .O(\phase_state[1]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair26" *) 
  LUT3 #(
    .INIT(8'h04)) 
    \phase_state[1]_i_2 
       (.I0(Q),
        .I1(state__0[2]),
        .I2(state__0[0]),
        .O(\phase_state[1]_i_2_n_0 ));
  LUT6 #(
    .INIT(64'h00F0FFFF00E00000)) 
    \phase_state[2]_i_1 
       (.I0(\phase_state_reg[1]_0 ),
        .I1(state__0[0]),
        .I2(state__0[2]),
        .I3(Q),
        .I4(\phase_state[2]_i_2_n_0 ),
        .I5(\phase_state_reg[2]_0 ),
        .O(\phase_state[2]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'hFFFFFFFF2F2A0000)) 
    \phase_state[2]_i_2 
       (.I0(state__0[0]),
        .I1(state1_carry_n_0),
        .I2(state__0[1]),
        .I3(\FSM_sequential_state[1]_i_2_n_0 ),
        .I4(\phase_state[2]_i_3_n_0 ),
        .I5(error_reg_i_4_n_0),
        .O(\phase_state[2]_i_2_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair4" *) 
  LUT5 #(
    .INIT(32'h00800000)) 
    \phase_state[2]_i_3 
       (.I0(i2c_tick_reg_0),
        .I1(\q_cnt_reg[0]_0 ),
        .I2(\q_cnt_reg[1]_0 ),
        .I3(Q),
        .I4(state__0[2]),
        .O(\phase_state[2]_i_3_n_0 ));
  FDRE #(
    .INIT(1'b0)) 
    \phase_state_reg[0] 
       (.C(clk),
        .CE(1'b1),
        .D(\phase_state[0]_i_1_n_0 ),
        .Q(\phase_state_reg[0]_0 ),
        .R(SR));
  FDRE #(
    .INIT(1'b0)) 
    \phase_state_reg[1] 
       (.C(clk),
        .CE(1'b1),
        .D(\phase_state[1]_i_1_n_0 ),
        .Q(\phase_state_reg[1]_0 ),
        .R(SR));
  FDRE #(
    .INIT(1'b0)) 
    \phase_state_reg[2] 
       (.C(clk),
        .CE(1'b1),
        .D(\phase_state[2]_i_1_n_0 ),
        .Q(\phase_state_reg[2]_0 ),
        .R(SR));
  LUT6 #(
    .INIT(64'hFEFF02FF0100FC00)) 
    \q_cnt[0]_i_1 
       (.I0(state__0[1]),
        .I1(state__0[0]),
        .I2(state__0[2]),
        .I3(i2c_tick_reg_0),
        .I4(Q),
        .I5(\q_cnt_reg[0]_0 ),
        .O(\q_cnt[0]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'hFDFF5CFF0200A000)) 
    \q_cnt[1]_i_1 
       (.I0(\q_cnt_reg[0]_0 ),
        .I1(state__0[1]),
        .I2(busy_reg_i_2_n_0),
        .I3(i2c_tick_reg_0),
        .I4(Q),
        .I5(\q_cnt_reg[1]_0 ),
        .O(\q_cnt[1]_i_1_n_0 ));
  FDRE #(
    .INIT(1'b0)) 
    \q_cnt_reg[0] 
       (.C(clk),
        .CE(1'b1),
        .D(\q_cnt[0]_i_1_n_0 ),
        .Q(\q_cnt_reg[0]_0 ),
        .R(SR));
  FDRE #(
    .INIT(1'b0)) 
    \q_cnt_reg[1] 
       (.C(clk),
        .CE(1'b1),
        .D(\q_cnt[1]_i_1_n_0 ),
        .Q(\q_cnt_reg[1]_0 ),
        .R(SR));
  (* SOFT_HLUTNM = "soft_lutpair20" *) 
  LUT4 #(
    .INIT(16'h0200)) 
    \raw_bytes[0][7]_i_1 
       (.I0(\byte_idx[3]_i_5_n_0 ),
        .I1(\raw_bytes_reg[0][0] [2]),
        .I2(\raw_bytes_reg[0][0] [3]),
        .I3(\byte_idx_reg[0]_0 ),
        .O(\byte_idx_reg[2]_6 ));
  (* SOFT_HLUTNM = "soft_lutpair9" *) 
  LUT5 #(
    .INIT(32'h00200000)) 
    \raw_bytes[10][7]_i_1 
       (.I0(\raw_bytes[3][7]_i_2_n_0 ),
        .I1(\raw_bytes_reg[0][0] [2]),
        .I2(\raw_bytes_reg[0][0] [3]),
        .I3(\raw_bytes_reg[0][0] [0]),
        .I4(\byte_idx_reg[0]_0 ),
        .O(\byte_idx_reg[2]_1 ));
  (* SOFT_HLUTNM = "soft_lutpair9" *) 
  LUT5 #(
    .INIT(32'h20000000)) 
    \raw_bytes[11][7]_i_1 
       (.I0(\raw_bytes[3][7]_i_2_n_0 ),
        .I1(\raw_bytes_reg[0][0] [2]),
        .I2(\raw_bytes_reg[0][0] [0]),
        .I3(\raw_bytes_reg[0][0] [3]),
        .I4(\byte_idx_reg[0]_0 ),
        .O(\byte_idx_reg[2]_0 ));
  (* SOFT_HLUTNM = "soft_lutpair19" *) 
  LUT4 #(
    .INIT(16'h8000)) 
    \raw_bytes[12][7]_i_1 
       (.I0(\raw_bytes_reg[0][0] [2]),
        .I1(\raw_bytes_reg[0][0] [3]),
        .I2(\byte_idx[3]_i_5_n_0 ),
        .I3(\byte_idx_reg[0]_0 ),
        .O(\byte_idx_reg[2] ));
  (* SOFT_HLUTNM = "soft_lutpair8" *) 
  LUT5 #(
    .INIT(32'h80000000)) 
    \raw_bytes[13][7]_i_1 
       (.I0(\raw_bytes[1][7]_i_2_n_0 ),
        .I1(\raw_bytes_reg[0][0] [0]),
        .I2(\raw_bytes_reg[0][0] [3]),
        .I3(\raw_bytes_reg[0][0] [2]),
        .I4(\byte_idx_reg[0]_0 ),
        .O(\byte_idx_reg[0] ));
  (* SOFT_HLUTNM = "soft_lutpair11" *) 
  LUT5 #(
    .INIT(32'h00200000)) 
    \raw_bytes[1][7]_i_1 
       (.I0(\raw_bytes[1][7]_i_2_n_0 ),
        .I1(\raw_bytes_reg[0][0] [2]),
        .I2(\raw_bytes_reg[0][0] [0]),
        .I3(\raw_bytes_reg[0][0] [3]),
        .I4(\byte_idx_reg[0]_0 ),
        .O(\byte_idx_reg[2]_5 ));
  (* SOFT_HLUTNM = "soft_lutpair28" *) 
  LUT3 #(
    .INIT(8'h08)) 
    \raw_bytes[1][7]_i_2 
       (.I0(byte_valid_reg_0),
        .I1(\m_axis_tkeep_reg[0] [8]),
        .I2(\raw_bytes_reg[0][0] [1]),
        .O(\raw_bytes[1][7]_i_2_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair12" *) 
  LUT5 #(
    .INIT(32'h00020000)) 
    \raw_bytes[2][7]_i_1 
       (.I0(\raw_bytes[3][7]_i_2_n_0 ),
        .I1(\raw_bytes_reg[0][0] [3]),
        .I2(\raw_bytes_reg[0][0] [2]),
        .I3(\raw_bytes_reg[0][0] [0]),
        .I4(\byte_idx_reg[0]_0 ),
        .O(\byte_idx_reg[3]_3 ));
  (* SOFT_HLUTNM = "soft_lutpair12" *) 
  LUT5 #(
    .INIT(32'h00200000)) 
    \raw_bytes[3][7]_i_1 
       (.I0(\raw_bytes[3][7]_i_2_n_0 ),
        .I1(\raw_bytes_reg[0][0] [2]),
        .I2(\raw_bytes_reg[0][0] [0]),
        .I3(\raw_bytes_reg[0][0] [3]),
        .I4(\byte_idx_reg[0]_0 ),
        .O(\byte_idx_reg[2]_4 ));
  (* SOFT_HLUTNM = "soft_lutpair21" *) 
  LUT3 #(
    .INIT(8'h80)) 
    \raw_bytes[3][7]_i_2 
       (.I0(byte_valid_reg_0),
        .I1(\m_axis_tkeep_reg[0] [8]),
        .I2(\raw_bytes_reg[0][0] [1]),
        .O(\raw_bytes[3][7]_i_2_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair20" *) 
  LUT4 #(
    .INIT(16'h2000)) 
    \raw_bytes[4][7]_i_1 
       (.I0(\raw_bytes_reg[0][0] [2]),
        .I1(\raw_bytes_reg[0][0] [3]),
        .I2(\byte_idx[3]_i_5_n_0 ),
        .I3(\byte_idx_reg[0]_0 ),
        .O(\byte_idx_reg[2]_3 ));
  (* SOFT_HLUTNM = "soft_lutpair11" *) 
  LUT5 #(
    .INIT(32'h20000000)) 
    \raw_bytes[5][7]_i_1 
       (.I0(\raw_bytes[1][7]_i_2_n_0 ),
        .I1(\raw_bytes_reg[0][0] [3]),
        .I2(\raw_bytes_reg[0][0] [2]),
        .I3(\raw_bytes_reg[0][0] [0]),
        .I4(\byte_idx_reg[0]_0 ),
        .O(\byte_idx_reg[3]_2 ));
  (* SOFT_HLUTNM = "soft_lutpair10" *) 
  LUT5 #(
    .INIT(32'h00200000)) 
    \raw_bytes[6][7]_i_1 
       (.I0(\raw_bytes[3][7]_i_2_n_0 ),
        .I1(\raw_bytes_reg[0][0] [3]),
        .I2(\raw_bytes_reg[0][0] [2]),
        .I3(\raw_bytes_reg[0][0] [0]),
        .I4(\byte_idx_reg[0]_0 ),
        .O(\byte_idx_reg[3]_1 ));
  (* SOFT_HLUTNM = "soft_lutpair10" *) 
  LUT5 #(
    .INIT(32'h20000000)) 
    \raw_bytes[7][7]_i_1 
       (.I0(\raw_bytes[3][7]_i_2_n_0 ),
        .I1(\raw_bytes_reg[0][0] [3]),
        .I2(\raw_bytes_reg[0][0] [2]),
        .I3(\raw_bytes_reg[0][0] [0]),
        .I4(\byte_idx_reg[0]_0 ),
        .O(\byte_idx_reg[3]_0 ));
  (* SOFT_HLUTNM = "soft_lutpair19" *) 
  LUT4 #(
    .INIT(16'h0800)) 
    \raw_bytes[8][7]_i_1 
       (.I0(\byte_idx[3]_i_5_n_0 ),
        .I1(\raw_bytes_reg[0][0] [3]),
        .I2(\raw_bytes_reg[0][0] [2]),
        .I3(\byte_idx_reg[0]_0 ),
        .O(\byte_idx_reg[3] ));
  (* SOFT_HLUTNM = "soft_lutpair8" *) 
  LUT5 #(
    .INIT(32'h20000000)) 
    \raw_bytes[9][7]_i_1 
       (.I0(\raw_bytes[1][7]_i_2_n_0 ),
        .I1(\raw_bytes_reg[0][0] [2]),
        .I2(\raw_bytes_reg[0][0] [0]),
        .I3(\raw_bytes_reg[0][0] [3]),
        .I4(\byte_idx_reg[0]_0 ),
        .O(\byte_idx_reg[2]_2 ));
  LUT4 #(
    .INIT(16'h0800)) 
    \rd_data[7]_i_1 
       (.I0(state__0[0]),
        .I1(\q_cnt_reg[1]_0 ),
        .I2(\q_cnt_reg[0]_0 ),
        .I3(\rd_data[7]_i_2_n_0 ),
        .O(\rd_data[7]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair6" *) 
  LUT5 #(
    .INIT(32'h00800000)) 
    \rd_data[7]_i_2 
       (.I0(reset_n),
        .I1(i2c_tick_reg_0),
        .I2(state__0[1]),
        .I3(Q),
        .I4(state__0[2]),
        .O(\rd_data[7]_i_2_n_0 ));
  FDRE \rd_data_reg[0] 
       (.C(clk),
        .CE(\rd_data[7]_i_1_n_0 ),
        .D(\shift_reg_reg_n_0_[0] ),
        .Q(\rd_data_reg[7]_0 [0]),
        .R(1'b0));
  FDRE \rd_data_reg[1] 
       (.C(clk),
        .CE(\rd_data[7]_i_1_n_0 ),
        .D(\shift_reg_reg_n_0_[1] ),
        .Q(\rd_data_reg[7]_0 [1]),
        .R(1'b0));
  FDRE \rd_data_reg[2] 
       (.C(clk),
        .CE(\rd_data[7]_i_1_n_0 ),
        .D(\shift_reg_reg_n_0_[2] ),
        .Q(\rd_data_reg[7]_0 [2]),
        .R(1'b0));
  FDRE \rd_data_reg[3] 
       (.C(clk),
        .CE(\rd_data[7]_i_1_n_0 ),
        .D(\shift_reg_reg_n_0_[3] ),
        .Q(\rd_data_reg[7]_0 [3]),
        .R(1'b0));
  FDRE \rd_data_reg[4] 
       (.C(clk),
        .CE(\rd_data[7]_i_1_n_0 ),
        .D(\shift_reg_reg_n_0_[4] ),
        .Q(\rd_data_reg[7]_0 [4]),
        .R(1'b0));
  FDRE \rd_data_reg[5] 
       (.C(clk),
        .CE(\rd_data[7]_i_1_n_0 ),
        .D(\shift_reg_reg_n_0_[5] ),
        .Q(\rd_data_reg[7]_0 [5]),
        .R(1'b0));
  FDRE \rd_data_reg[6] 
       (.C(clk),
        .CE(\rd_data[7]_i_1_n_0 ),
        .D(\shift_reg_reg_n_0_[6] ),
        .Q(\rd_data_reg[7]_0 [6]),
        .R(1'b0));
  FDRE \rd_data_reg[7] 
       (.C(clk),
        .CE(\rd_data[7]_i_1_n_0 ),
        .D(\shift_reg_reg_n_0_[7] ),
        .Q(\rd_data_reg[7]_0 [7]),
        .R(1'b0));
  FDRE \reg_addr_reg_reg[0] 
       (.C(clk),
        .CE(dev_addr_reg),
        .D(\reg_addr_reg_reg[6]_0 [0]),
        .Q(reg_addr_reg[0]),
        .R(1'b0));
  FDRE \reg_addr_reg_reg[1] 
       (.C(clk),
        .CE(dev_addr_reg),
        .D(\reg_addr_reg_reg[6]_0 [1]),
        .Q(reg_addr_reg[1]),
        .R(1'b0));
  FDRE \reg_addr_reg_reg[2] 
       (.C(clk),
        .CE(dev_addr_reg),
        .D(\reg_addr_reg_reg[6]_0 [2]),
        .Q(reg_addr_reg[2]),
        .R(1'b0));
  FDRE \reg_addr_reg_reg[3] 
       (.C(clk),
        .CE(dev_addr_reg),
        .D(\reg_addr_reg_reg[6]_0 [3]),
        .Q(reg_addr_reg[3]),
        .R(1'b0));
  FDRE \reg_addr_reg_reg[4] 
       (.C(clk),
        .CE(dev_addr_reg),
        .D(\reg_addr_reg_reg[6]_0 [4]),
        .Q(reg_addr_reg[4]),
        .R(1'b0));
  FDRE \reg_addr_reg_reg[5] 
       (.C(clk),
        .CE(dev_addr_reg),
        .D(\reg_addr_reg_reg[6]_0 [5]),
        .Q(reg_addr_reg[5]),
        .R(1'b0));
  FDRE \reg_addr_reg_reg[6] 
       (.C(clk),
        .CE(dev_addr_reg),
        .D(\reg_addr_reg_reg[6]_0 [6]),
        .Q(reg_addr_reg[6]),
        .R(1'b0));
  LUT1 #(
    .INIT(2'h1)) 
    scl_out_i_1
       (.I0(reset_n),
        .O(SR));
  LUT3 #(
    .INIT(8'hB8)) 
    scl_out_i_2
       (.I0(scl_out),
        .I1(scl_out1_out),
        .I2(scl_t),
        .O(scl_out_i_2_n_0));
  LUT6 #(
    .INIT(64'h14141414140514FD)) 
    scl_out_i_3
       (.I0(Q),
        .I1(\q_cnt_reg[0]_0 ),
        .I2(\q_cnt_reg[1]_0 ),
        .I3(state__0[2]),
        .I4(state__0[0]),
        .I5(state__0[1]),
        .O(scl_out));
  LUT6 #(
    .INIT(64'h00020000A8AAA88A)) 
    scl_out_i_4
       (.I0(i2c_tick_reg_0),
        .I1(state__0[2]),
        .I2(state__0[0]),
        .I3(state__0[1]),
        .I4(scl_out_i_5_n_0),
        .I5(Q),
        .O(scl_out1_out));
  (* SOFT_HLUTNM = "soft_lutpair31" *) 
  LUT2 #(
    .INIT(4'h7)) 
    scl_out_i_5
       (.I0(\q_cnt_reg[1]_0 ),
        .I1(\q_cnt_reg[0]_0 ),
        .O(scl_out_i_5_n_0));
  FDSE #(
    .INIT(1'b1)) 
    scl_out_reg
       (.C(clk),
        .CE(1'b1),
        .D(scl_out_i_2_n_0),
        .Q(scl_t),
        .S(SR));
  LUT6 #(
    .INIT(64'hFFAEFFFFFFAE0000)) 
    sda_out_i_1
       (.I0(sda_out_i_2_n_0),
        .I1(sda_out_i_3_n_0),
        .I2(state1_carry_n_0),
        .I3(sda_out_i_4_n_0),
        .I4(sda_out0_out),
        .I5(sda_t),
        .O(sda_out_i_1_n_0));
  LUT6 #(
    .INIT(64'h1110771F00000000)) 
    sda_out_i_10
       (.I0(\q_cnt_reg[0]_0 ),
        .I1(\q_cnt_reg[1]_0 ),
        .I2(state__0[2]),
        .I3(state__0[0]),
        .I4(state__0[1]),
        .I5(sda_out_i_11_n_0),
        .O(sda_out_i_10_n_0));
  (* SOFT_HLUTNM = "soft_lutpair32" *) 
  LUT2 #(
    .INIT(4'h2)) 
    sda_out_i_11
       (.I0(i2c_tick_reg_0),
        .I1(Q),
        .O(sda_out_i_11_n_0));
  LUT6 #(
    .INIT(64'h00000000050155F5)) 
    sda_out_i_2
       (.I0(Q),
        .I1(\q_cnt_reg[0]_0 ),
        .I2(\q_cnt_reg[1]_0 ),
        .I3(state__0[2]),
        .I4(state__0[0]),
        .I5(state__0[1]),
        .O(sda_out_i_2_n_0));
  (* SOFT_HLUTNM = "soft_lutpair23" *) 
  LUT3 #(
    .INIT(8'h40)) 
    sda_out_i_3
       (.I0(Q),
        .I1(state__0[1]),
        .I2(state__0[2]),
        .O(sda_out_i_3_n_0));
  LUT6 #(
    .INIT(64'hFFFFFFFFFFFFF080)) 
    sda_out_i_4
       (.I0(\bit_idx_reg[0]_0 ),
        .I1(sda_out_i_6_n_0),
        .I2(sda_out_i_7_n_0),
        .I3(sda_out_i_8_n_0),
        .I4(\phase_state[1]_i_2_n_0 ),
        .I5(sda_out_i_9_n_0),
        .O(sda_out_i_4_n_0));
  LUT6 #(
    .INIT(64'hAAAAAABFAAAAAAAA)) 
    sda_out_i_5
       (.I0(sda_out_i_10_n_0),
        .I1(\q_cnt_reg[1]_0 ),
        .I2(\q_cnt_reg[0]_0 ),
        .I3(state__0[1]),
        .I4(busy_reg_i_2_n_0),
        .I5(i2c_tick_reg_0),
        .O(sda_out0_out));
  LUT6 #(
    .INIT(64'hAFA0CFCFAFA0C0C0)) 
    sda_out_i_6
       (.I0(\shift_reg_reg_n_0_[7] ),
        .I1(\shift_reg_reg_n_0_[3] ),
        .I2(\bit_idx_reg[1]_0 ),
        .I3(\shift_reg_reg_n_0_[5] ),
        .I4(\bit_idx_reg[2]_0 ),
        .I5(\shift_reg_reg_n_0_[1] ),
        .O(sda_out_i_6_n_0));
  (* SOFT_HLUTNM = "soft_lutpair27" *) 
  LUT3 #(
    .INIT(8'h04)) 
    sda_out_i_7
       (.I0(Q),
        .I1(state__0[1]),
        .I2(state__0[2]),
        .O(sda_out_i_7_n_0));
  (* SOFT_HLUTNM = "soft_lutpair14" *) 
  LUT5 #(
    .INIT(32'h44400040)) 
    sda_out_i_8
       (.I0(\bit_idx_reg[0]_0 ),
        .I1(\bit_idx_reg[1]_0 ),
        .I2(\shift_reg_reg_n_0_[2] ),
        .I3(\bit_idx_reg[2]_0 ),
        .I4(\shift_reg_reg_n_0_[6] ),
        .O(sda_out_i_8_n_0));
  LUT6 #(
    .INIT(64'h000000000000C808)) 
    sda_out_i_9
       (.I0(\shift_reg_reg_n_0_[0] ),
        .I1(sda_out_i_7_n_0),
        .I2(\bit_idx_reg[2]_0 ),
        .I3(\shift_reg_reg_n_0_[4] ),
        .I4(\bit_idx_reg[1]_0 ),
        .I5(\bit_idx_reg[0]_0 ),
        .O(sda_out_i_9_n_0));
  FDSE #(
    .INIT(1'b1)) 
    sda_out_reg
       (.C(clk),
        .CE(1'b1),
        .D(sda_out_i_1_n_0),
        .Q(sda_t),
        .S(SR));
  LUT5 #(
    .INIT(32'hAAEFAA20)) 
    \shift_reg[0]_i_1 
       (.I0(\shift_reg[0]_i_2_n_0 ),
        .I1(\FSM_sequential_state[3]_i_5_n_0 ),
        .I2(\shift_reg[4]_i_3_n_0 ),
        .I3(\shift_reg[7]_i_4_n_0 ),
        .I4(\shift_reg_reg_n_0_[0] ),
        .O(\shift_reg[0]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'hFFAAAAAAEAEAAAAA)) 
    \shift_reg[0]_i_2 
       (.I0(\shift_reg[0]_i_3_n_0 ),
        .I1(reg_addr_reg[0]),
        .I2(\phase_state_reg[0]_0 ),
        .I3(wr_data_reg[0]),
        .I4(\shift_reg[6]_i_2_n_0 ),
        .I5(\phase_state_reg[1]_0 ),
        .O(\shift_reg[0]_i_2_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair17" *) 
  LUT4 #(
    .INIT(16'h00AC)) 
    \shift_reg[0]_i_3 
       (.I0(sda_i),
        .I1(\phase_state_reg[2]_0 ),
        .I2(state__0[2]),
        .I3(Q),
        .O(\shift_reg[0]_i_3_n_0 ));
  LUT6 #(
    .INIT(64'hAAAAFFFBAAAA0008)) 
    \shift_reg[1]_i_1 
       (.I0(\shift_reg[1]_i_2_n_0 ),
        .I1(\shift_reg[7]_i_3_n_0 ),
        .I2(\bit_idx_reg[1]_0 ),
        .I3(\bit_idx_reg[2]_0 ),
        .I4(\shift_reg[7]_i_4_n_0 ),
        .I5(\shift_reg_reg_n_0_[1] ),
        .O(\shift_reg[1]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'hFFFFF888F888F888)) 
    \shift_reg[1]_i_2 
       (.I0(\FSM_sequential_state_reg[2]_0 ),
        .I1(sda_i),
        .I2(\shift_reg[5]_i_3_n_0 ),
        .I3(reg_addr_reg[1]),
        .I4(wr_data_reg[1]),
        .I5(\shift_reg[5]_i_4_n_0 ),
        .O(\shift_reg[1]_i_2_n_0 ));
  LUT6 #(
    .INIT(64'hAAAAFFBFAAAA0080)) 
    \shift_reg[2]_i_1 
       (.I0(\shift_reg[2]_i_2_n_0 ),
        .I1(\shift_reg[4]_i_3_n_0 ),
        .I2(\shift_reg[2]_i_3_n_0 ),
        .I3(\bit_idx_reg[0]_0 ),
        .I4(\shift_reg[7]_i_4_n_0 ),
        .I5(\shift_reg_reg_n_0_[2] ),
        .O(\shift_reg[2]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'hFFFFF888F888F888)) 
    \shift_reg[2]_i_2 
       (.I0(\FSM_sequential_state_reg[2]_0 ),
        .I1(sda_i),
        .I2(\shift_reg[5]_i_3_n_0 ),
        .I3(reg_addr_reg[2]),
        .I4(wr_data_reg[2]),
        .I5(\shift_reg[5]_i_4_n_0 ),
        .O(\shift_reg[2]_i_2_n_0 ));
  LUT2 #(
    .INIT(4'h2)) 
    \shift_reg[2]_i_3 
       (.I0(\bit_idx_reg[1]_0 ),
        .I1(\bit_idx_reg[2]_0 ),
        .O(\shift_reg[2]_i_3_n_0 ));
  LUT6 #(
    .INIT(64'hAAAAFFBFAAAA0080)) 
    \shift_reg[3]_i_1 
       (.I0(\shift_reg[3]_i_2_n_0 ),
        .I1(\shift_reg[7]_i_3_n_0 ),
        .I2(\bit_idx_reg[1]_0 ),
        .I3(\bit_idx_reg[2]_0 ),
        .I4(\shift_reg[7]_i_4_n_0 ),
        .I5(\shift_reg_reg_n_0_[3] ),
        .O(\shift_reg[3]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'hFFFFF888F888F888)) 
    \shift_reg[3]_i_2 
       (.I0(\FSM_sequential_state_reg[2]_0 ),
        .I1(sda_i),
        .I2(\shift_reg[5]_i_3_n_0 ),
        .I3(reg_addr_reg[3]),
        .I4(wr_data_reg[3]),
        .I5(\shift_reg[5]_i_4_n_0 ),
        .O(\shift_reg[3]_i_2_n_0 ));
  LUT6 #(
    .INIT(64'hAAAAFFBFAAAA0080)) 
    \shift_reg[4]_i_1 
       (.I0(\shift_reg[4]_i_2_n_0 ),
        .I1(\shift_reg[4]_i_3_n_0 ),
        .I2(\bit_idx_reg[2]_0 ),
        .I3(\bit_idx[2]_i_2_n_0 ),
        .I4(\shift_reg[7]_i_4_n_0 ),
        .I5(\shift_reg_reg_n_0_[4] ),
        .O(\shift_reg[4]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'hAAAAAAFAAAAAAAEE)) 
    \shift_reg[4]_i_2 
       (.I0(\shift_reg[6]_i_3_n_0 ),
        .I1(reg_addr_reg[4]),
        .I2(wr_data_reg[4]),
        .I3(Q),
        .I4(state__0[2]),
        .I5(\phase_state_reg[1]_0 ),
        .O(\shift_reg[4]_i_2_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair18" *) 
  LUT4 #(
    .INIT(16'h0400)) 
    \shift_reg[4]_i_3 
       (.I0(state__0[0]),
        .I1(\q_cnt_reg[1]_0 ),
        .I2(\q_cnt_reg[0]_0 ),
        .I3(\rd_data[7]_i_2_n_0 ),
        .O(\shift_reg[4]_i_3_n_0 ));
  LUT6 #(
    .INIT(64'hAAAAFFBFAAAA0080)) 
    \shift_reg[5]_i_1 
       (.I0(\shift_reg[5]_i_2_n_0 ),
        .I1(\shift_reg[7]_i_3_n_0 ),
        .I2(\bit_idx_reg[2]_0 ),
        .I3(\bit_idx_reg[1]_0 ),
        .I4(\shift_reg[7]_i_4_n_0 ),
        .I5(\shift_reg_reg_n_0_[5] ),
        .O(\shift_reg[5]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'hFFFFF888F888F888)) 
    \shift_reg[5]_i_2 
       (.I0(\FSM_sequential_state_reg[2]_0 ),
        .I1(sda_i),
        .I2(\shift_reg[5]_i_3_n_0 ),
        .I3(reg_addr_reg[5]),
        .I4(wr_data_reg[5]),
        .I5(\shift_reg[5]_i_4_n_0 ),
        .O(\shift_reg[5]_i_2_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair13" *) 
  LUT5 #(
    .INIT(32'h00000002)) 
    \shift_reg[5]_i_3 
       (.I0(\phase_state_reg[0]_0 ),
        .I1(state__0[2]),
        .I2(Q),
        .I3(\phase_state_reg[2]_0 ),
        .I4(\phase_state_reg[1]_0 ),
        .O(\shift_reg[5]_i_3_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair13" *) 
  LUT4 #(
    .INIT(16'h0002)) 
    \shift_reg[5]_i_4 
       (.I0(\phase_state_reg[1]_0 ),
        .I1(state__0[2]),
        .I2(Q),
        .I3(\phase_state_reg[2]_0 ),
        .O(\shift_reg[5]_i_4_n_0 ));
  LUT6 #(
    .INIT(64'hFF08FFFFFF080000)) 
    \shift_reg[6]_i_1 
       (.I0(\shift_reg[6]_i_2_n_0 ),
        .I1(reg_addr_reg[6]),
        .I2(\phase_state_reg[1]_0 ),
        .I3(\shift_reg[6]_i_3_n_0 ),
        .I4(p_0_in_0),
        .I5(\shift_reg_reg_n_0_[6] ),
        .O(\shift_reg[6]_i_1_n_0 ));
  LUT2 #(
    .INIT(4'h1)) 
    \shift_reg[6]_i_2 
       (.I0(state__0[2]),
        .I1(Q),
        .O(\shift_reg[6]_i_2_n_0 ));
  LUT6 #(
    .INIT(64'h0F0F0F01000F0001)) 
    \shift_reg[6]_i_3 
       (.I0(\phase_state_reg[0]_0 ),
        .I1(\phase_state_reg[1]_0 ),
        .I2(Q),
        .I3(state__0[2]),
        .I4(\phase_state_reg[2]_0 ),
        .I5(sda_i),
        .O(\shift_reg[6]_i_3_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair5" *) 
  LUT5 #(
    .INIT(32'hFFFF0080)) 
    \shift_reg[6]_i_4 
       (.I0(\shift_reg[4]_i_3_n_0 ),
        .I1(\bit_idx_reg[2]_0 ),
        .I2(\bit_idx_reg[1]_0 ),
        .I3(\bit_idx_reg[0]_0 ),
        .I4(\shift_reg[7]_i_4_n_0 ),
        .O(p_0_in_0));
  LUT6 #(
    .INIT(64'hAAAABFFFAAAA8000)) 
    \shift_reg[7]_i_1 
       (.I0(\shift_reg[7]_i_2_n_0 ),
        .I1(\shift_reg[7]_i_3_n_0 ),
        .I2(\bit_idx_reg[1]_0 ),
        .I3(\bit_idx_reg[2]_0 ),
        .I4(\shift_reg[7]_i_4_n_0 ),
        .I5(\shift_reg_reg_n_0_[7] ),
        .O(\shift_reg[7]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'hFECCFCCCFECCFFCC)) 
    \shift_reg[7]_i_2 
       (.I0(wr_data_reg[7]),
        .I1(error_reg),
        .I2(\phase_state_reg[2]_0 ),
        .I3(\shift_reg[6]_i_2_n_0 ),
        .I4(\phase_state_reg[1]_0 ),
        .I5(\phase_state_reg[0]_0 ),
        .O(\shift_reg[7]_i_2_n_0 ));
  LUT2 #(
    .INIT(4'h8)) 
    \shift_reg[7]_i_3 
       (.I0(\shift_reg[4]_i_3_n_0 ),
        .I1(\bit_idx_reg[0]_0 ),
        .O(\shift_reg[7]_i_3_n_0 ));
  LUT6 #(
    .INIT(64'h0000001000101010)) 
    \shift_reg[7]_i_4 
       (.I0(state__0[0]),
        .I1(state__0[2]),
        .I2(\shift_reg[7]_i_5_n_0 ),
        .I3(\phase_state_reg[1]_0 ),
        .I4(\phase_state_reg[2]_0 ),
        .I5(\phase_state_reg[0]_0 ),
        .O(\shift_reg[7]_i_4_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair6" *) 
  LUT4 #(
    .INIT(16'h4000)) 
    \shift_reg[7]_i_5 
       (.I0(Q),
        .I1(state__0[1]),
        .I2(i2c_tick_reg_0),
        .I3(reset_n),
        .O(\shift_reg[7]_i_5_n_0 ));
  FDRE #(
    .INIT(1'b0)) 
    \shift_reg_reg[0] 
       (.C(clk),
        .CE(1'b1),
        .D(\shift_reg[0]_i_1_n_0 ),
        .Q(\shift_reg_reg_n_0_[0] ),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \shift_reg_reg[1] 
       (.C(clk),
        .CE(1'b1),
        .D(\shift_reg[1]_i_1_n_0 ),
        .Q(\shift_reg_reg_n_0_[1] ),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \shift_reg_reg[2] 
       (.C(clk),
        .CE(1'b1),
        .D(\shift_reg[2]_i_1_n_0 ),
        .Q(\shift_reg_reg_n_0_[2] ),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \shift_reg_reg[3] 
       (.C(clk),
        .CE(1'b1),
        .D(\shift_reg[3]_i_1_n_0 ),
        .Q(\shift_reg_reg_n_0_[3] ),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \shift_reg_reg[4] 
       (.C(clk),
        .CE(1'b1),
        .D(\shift_reg[4]_i_1_n_0 ),
        .Q(\shift_reg_reg_n_0_[4] ),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \shift_reg_reg[5] 
       (.C(clk),
        .CE(1'b1),
        .D(\shift_reg[5]_i_1_n_0 ),
        .Q(\shift_reg_reg_n_0_[5] ),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \shift_reg_reg[6] 
       (.C(clk),
        .CE(1'b1),
        .D(\shift_reg[6]_i_1_n_0 ),
        .Q(\shift_reg_reg_n_0_[6] ),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \shift_reg_reg[7] 
       (.C(clk),
        .CE(1'b1),
        .D(\shift_reg[7]_i_1_n_0 ),
        .Q(\shift_reg_reg_n_0_[7] ),
        .R(1'b0));
  (* COMPARATOR_THRESHOLD = "11" *) 
  CARRY4 state1_carry
       (.CI(1'b0),
        .CO({state1_carry_n_0,state1_carry_n_1,state1_carry_n_2,state1_carry_n_3}),
        .CYINIT(1'b0),
        .DI({state1_carry_i_1_n_0,state1_carry_i_2_n_0,state1_carry_i_3_n_0,state1_carry_i_4_n_0}),
        .O(NLW_state1_carry_O_UNCONNECTED[3:0]),
        .S({state1_carry_i_5_n_0,state1_carry_i_6_n_0,state1_carry_i_7_n_0,state1_carry_i_8_n_0}));
  LUT3 #(
    .INIT(8'h07)) 
    state1_carry_i_1
       (.I0(\byte_cnt_reg_n_0_[7] ),
        .I1(\byte_cnt_reg_n_0_[6] ),
        .I2(burst_len_reg),
        .O(state1_carry_i_1_n_0));
  LUT3 #(
    .INIT(8'h07)) 
    state1_carry_i_2
       (.I0(\byte_cnt_reg_n_0_[5] ),
        .I1(\byte_cnt_reg_n_0_[4] ),
        .I2(burst_len_reg),
        .O(state1_carry_i_2_n_0));
  LUT2 #(
    .INIT(4'h7)) 
    state1_carry_i_3
       (.I0(\byte_cnt_reg_n_0_[2] ),
        .I1(\byte_cnt_reg_n_0_[3] ),
        .O(state1_carry_i_3_n_0));
  LUT3 #(
    .INIT(8'h17)) 
    state1_carry_i_4
       (.I0(burst_len_reg),
        .I1(\byte_cnt_reg_n_0_[0] ),
        .I2(\byte_cnt_reg_n_0_[1] ),
        .O(state1_carry_i_4_n_0));
  LUT3 #(
    .INIT(8'h42)) 
    state1_carry_i_5
       (.I0(burst_len_reg),
        .I1(\byte_cnt_reg_n_0_[6] ),
        .I2(\byte_cnt_reg_n_0_[7] ),
        .O(state1_carry_i_5_n_0));
  LUT3 #(
    .INIT(8'h42)) 
    state1_carry_i_6
       (.I0(burst_len_reg),
        .I1(\byte_cnt_reg_n_0_[4] ),
        .I2(\byte_cnt_reg_n_0_[5] ),
        .O(state1_carry_i_6_n_0));
  LUT2 #(
    .INIT(4'h8)) 
    state1_carry_i_7
       (.I0(\byte_cnt_reg_n_0_[3] ),
        .I1(\byte_cnt_reg_n_0_[2] ),
        .O(state1_carry_i_7_n_0));
  LUT3 #(
    .INIT(8'h48)) 
    state1_carry_i_8
       (.I0(burst_len_reg),
        .I1(\byte_cnt_reg_n_0_[0] ),
        .I2(\byte_cnt_reg_n_0_[1] ),
        .O(state1_carry_i_8_n_0));
  (* SOFT_HLUTNM = "soft_lutpair30" *) 
  LUT1 #(
    .INIT(2'h1)) 
    \tick_cnt[0]_i_1 
       (.I0(tick_cnt_reg[0]),
        .O(\tick_cnt[0]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair30" *) 
  LUT2 #(
    .INIT(4'h6)) 
    \tick_cnt[1]_i_1 
       (.I0(tick_cnt_reg[0]),
        .I1(tick_cnt_reg[1]),
        .O(\tick_cnt[1]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair24" *) 
  LUT3 #(
    .INIT(8'h78)) 
    \tick_cnt[2]_i_1 
       (.I0(tick_cnt_reg[1]),
        .I1(tick_cnt_reg[0]),
        .I2(tick_cnt_reg[2]),
        .O(\tick_cnt[2]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair3" *) 
  LUT4 #(
    .INIT(16'h7F80)) 
    \tick_cnt[3]_i_1 
       (.I0(tick_cnt_reg[1]),
        .I1(tick_cnt_reg[0]),
        .I2(tick_cnt_reg[2]),
        .I3(tick_cnt_reg[3]),
        .O(\tick_cnt[3]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair3" *) 
  LUT5 #(
    .INIT(32'h7FFF8000)) 
    \tick_cnt[4]_i_1 
       (.I0(tick_cnt_reg[1]),
        .I1(tick_cnt_reg[2]),
        .I2(tick_cnt_reg[0]),
        .I3(tick_cnt_reg[3]),
        .I4(tick_cnt_reg[4]),
        .O(\tick_cnt[4]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'h0080FFFF)) 
    \tick_cnt[5]_i_1 
       (.I0(\tick_cnt[5]_i_3_n_0 ),
        .I1(tick_cnt_reg[5]),
        .I2(tick_cnt_reg[4]),
        .I3(tick_cnt_reg[1]),
        .I4(reset_n),
        .O(\tick_cnt[5]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'h7FFFFFFF80000000)) 
    \tick_cnt[5]_i_2 
       (.I0(tick_cnt_reg[2]),
        .I1(tick_cnt_reg[0]),
        .I2(tick_cnt_reg[3]),
        .I3(tick_cnt_reg[1]),
        .I4(tick_cnt_reg[4]),
        .I5(tick_cnt_reg[5]),
        .O(\tick_cnt[5]_i_2_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair24" *) 
  LUT3 #(
    .INIT(8'h80)) 
    \tick_cnt[5]_i_3 
       (.I0(tick_cnt_reg[2]),
        .I1(tick_cnt_reg[0]),
        .I2(tick_cnt_reg[3]),
        .O(\tick_cnt[5]_i_3_n_0 ));
  FDRE #(
    .INIT(1'b0)) 
    \tick_cnt_reg[0] 
       (.C(clk),
        .CE(1'b1),
        .D(\tick_cnt[0]_i_1_n_0 ),
        .Q(tick_cnt_reg[0]),
        .R(\tick_cnt[5]_i_1_n_0 ));
  FDRE #(
    .INIT(1'b0)) 
    \tick_cnt_reg[1] 
       (.C(clk),
        .CE(1'b1),
        .D(\tick_cnt[1]_i_1_n_0 ),
        .Q(tick_cnt_reg[1]),
        .R(\tick_cnt[5]_i_1_n_0 ));
  FDRE #(
    .INIT(1'b0)) 
    \tick_cnt_reg[2] 
       (.C(clk),
        .CE(1'b1),
        .D(\tick_cnt[2]_i_1_n_0 ),
        .Q(tick_cnt_reg[2]),
        .R(\tick_cnt[5]_i_1_n_0 ));
  FDRE #(
    .INIT(1'b0)) 
    \tick_cnt_reg[3] 
       (.C(clk),
        .CE(1'b1),
        .D(\tick_cnt[3]_i_1_n_0 ),
        .Q(tick_cnt_reg[3]),
        .R(\tick_cnt[5]_i_1_n_0 ));
  FDRE #(
    .INIT(1'b0)) 
    \tick_cnt_reg[4] 
       (.C(clk),
        .CE(1'b1),
        .D(\tick_cnt[4]_i_1_n_0 ),
        .Q(tick_cnt_reg[4]),
        .R(\tick_cnt[5]_i_1_n_0 ));
  FDRE #(
    .INIT(1'b0)) 
    \tick_cnt_reg[5] 
       (.C(clk),
        .CE(1'b1),
        .D(\tick_cnt[5]_i_2_n_0 ),
        .Q(tick_cnt_reg[5]),
        .R(\tick_cnt[5]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'h00005554)) 
    \timeout_cnt[0]_i_1 
       (.I0(i2c_done),
        .I1(\m_axis_tkeep_reg[0] [8]),
        .I2(\m_axis_tkeep_reg[0] [5]),
        .I3(\m_axis_tkeep_reg[0] [2]),
        .I4(\timeout_cnt_reg[0] ),
        .O(done_reg_2[0]));
  LUT5 #(
    .INIT(32'h55540000)) 
    \timeout_cnt[10]_i_1 
       (.I0(i2c_done),
        .I1(\m_axis_tkeep_reg[0] [8]),
        .I2(\m_axis_tkeep_reg[0] [5]),
        .I3(\m_axis_tkeep_reg[0] [2]),
        .I4(\timeout_cnt_reg[12] [1]),
        .O(done_reg_2[10]));
  LUT5 #(
    .INIT(32'h55540000)) 
    \timeout_cnt[11]_i_1 
       (.I0(i2c_done),
        .I1(\m_axis_tkeep_reg[0] [8]),
        .I2(\m_axis_tkeep_reg[0] [5]),
        .I3(\m_axis_tkeep_reg[0] [2]),
        .I4(\timeout_cnt_reg[12] [2]),
        .O(done_reg_2[11]));
  LUT5 #(
    .INIT(32'h55540000)) 
    \timeout_cnt[12]_i_1 
       (.I0(i2c_done),
        .I1(\m_axis_tkeep_reg[0] [8]),
        .I2(\m_axis_tkeep_reg[0] [5]),
        .I3(\m_axis_tkeep_reg[0] [2]),
        .I4(\timeout_cnt_reg[12] [3]),
        .O(done_reg_2[12]));
  LUT5 #(
    .INIT(32'h55540000)) 
    \timeout_cnt[13]_i_1 
       (.I0(i2c_done),
        .I1(\m_axis_tkeep_reg[0] [8]),
        .I2(\m_axis_tkeep_reg[0] [5]),
        .I3(\m_axis_tkeep_reg[0] [2]),
        .I4(\timeout_cnt_reg[16] [0]),
        .O(done_reg_2[13]));
  LUT5 #(
    .INIT(32'h55540000)) 
    \timeout_cnt[14]_i_1 
       (.I0(i2c_done),
        .I1(\m_axis_tkeep_reg[0] [8]),
        .I2(\m_axis_tkeep_reg[0] [5]),
        .I3(\m_axis_tkeep_reg[0] [2]),
        .I4(\timeout_cnt_reg[16] [1]),
        .O(done_reg_2[14]));
  LUT5 #(
    .INIT(32'h55540000)) 
    \timeout_cnt[15]_i_1 
       (.I0(i2c_done),
        .I1(\m_axis_tkeep_reg[0] [8]),
        .I2(\m_axis_tkeep_reg[0] [5]),
        .I3(\m_axis_tkeep_reg[0] [2]),
        .I4(\timeout_cnt_reg[16] [2]),
        .O(done_reg_2[15]));
  LUT5 #(
    .INIT(32'h55540000)) 
    \timeout_cnt[16]_i_1 
       (.I0(i2c_done),
        .I1(\m_axis_tkeep_reg[0] [8]),
        .I2(\m_axis_tkeep_reg[0] [5]),
        .I3(\m_axis_tkeep_reg[0] [2]),
        .I4(\timeout_cnt_reg[16] [3]),
        .O(done_reg_2[16]));
  LUT5 #(
    .INIT(32'h55540000)) 
    \timeout_cnt[17]_i_1 
       (.I0(i2c_done),
        .I1(\m_axis_tkeep_reg[0] [8]),
        .I2(\m_axis_tkeep_reg[0] [5]),
        .I3(\m_axis_tkeep_reg[0] [2]),
        .I4(\timeout_cnt_reg[20] [0]),
        .O(done_reg_2[17]));
  LUT5 #(
    .INIT(32'h55540000)) 
    \timeout_cnt[18]_i_1 
       (.I0(i2c_done),
        .I1(\m_axis_tkeep_reg[0] [8]),
        .I2(\m_axis_tkeep_reg[0] [5]),
        .I3(\m_axis_tkeep_reg[0] [2]),
        .I4(\timeout_cnt_reg[20] [1]),
        .O(done_reg_2[18]));
  LUT5 #(
    .INIT(32'h55540000)) 
    \timeout_cnt[19]_i_1 
       (.I0(i2c_done),
        .I1(\m_axis_tkeep_reg[0] [8]),
        .I2(\m_axis_tkeep_reg[0] [5]),
        .I3(\m_axis_tkeep_reg[0] [2]),
        .I4(\timeout_cnt_reg[20] [2]),
        .O(done_reg_2[19]));
  LUT5 #(
    .INIT(32'h55540000)) 
    \timeout_cnt[1]_i_1 
       (.I0(i2c_done),
        .I1(\m_axis_tkeep_reg[0] [8]),
        .I2(\m_axis_tkeep_reg[0] [5]),
        .I3(\m_axis_tkeep_reg[0] [2]),
        .I4(O[0]),
        .O(done_reg_2[1]));
  LUT5 #(
    .INIT(32'h55540000)) 
    \timeout_cnt[20]_i_1 
       (.I0(i2c_done),
        .I1(\m_axis_tkeep_reg[0] [8]),
        .I2(\m_axis_tkeep_reg[0] [5]),
        .I3(\m_axis_tkeep_reg[0] [2]),
        .I4(\timeout_cnt_reg[20] [3]),
        .O(done_reg_2[20]));
  LUT5 #(
    .INIT(32'h55540000)) 
    \timeout_cnt[21]_i_1 
       (.I0(i2c_done),
        .I1(\m_axis_tkeep_reg[0] [8]),
        .I2(\m_axis_tkeep_reg[0] [5]),
        .I3(\m_axis_tkeep_reg[0] [2]),
        .I4(\timeout_cnt_reg[23] [0]),
        .O(done_reg_2[21]));
  LUT5 #(
    .INIT(32'h55540000)) 
    \timeout_cnt[22]_i_1 
       (.I0(i2c_done),
        .I1(\m_axis_tkeep_reg[0] [8]),
        .I2(\m_axis_tkeep_reg[0] [5]),
        .I3(\m_axis_tkeep_reg[0] [2]),
        .I4(\timeout_cnt_reg[23] [1]),
        .O(done_reg_2[22]));
  LUT5 #(
    .INIT(32'h55540000)) 
    \timeout_cnt[23]_i_2 
       (.I0(i2c_done),
        .I1(\m_axis_tkeep_reg[0] [8]),
        .I2(\m_axis_tkeep_reg[0] [5]),
        .I3(\m_axis_tkeep_reg[0] [2]),
        .I4(\timeout_cnt_reg[23] [2]),
        .O(done_reg_2[23]));
  LUT5 #(
    .INIT(32'h55540000)) 
    \timeout_cnt[2]_i_1 
       (.I0(i2c_done),
        .I1(\m_axis_tkeep_reg[0] [8]),
        .I2(\m_axis_tkeep_reg[0] [5]),
        .I3(\m_axis_tkeep_reg[0] [2]),
        .I4(O[1]),
        .O(done_reg_2[2]));
  LUT5 #(
    .INIT(32'h55540000)) 
    \timeout_cnt[3]_i_1 
       (.I0(i2c_done),
        .I1(\m_axis_tkeep_reg[0] [8]),
        .I2(\m_axis_tkeep_reg[0] [5]),
        .I3(\m_axis_tkeep_reg[0] [2]),
        .I4(O[2]),
        .O(done_reg_2[3]));
  LUT5 #(
    .INIT(32'h55540000)) 
    \timeout_cnt[4]_i_1 
       (.I0(i2c_done),
        .I1(\m_axis_tkeep_reg[0] [8]),
        .I2(\m_axis_tkeep_reg[0] [5]),
        .I3(\m_axis_tkeep_reg[0] [2]),
        .I4(O[3]),
        .O(done_reg_2[4]));
  LUT5 #(
    .INIT(32'h55540000)) 
    \timeout_cnt[5]_i_1 
       (.I0(i2c_done),
        .I1(\m_axis_tkeep_reg[0] [8]),
        .I2(\m_axis_tkeep_reg[0] [5]),
        .I3(\m_axis_tkeep_reg[0] [2]),
        .I4(\timeout_cnt_reg[8] [0]),
        .O(done_reg_2[5]));
  LUT5 #(
    .INIT(32'h55540000)) 
    \timeout_cnt[6]_i_1 
       (.I0(i2c_done),
        .I1(\m_axis_tkeep_reg[0] [8]),
        .I2(\m_axis_tkeep_reg[0] [5]),
        .I3(\m_axis_tkeep_reg[0] [2]),
        .I4(\timeout_cnt_reg[8] [1]),
        .O(done_reg_2[6]));
  LUT5 #(
    .INIT(32'h55540000)) 
    \timeout_cnt[7]_i_1 
       (.I0(i2c_done),
        .I1(\m_axis_tkeep_reg[0] [8]),
        .I2(\m_axis_tkeep_reg[0] [5]),
        .I3(\m_axis_tkeep_reg[0] [2]),
        .I4(\timeout_cnt_reg[8] [2]),
        .O(done_reg_2[7]));
  LUT5 #(
    .INIT(32'h55540000)) 
    \timeout_cnt[8]_i_1 
       (.I0(i2c_done),
        .I1(\m_axis_tkeep_reg[0] [8]),
        .I2(\m_axis_tkeep_reg[0] [5]),
        .I3(\m_axis_tkeep_reg[0] [2]),
        .I4(\timeout_cnt_reg[8] [3]),
        .O(done_reg_2[8]));
  LUT5 #(
    .INIT(32'h55540000)) 
    \timeout_cnt[9]_i_1 
       (.I0(i2c_done),
        .I1(\m_axis_tkeep_reg[0] [8]),
        .I2(\m_axis_tkeep_reg[0] [5]),
        .I3(\m_axis_tkeep_reg[0] [2]),
        .I4(\timeout_cnt_reg[12] [0]),
        .O(done_reg_2[9]));
  FDRE \wr_data_reg_reg[0] 
       (.C(clk),
        .CE(dev_addr_reg),
        .D(\wr_data_reg_reg[7]_0 [0]),
        .Q(wr_data_reg[0]),
        .R(1'b0));
  FDRE \wr_data_reg_reg[1] 
       (.C(clk),
        .CE(dev_addr_reg),
        .D(\wr_data_reg_reg[7]_0 [1]),
        .Q(wr_data_reg[1]),
        .R(1'b0));
  FDRE \wr_data_reg_reg[2] 
       (.C(clk),
        .CE(dev_addr_reg),
        .D(\wr_data_reg_reg[7]_0 [2]),
        .Q(wr_data_reg[2]),
        .R(1'b0));
  FDRE \wr_data_reg_reg[3] 
       (.C(clk),
        .CE(dev_addr_reg),
        .D(\wr_data_reg_reg[7]_0 [3]),
        .Q(wr_data_reg[3]),
        .R(1'b0));
  FDRE \wr_data_reg_reg[4] 
       (.C(clk),
        .CE(dev_addr_reg),
        .D(\wr_data_reg_reg[7]_0 [4]),
        .Q(wr_data_reg[4]),
        .R(1'b0));
  FDRE \wr_data_reg_reg[5] 
       (.C(clk),
        .CE(dev_addr_reg),
        .D(\wr_data_reg_reg[7]_0 [5]),
        .Q(wr_data_reg[5]),
        .R(1'b0));
  FDRE \wr_data_reg_reg[7] 
       (.C(clk),
        .CE(dev_addr_reg),
        .D(\wr_data_reg_reg[7]_0 [6]),
        .Q(wr_data_reg[7]),
        .R(1'b0));
endmodule

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_mpu6050_axis
   (Q,
    \phase_state_reg[0] ,
    \phase_state_reg[2] ,
    \phase_state_reg[1] ,
    \byte_idx_reg[3]_0 ,
    m_axis_tvalid,
    debug_i2c_tick,
    \q_cnt_reg[0] ,
    \q_cnt_reg[1] ,
    \bit_idx_reg[2] ,
    \bit_idx_reg[0] ,
    \bit_idx_reg[1] ,
    \FSM_sequential_state_reg[1] ,
    \FSM_sequential_state_reg[2] ,
    \rd_data_reg[7] ,
    m_axis_tdata,
    m_axis_tkeep,
    cmd_ready_reg_reg,
    i2c_cmd_valid_r_reg_0,
    byte_valid_reg,
    debug_mpu_state,
    debug_i2c_state,
    debug_i2c_busy,
    debug_i2c_error,
    scl_t,
    sda_t,
    clk,
    mpu_int,
    reset_n,
    sda_i,
    m_axis_tready);
  output [0:0]Q;
  output \phase_state_reg[0] ;
  output \phase_state_reg[2] ;
  output \phase_state_reg[1] ;
  output [3:0]\byte_idx_reg[3]_0 ;
  output m_axis_tvalid;
  output debug_i2c_tick;
  output \q_cnt_reg[0] ;
  output \q_cnt_reg[1] ;
  output \bit_idx_reg[2] ;
  output \bit_idx_reg[0] ;
  output \bit_idx_reg[1] ;
  output \FSM_sequential_state_reg[1] ;
  output \FSM_sequential_state_reg[2] ;
  output [7:0]\rd_data_reg[7] ;
  output [111:0]m_axis_tdata;
  output [0:0]m_axis_tkeep;
  output cmd_ready_reg_reg;
  output i2c_cmd_valid_r_reg_0;
  output byte_valid_reg;
  output [3:0]debug_mpu_state;
  output [0:0]debug_i2c_state;
  output debug_i2c_busy;
  output debug_i2c_error;
  output scl_t;
  output sda_t;
  input clk;
  input mpu_int;
  input reset_n;
  input sda_i;
  input m_axis_tready;

  wire \FSM_onehot_mpu_state[0]_i_1_n_0 ;
  wire \FSM_onehot_mpu_state[3]_i_1_n_0 ;
  wire \FSM_onehot_mpu_state[4]_i_1_n_0 ;
  wire \FSM_onehot_mpu_state[6]_i_1_n_0 ;
  wire \FSM_onehot_mpu_state[9]_i_10_n_0 ;
  wire \FSM_onehot_mpu_state[9]_i_2_n_0 ;
  wire \FSM_onehot_mpu_state[9]_i_3_n_0 ;
  wire \FSM_onehot_mpu_state[9]_i_5_n_0 ;
  wire \FSM_onehot_mpu_state[9]_i_6_n_0 ;
  wire \FSM_onehot_mpu_state[9]_i_7_n_0 ;
  wire \FSM_onehot_mpu_state[9]_i_9_n_0 ;
  wire \FSM_onehot_mpu_state_reg_n_0_[0] ;
  wire \FSM_onehot_mpu_state_reg_n_0_[1] ;
  wire \FSM_onehot_mpu_state_reg_n_0_[2] ;
  wire \FSM_onehot_mpu_state_reg_n_0_[3] ;
  wire \FSM_onehot_mpu_state_reg_n_0_[4] ;
  wire \FSM_onehot_mpu_state_reg_n_0_[5] ;
  wire \FSM_onehot_mpu_state_reg_n_0_[6] ;
  wire \FSM_onehot_mpu_state_reg_n_0_[8] ;
  wire \FSM_onehot_mpu_state_reg_n_0_[9] ;
  wire \FSM_sequential_state_reg[1] ;
  wire \FSM_sequential_state_reg[2] ;
  wire [0:0]Q;
  wire [15:0]ay_raw;
  wire [15:0]az_raw;
  wire \bit_idx_reg[0] ;
  wire \bit_idx_reg[1] ;
  wire \bit_idx_reg[2] ;
  wire [3:1]byte_idx;
  wire \byte_idx[0]_i_1_n_0 ;
  wire \byte_idx[3]_i_4_n_0 ;
  wire [3:0]\byte_idx_reg[3]_0 ;
  wire byte_valid_reg;
  wire clk;
  wire cmd_ready_reg_reg;
  wire [23:1]data0;
  wire debug_i2c_busy;
  wire debug_i2c_error;
  wire [0:0]debug_i2c_state;
  wire debug_i2c_tick;
  wire [3:0]debug_mpu_state;
  wire delay_cnt0_carry__0_n_0;
  wire delay_cnt0_carry__0_n_1;
  wire delay_cnt0_carry__0_n_2;
  wire delay_cnt0_carry__0_n_3;
  wire delay_cnt0_carry__1_n_0;
  wire delay_cnt0_carry__1_n_1;
  wire delay_cnt0_carry__1_n_2;
  wire delay_cnt0_carry__1_n_3;
  wire delay_cnt0_carry__2_n_0;
  wire delay_cnt0_carry__2_n_1;
  wire delay_cnt0_carry__2_n_2;
  wire delay_cnt0_carry__2_n_3;
  wire delay_cnt0_carry__3_n_0;
  wire delay_cnt0_carry__3_n_1;
  wire delay_cnt0_carry__3_n_2;
  wire delay_cnt0_carry__3_n_3;
  wire delay_cnt0_carry__4_n_2;
  wire delay_cnt0_carry__4_n_3;
  wire delay_cnt0_carry_n_0;
  wire delay_cnt0_carry_n_1;
  wire delay_cnt0_carry_n_2;
  wire delay_cnt0_carry_n_3;
  wire \delay_cnt[0]_i_1_n_0 ;
  wire \delay_cnt[10]_i_1_n_0 ;
  wire \delay_cnt[11]_i_1_n_0 ;
  wire \delay_cnt[12]_i_1_n_0 ;
  wire \delay_cnt[13]_i_1_n_0 ;
  wire \delay_cnt[14]_i_1_n_0 ;
  wire \delay_cnt[15]_i_1_n_0 ;
  wire \delay_cnt[16]_i_1_n_0 ;
  wire \delay_cnt[17]_i_1_n_0 ;
  wire \delay_cnt[18]_i_1_n_0 ;
  wire \delay_cnt[19]_i_1_n_0 ;
  wire \delay_cnt[1]_i_1_n_0 ;
  wire \delay_cnt[20]_i_1_n_0 ;
  wire \delay_cnt[21]_i_1_n_0 ;
  wire \delay_cnt[22]_i_1_n_0 ;
  wire \delay_cnt[23]_i_1_n_0 ;
  wire \delay_cnt[23]_i_2_n_0 ;
  wire \delay_cnt[2]_i_1_n_0 ;
  wire \delay_cnt[3]_i_1_n_0 ;
  wire \delay_cnt[4]_i_1_n_0 ;
  wire \delay_cnt[5]_i_1_n_0 ;
  wire \delay_cnt[6]_i_1_n_0 ;
  wire \delay_cnt[7]_i_1_n_0 ;
  wire \delay_cnt[8]_i_1_n_0 ;
  wire \delay_cnt[9]_i_1_n_0 ;
  wire \delay_cnt_reg_n_0_[0] ;
  wire \delay_cnt_reg_n_0_[10] ;
  wire \delay_cnt_reg_n_0_[11] ;
  wire \delay_cnt_reg_n_0_[12] ;
  wire \delay_cnt_reg_n_0_[13] ;
  wire \delay_cnt_reg_n_0_[14] ;
  wire \delay_cnt_reg_n_0_[15] ;
  wire \delay_cnt_reg_n_0_[16] ;
  wire \delay_cnt_reg_n_0_[17] ;
  wire \delay_cnt_reg_n_0_[18] ;
  wire \delay_cnt_reg_n_0_[19] ;
  wire \delay_cnt_reg_n_0_[1] ;
  wire \delay_cnt_reg_n_0_[20] ;
  wire \delay_cnt_reg_n_0_[21] ;
  wire \delay_cnt_reg_n_0_[22] ;
  wire \delay_cnt_reg_n_0_[23] ;
  wire \delay_cnt_reg_n_0_[2] ;
  wire \delay_cnt_reg_n_0_[3] ;
  wire \delay_cnt_reg_n_0_[4] ;
  wire \delay_cnt_reg_n_0_[5] ;
  wire \delay_cnt_reg_n_0_[6] ;
  wire \delay_cnt_reg_n_0_[7] ;
  wire \delay_cnt_reg_n_0_[8] ;
  wire \delay_cnt_reg_n_0_[9] ;
  wire [15:0]gy_raw;
  wire [15:0]gz_raw;
  wire \i2c_burst_len[3]_i_1_n_0 ;
  wire \i2c_burst_len_reg_n_0_[3] ;
  wire [1:1]i2c_cmd_type;
  wire \i2c_cmd_type[1]_i_1_n_0 ;
  wire \i2c_cmd_type_reg_n_0_[1] ;
  wire i2c_cmd_valid_r;
  wire i2c_cmd_valid_r_reg_0;
  wire i2c_done;
  wire \i2c_reg_addr[0]_i_1_n_0 ;
  wire \i2c_reg_addr[1]_i_1_n_0 ;
  wire \i2c_reg_addr[2]_i_1_n_0 ;
  wire \i2c_reg_addr[3]_i_1_n_0 ;
  wire \i2c_reg_addr[4]_i_1_n_0 ;
  wire \i2c_reg_addr[5]_i_1_n_0 ;
  wire \i2c_reg_addr[6]_i_1_n_0 ;
  wire \i2c_reg_addr_reg_n_0_[0] ;
  wire \i2c_reg_addr_reg_n_0_[1] ;
  wire \i2c_reg_addr_reg_n_0_[2] ;
  wire \i2c_reg_addr_reg_n_0_[3] ;
  wire \i2c_reg_addr_reg_n_0_[4] ;
  wire \i2c_reg_addr_reg_n_0_[5] ;
  wire \i2c_reg_addr_reg_n_0_[6] ;
  wire i2c_rst;
  wire [4:0]i2c_wr_data0_in;
  wire \i2c_wr_data[7]_i_1_n_0 ;
  wire \i2c_wr_data_reg_n_0_[0] ;
  wire \i2c_wr_data_reg_n_0_[1] ;
  wire \i2c_wr_data_reg_n_0_[2] ;
  wire \i2c_wr_data_reg_n_0_[3] ;
  wire \i2c_wr_data_reg_n_0_[4] ;
  wire \i2c_wr_data_reg_n_0_[5] ;
  wire \i2c_wr_data_reg_n_0_[7] ;
  wire \init_idx[2]_i_3_n_0 ;
  wire \init_idx_reg_n_0_[0] ;
  wire \init_idx_reg_n_0_[1] ;
  wire \init_idx_reg_n_0_[2] ;
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
  wire \m_axis_tdata[111]_i_10_n_0 ;
  wire \m_axis_tdata[111]_i_11_n_0 ;
  wire \m_axis_tdata[111]_i_12_n_0 ;
  wire \m_axis_tdata[111]_i_13_n_0 ;
  wire \m_axis_tdata[111]_i_3_n_0 ;
  wire \m_axis_tdata[111]_i_4_n_0 ;
  wire \m_axis_tdata[111]_i_5_n_0 ;
  wire \m_axis_tdata[111]_i_6_n_0 ;
  wire \m_axis_tdata[111]_i_7_n_0 ;
  wire \m_axis_tdata[111]_i_8_n_0 ;
  wire \m_axis_tdata[111]_i_9_n_0 ;
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
  wire [0:0]m_axis_tkeep;
  wire m_axis_tready;
  wire m_axis_tvalid;
  wire m_valid_reg_i_2_n_0;
  wire mpu_int;
  wire mpu_int_m;
  wire mpu_int_sync;
  wire \phase_state_reg[0] ;
  wire \phase_state_reg[1] ;
  wire \phase_state_reg[2] ;
  wire \q_cnt_reg[0] ;
  wire \q_cnt_reg[1] ;
  wire [0:0]\raw_bytes[0]_19 ;
  wire [0:0]\raw_bytes[10]_9 ;
  wire [0:0]\raw_bytes[11]_8 ;
  wire [0:0]\raw_bytes[12]_7 ;
  wire [0:0]\raw_bytes[13]_6 ;
  wire [0:0]\raw_bytes[1]_18 ;
  wire [0:0]\raw_bytes[2]_17 ;
  wire [0:0]\raw_bytes[3]_16 ;
  wire [0:0]\raw_bytes[4]_15 ;
  wire [0:0]\raw_bytes[5]_14 ;
  wire [0:0]\raw_bytes[6]_13 ;
  wire [0:0]\raw_bytes[7]_12 ;
  wire [0:0]\raw_bytes[8]_11 ;
  wire [0:0]\raw_bytes[9]_10 ;
  wire [7:0]\raw_bytes_reg[0]_1 ;
  wire [7:0]\raw_bytes_reg[1]_0 ;
  wire [7:0]\raw_bytes_reg[6]_3 ;
  wire [7:0]\raw_bytes_reg[7]_2 ;
  wire [7:0]\raw_bytes_reg[8]_5 ;
  wire [7:0]\raw_bytes_reg[9]_4 ;
  wire [7:0]\rd_data_reg[7] ;
  wire reset_n;
  wire scl_t;
  wire sda_i;
  wire sda_t;
  wire [23:0]timeout_cnt;
  wire \timeout_cnt0_inferred__0/i__carry__0_n_0 ;
  wire \timeout_cnt0_inferred__0/i__carry__0_n_1 ;
  wire \timeout_cnt0_inferred__0/i__carry__0_n_2 ;
  wire \timeout_cnt0_inferred__0/i__carry__0_n_3 ;
  wire \timeout_cnt0_inferred__0/i__carry__0_n_4 ;
  wire \timeout_cnt0_inferred__0/i__carry__0_n_5 ;
  wire \timeout_cnt0_inferred__0/i__carry__0_n_6 ;
  wire \timeout_cnt0_inferred__0/i__carry__0_n_7 ;
  wire \timeout_cnt0_inferred__0/i__carry__1_n_0 ;
  wire \timeout_cnt0_inferred__0/i__carry__1_n_1 ;
  wire \timeout_cnt0_inferred__0/i__carry__1_n_2 ;
  wire \timeout_cnt0_inferred__0/i__carry__1_n_3 ;
  wire \timeout_cnt0_inferred__0/i__carry__1_n_4 ;
  wire \timeout_cnt0_inferred__0/i__carry__1_n_5 ;
  wire \timeout_cnt0_inferred__0/i__carry__1_n_6 ;
  wire \timeout_cnt0_inferred__0/i__carry__1_n_7 ;
  wire \timeout_cnt0_inferred__0/i__carry__2_n_0 ;
  wire \timeout_cnt0_inferred__0/i__carry__2_n_1 ;
  wire \timeout_cnt0_inferred__0/i__carry__2_n_2 ;
  wire \timeout_cnt0_inferred__0/i__carry__2_n_3 ;
  wire \timeout_cnt0_inferred__0/i__carry__2_n_4 ;
  wire \timeout_cnt0_inferred__0/i__carry__2_n_5 ;
  wire \timeout_cnt0_inferred__0/i__carry__2_n_6 ;
  wire \timeout_cnt0_inferred__0/i__carry__2_n_7 ;
  wire \timeout_cnt0_inferred__0/i__carry__3_n_0 ;
  wire \timeout_cnt0_inferred__0/i__carry__3_n_1 ;
  wire \timeout_cnt0_inferred__0/i__carry__3_n_2 ;
  wire \timeout_cnt0_inferred__0/i__carry__3_n_3 ;
  wire \timeout_cnt0_inferred__0/i__carry__3_n_4 ;
  wire \timeout_cnt0_inferred__0/i__carry__3_n_5 ;
  wire \timeout_cnt0_inferred__0/i__carry__3_n_6 ;
  wire \timeout_cnt0_inferred__0/i__carry__3_n_7 ;
  wire \timeout_cnt0_inferred__0/i__carry__4_n_2 ;
  wire \timeout_cnt0_inferred__0/i__carry__4_n_3 ;
  wire \timeout_cnt0_inferred__0/i__carry__4_n_5 ;
  wire \timeout_cnt0_inferred__0/i__carry__4_n_6 ;
  wire \timeout_cnt0_inferred__0/i__carry__4_n_7 ;
  wire \timeout_cnt0_inferred__0/i__carry_n_0 ;
  wire \timeout_cnt0_inferred__0/i__carry_n_1 ;
  wire \timeout_cnt0_inferred__0/i__carry_n_2 ;
  wire \timeout_cnt0_inferred__0/i__carry_n_3 ;
  wire \timeout_cnt0_inferred__0/i__carry_n_4 ;
  wire \timeout_cnt0_inferred__0/i__carry_n_5 ;
  wire \timeout_cnt0_inferred__0/i__carry_n_6 ;
  wire \timeout_cnt0_inferred__0/i__carry_n_7 ;
  wire \timeout_cnt[23]_i_1_n_0 ;
  wire \timeout_cnt[23]_i_3_n_0 ;
  wire u_i2c_master_n_13;
  wire u_i2c_master_n_14;
  wire u_i2c_master_n_15;
  wire u_i2c_master_n_16;
  wire u_i2c_master_n_24;
  wire u_i2c_master_n_25;
  wire u_i2c_master_n_40;
  wire u_i2c_master_n_42;
  wire u_i2c_master_n_43;
  wire u_i2c_master_n_44;
  wire u_i2c_master_n_45;
  wire u_i2c_master_n_46;
  wire u_i2c_master_n_47;
  wire u_i2c_master_n_48;
  wire u_i2c_master_n_49;
  wire u_i2c_master_n_50;
  wire u_i2c_master_n_51;
  wire u_i2c_master_n_52;
  wire u_i2c_master_n_53;
  wire u_i2c_master_n_54;
  wire u_i2c_master_n_55;
  wire u_i2c_master_n_56;
  wire u_i2c_master_n_57;
  wire u_i2c_master_n_58;
  wire u_i2c_master_n_59;
  wire u_i2c_master_n_60;
  wire u_i2c_master_n_61;
  wire u_i2c_master_n_62;
  wire u_i2c_master_n_63;
  wire u_i2c_master_n_64;
  wire u_i2c_master_n_65;
  wire u_i2c_master_n_66;
  wire [3:2]NLW_delay_cnt0_carry__4_CO_UNCONNECTED;
  wire [3:3]NLW_delay_cnt0_carry__4_O_UNCONNECTED;
  wire [3:3]\NLW_m_axis_tdata_reg[111]_i_2_CO_UNCONNECTED ;
  wire [3:3]\NLW_m_axis_tdata_reg[31]_i_1_CO_UNCONNECTED ;
  wire [3:3]\NLW_m_axis_tdata_reg[47]_i_1_CO_UNCONNECTED ;
  wire [3:3]\NLW_m_axis_tdata_reg[95]_i_1_CO_UNCONNECTED ;
  wire [3:2]\NLW_timeout_cnt0_inferred__0/i__carry__4_CO_UNCONNECTED ;
  wire [3:3]\NLW_timeout_cnt0_inferred__0/i__carry__4_O_UNCONNECTED ;

  (* SOFT_HLUTNM = "soft_lutpair37" *) 
  LUT4 #(
    .INIT(16'h00FE)) 
    \FSM_onehot_mpu_state[0]_i_1 
       (.I0(\FSM_onehot_mpu_state_reg_n_0_[2] ),
        .I1(\FSM_onehot_mpu_state_reg_n_0_[5] ),
        .I2(\FSM_onehot_mpu_state_reg_n_0_[8] ),
        .I3(\m_axis_tdata[111]_i_3_n_0 ),
        .O(\FSM_onehot_mpu_state[0]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair40" *) 
  LUT2 #(
    .INIT(4'h8)) 
    \FSM_onehot_mpu_state[3]_i_1 
       (.I0(\m_axis_tdata[111]_i_3_n_0 ),
        .I1(\FSM_onehot_mpu_state_reg_n_0_[2] ),
        .O(\FSM_onehot_mpu_state[3]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'hFFFFFFFFA2AA0000)) 
    \FSM_onehot_mpu_state[4]_i_1 
       (.I0(\m_axis_tdata[111]_i_3_n_0 ),
        .I1(\init_idx_reg_n_0_[0] ),
        .I2(\init_idx_reg_n_0_[1] ),
        .I3(\init_idx_reg_n_0_[2] ),
        .I4(\FSM_onehot_mpu_state_reg_n_0_[5] ),
        .I5(\FSM_onehot_mpu_state_reg_n_0_[3] ),
        .O(\FSM_onehot_mpu_state[4]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'hFFFFFFFF00800000)) 
    \FSM_onehot_mpu_state[6]_i_1 
       (.I0(\m_axis_tdata[111]_i_3_n_0 ),
        .I1(\FSM_onehot_mpu_state_reg_n_0_[5] ),
        .I2(\init_idx_reg_n_0_[0] ),
        .I3(\init_idx_reg_n_0_[1] ),
        .I4(\init_idx_reg_n_0_[2] ),
        .I5(\FSM_onehot_mpu_state_reg_n_0_[9] ),
        .O(\FSM_onehot_mpu_state[6]_i_1_n_0 ));
  LUT4 #(
    .INIT(16'hFFDF)) 
    \FSM_onehot_mpu_state[9]_i_10 
       (.I0(\delay_cnt_reg_n_0_[7] ),
        .I1(\delay_cnt_reg_n_0_[6] ),
        .I2(\delay_cnt_reg_n_0_[9] ),
        .I3(\delay_cnt_reg_n_0_[8] ),
        .O(\FSM_onehot_mpu_state[9]_i_10_n_0 ));
  LUT2 #(
    .INIT(4'h8)) 
    \FSM_onehot_mpu_state[9]_i_2 
       (.I0(\m_axis_tdata[111]_i_3_n_0 ),
        .I1(\FSM_onehot_mpu_state_reg_n_0_[8] ),
        .O(\FSM_onehot_mpu_state[9]_i_2_n_0 ));
  LUT6 #(
    .INIT(64'hFFFFFFFFFFFFFEFF)) 
    \FSM_onehot_mpu_state[9]_i_3 
       (.I0(\FSM_onehot_mpu_state[9]_i_5_n_0 ),
        .I1(\delay_cnt_reg_n_0_[1] ),
        .I2(\delay_cnt_reg_n_0_[22] ),
        .I3(\delay_cnt_reg_n_0_[23] ),
        .I4(\FSM_onehot_mpu_state[9]_i_6_n_0 ),
        .I5(\FSM_onehot_mpu_state[9]_i_7_n_0 ),
        .O(\FSM_onehot_mpu_state[9]_i_3_n_0 ));
  LUT4 #(
    .INIT(16'hFFDF)) 
    \FSM_onehot_mpu_state[9]_i_5 
       (.I0(\delay_cnt_reg_n_0_[19] ),
        .I1(\delay_cnt_reg_n_0_[18] ),
        .I2(\delay_cnt_reg_n_0_[20] ),
        .I3(\delay_cnt_reg_n_0_[21] ),
        .O(\FSM_onehot_mpu_state[9]_i_5_n_0 ));
  LUT5 #(
    .INIT(32'hFFFFFBFF)) 
    \FSM_onehot_mpu_state[9]_i_6 
       (.I0(\delay_cnt_reg_n_0_[13] ),
        .I1(\delay_cnt_reg_n_0_[12] ),
        .I2(\delay_cnt_reg_n_0_[11] ),
        .I3(\delay_cnt_reg_n_0_[10] ),
        .I4(\FSM_onehot_mpu_state[9]_i_9_n_0 ),
        .O(\FSM_onehot_mpu_state[9]_i_6_n_0 ));
  LUT5 #(
    .INIT(32'hFFFFFFFE)) 
    \FSM_onehot_mpu_state[9]_i_7 
       (.I0(\delay_cnt_reg_n_0_[4] ),
        .I1(\delay_cnt_reg_n_0_[5] ),
        .I2(\delay_cnt_reg_n_0_[2] ),
        .I3(\delay_cnt_reg_n_0_[3] ),
        .I4(\FSM_onehot_mpu_state[9]_i_10_n_0 ),
        .O(\FSM_onehot_mpu_state[9]_i_7_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair41" *) 
  LUT3 #(
    .INIT(8'hFE)) 
    \FSM_onehot_mpu_state[9]_i_8 
       (.I0(i2c_cmd_type),
        .I1(\FSM_onehot_mpu_state_reg_n_0_[1] ),
        .I2(\FSM_onehot_mpu_state_reg_n_0_[4] ),
        .O(i2c_cmd_valid_r));
  LUT4 #(
    .INIT(16'hFFFD)) 
    \FSM_onehot_mpu_state[9]_i_9 
       (.I0(\delay_cnt_reg_n_0_[15] ),
        .I1(\delay_cnt_reg_n_0_[14] ),
        .I2(\delay_cnt_reg_n_0_[17] ),
        .I3(\delay_cnt_reg_n_0_[16] ),
        .O(\FSM_onehot_mpu_state[9]_i_9_n_0 ));
  (* FSM_ENCODED_STATES = "st_hw_reset_delay:0000001000,st_init_cmd:0000010000,st_hw_reset_wait:0000000100,st_hw_reset_cmd:0000000010,st_reset:0000000001,st_stream:1000000000,st_burst_cmd:0010000000,st_burst_wait:0100000000,st_wait_int:0001000000,st_init_wait:0000100000" *) 
  FDSE #(
    .INIT(1'b1)) 
    \FSM_onehot_mpu_state_reg[0] 
       (.C(clk),
        .CE(u_i2c_master_n_24),
        .D(\FSM_onehot_mpu_state[0]_i_1_n_0 ),
        .Q(\FSM_onehot_mpu_state_reg_n_0_[0] ),
        .S(i2c_rst));
  (* FSM_ENCODED_STATES = "st_hw_reset_delay:0000001000,st_init_cmd:0000010000,st_hw_reset_wait:0000000100,st_hw_reset_cmd:0000000010,st_reset:0000000001,st_stream:1000000000,st_burst_cmd:0010000000,st_burst_wait:0100000000,st_wait_int:0001000000,st_init_wait:0000100000" *) 
  FDRE #(
    .INIT(1'b0)) 
    \FSM_onehot_mpu_state_reg[1] 
       (.C(clk),
        .CE(u_i2c_master_n_24),
        .D(\FSM_onehot_mpu_state_reg_n_0_[0] ),
        .Q(\FSM_onehot_mpu_state_reg_n_0_[1] ),
        .R(i2c_rst));
  (* FSM_ENCODED_STATES = "st_hw_reset_delay:0000001000,st_init_cmd:0000010000,st_hw_reset_wait:0000000100,st_hw_reset_cmd:0000000010,st_reset:0000000001,st_stream:1000000000,st_burst_cmd:0010000000,st_burst_wait:0100000000,st_wait_int:0001000000,st_init_wait:0000100000" *) 
  FDRE #(
    .INIT(1'b0)) 
    \FSM_onehot_mpu_state_reg[2] 
       (.C(clk),
        .CE(u_i2c_master_n_24),
        .D(\FSM_onehot_mpu_state_reg_n_0_[1] ),
        .Q(\FSM_onehot_mpu_state_reg_n_0_[2] ),
        .R(i2c_rst));
  (* FSM_ENCODED_STATES = "st_hw_reset_delay:0000001000,st_init_cmd:0000010000,st_hw_reset_wait:0000000100,st_hw_reset_cmd:0000000010,st_reset:0000000001,st_stream:1000000000,st_burst_cmd:0010000000,st_burst_wait:0100000000,st_wait_int:0001000000,st_init_wait:0000100000" *) 
  FDRE #(
    .INIT(1'b0)) 
    \FSM_onehot_mpu_state_reg[3] 
       (.C(clk),
        .CE(u_i2c_master_n_24),
        .D(\FSM_onehot_mpu_state[3]_i_1_n_0 ),
        .Q(\FSM_onehot_mpu_state_reg_n_0_[3] ),
        .R(i2c_rst));
  (* FSM_ENCODED_STATES = "st_hw_reset_delay:0000001000,st_init_cmd:0000010000,st_hw_reset_wait:0000000100,st_hw_reset_cmd:0000000010,st_reset:0000000001,st_stream:1000000000,st_burst_cmd:0010000000,st_burst_wait:0100000000,st_wait_int:0001000000,st_init_wait:0000100000" *) 
  FDRE #(
    .INIT(1'b0)) 
    \FSM_onehot_mpu_state_reg[4] 
       (.C(clk),
        .CE(u_i2c_master_n_24),
        .D(\FSM_onehot_mpu_state[4]_i_1_n_0 ),
        .Q(\FSM_onehot_mpu_state_reg_n_0_[4] ),
        .R(i2c_rst));
  (* FSM_ENCODED_STATES = "st_hw_reset_delay:0000001000,st_init_cmd:0000010000,st_hw_reset_wait:0000000100,st_hw_reset_cmd:0000000010,st_reset:0000000001,st_stream:1000000000,st_burst_cmd:0010000000,st_burst_wait:0100000000,st_wait_int:0001000000,st_init_wait:0000100000" *) 
  FDRE #(
    .INIT(1'b0)) 
    \FSM_onehot_mpu_state_reg[5] 
       (.C(clk),
        .CE(u_i2c_master_n_24),
        .D(\FSM_onehot_mpu_state_reg_n_0_[4] ),
        .Q(\FSM_onehot_mpu_state_reg_n_0_[5] ),
        .R(i2c_rst));
  (* FSM_ENCODED_STATES = "st_hw_reset_delay:0000001000,st_init_cmd:0000010000,st_hw_reset_wait:0000000100,st_hw_reset_cmd:0000000010,st_reset:0000000001,st_stream:1000000000,st_burst_cmd:0010000000,st_burst_wait:0100000000,st_wait_int:0001000000,st_init_wait:0000100000" *) 
  FDRE #(
    .INIT(1'b0)) 
    \FSM_onehot_mpu_state_reg[6] 
       (.C(clk),
        .CE(u_i2c_master_n_24),
        .D(\FSM_onehot_mpu_state[6]_i_1_n_0 ),
        .Q(\FSM_onehot_mpu_state_reg_n_0_[6] ),
        .R(i2c_rst));
  (* FSM_ENCODED_STATES = "st_hw_reset_delay:0000001000,st_init_cmd:0000010000,st_hw_reset_wait:0000000100,st_hw_reset_cmd:0000000010,st_reset:0000000001,st_stream:1000000000,st_burst_cmd:0010000000,st_burst_wait:0100000000,st_wait_int:0001000000,st_init_wait:0000100000" *) 
  FDRE #(
    .INIT(1'b0)) 
    \FSM_onehot_mpu_state_reg[7] 
       (.C(clk),
        .CE(u_i2c_master_n_24),
        .D(\FSM_onehot_mpu_state_reg_n_0_[6] ),
        .Q(i2c_cmd_type),
        .R(i2c_rst));
  (* FSM_ENCODED_STATES = "st_hw_reset_delay:0000001000,st_init_cmd:0000010000,st_hw_reset_wait:0000000100,st_hw_reset_cmd:0000000010,st_reset:0000000001,st_stream:1000000000,st_burst_cmd:0010000000,st_burst_wait:0100000000,st_wait_int:0001000000,st_init_wait:0000100000" *) 
  FDRE #(
    .INIT(1'b0)) 
    \FSM_onehot_mpu_state_reg[8] 
       (.C(clk),
        .CE(u_i2c_master_n_24),
        .D(i2c_cmd_type),
        .Q(\FSM_onehot_mpu_state_reg_n_0_[8] ),
        .R(i2c_rst));
  (* FSM_ENCODED_STATES = "st_hw_reset_delay:0000001000,st_init_cmd:0000010000,st_hw_reset_wait:0000000100,st_hw_reset_cmd:0000000010,st_reset:0000000001,st_stream:1000000000,st_burst_cmd:0010000000,st_burst_wait:0100000000,st_wait_int:0001000000,st_init_wait:0000100000" *) 
  FDRE #(
    .INIT(1'b0)) 
    \FSM_onehot_mpu_state_reg[9] 
       (.C(clk),
        .CE(u_i2c_master_n_24),
        .D(\FSM_onehot_mpu_state[9]_i_2_n_0 ),
        .Q(\FSM_onehot_mpu_state_reg_n_0_[9] ),
        .R(i2c_rst));
  (* SOFT_HLUTNM = "soft_lutpair55" *) 
  LUT2 #(
    .INIT(4'h2)) 
    \byte_idx[0]_i_1 
       (.I0(\FSM_onehot_mpu_state_reg_n_0_[8] ),
        .I1(\byte_idx_reg[3]_0 [0]),
        .O(\byte_idx[0]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair55" *) 
  LUT3 #(
    .INIT(8'h28)) 
    \byte_idx[1]_i_1 
       (.I0(\FSM_onehot_mpu_state_reg_n_0_[8] ),
        .I1(\byte_idx_reg[3]_0 [1]),
        .I2(\byte_idx_reg[3]_0 [0]),
        .O(byte_idx[1]));
  (* SOFT_HLUTNM = "soft_lutpair36" *) 
  LUT4 #(
    .INIT(16'h2888)) 
    \byte_idx[2]_i_1 
       (.I0(\FSM_onehot_mpu_state_reg_n_0_[8] ),
        .I1(\byte_idx_reg[3]_0 [2]),
        .I2(\byte_idx_reg[3]_0 [0]),
        .I3(\byte_idx_reg[3]_0 [1]),
        .O(byte_idx[2]));
  (* SOFT_HLUTNM = "soft_lutpair36" *) 
  LUT5 #(
    .INIT(32'h28888888)) 
    \byte_idx[3]_i_2 
       (.I0(\FSM_onehot_mpu_state_reg_n_0_[8] ),
        .I1(\byte_idx_reg[3]_0 [3]),
        .I2(\byte_idx_reg[3]_0 [2]),
        .I3(\byte_idx_reg[3]_0 [1]),
        .I4(\byte_idx_reg[3]_0 [0]),
        .O(byte_idx[3]));
  LUT2 #(
    .INIT(4'h7)) 
    \byte_idx[3]_i_4 
       (.I0(\byte_idx_reg[3]_0 [2]),
        .I1(\byte_idx_reg[3]_0 [3]),
        .O(\byte_idx[3]_i_4_n_0 ));
  FDRE #(
    .INIT(1'b0)) 
    \byte_idx_reg[0] 
       (.C(clk),
        .CE(u_i2c_master_n_25),
        .D(\byte_idx[0]_i_1_n_0 ),
        .Q(\byte_idx_reg[3]_0 [0]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \byte_idx_reg[1] 
       (.C(clk),
        .CE(u_i2c_master_n_25),
        .D(byte_idx[1]),
        .Q(\byte_idx_reg[3]_0 [1]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \byte_idx_reg[2] 
       (.C(clk),
        .CE(u_i2c_master_n_25),
        .D(byte_idx[2]),
        .Q(\byte_idx_reg[3]_0 [2]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \byte_idx_reg[3] 
       (.C(clk),
        .CE(u_i2c_master_n_25),
        .D(byte_idx[3]),
        .Q(\byte_idx_reg[3]_0 [3]),
        .R(i2c_rst));
  LUT5 #(
    .INIT(32'hFFFFFFFE)) 
    \debug_mpu_state[0]_INST_0 
       (.I0(\FSM_onehot_mpu_state_reg_n_0_[1] ),
        .I1(i2c_cmd_type),
        .I2(\FSM_onehot_mpu_state_reg_n_0_[3] ),
        .I3(\FSM_onehot_mpu_state_reg_n_0_[9] ),
        .I4(\FSM_onehot_mpu_state_reg_n_0_[5] ),
        .O(debug_mpu_state[0]));
  (* SOFT_HLUTNM = "soft_lutpair40" *) 
  LUT4 #(
    .INIT(16'hFFFE)) 
    \debug_mpu_state[1]_INST_0 
       (.I0(\FSM_onehot_mpu_state_reg_n_0_[3] ),
        .I1(\FSM_onehot_mpu_state_reg_n_0_[2] ),
        .I2(i2c_cmd_type),
        .I3(\FSM_onehot_mpu_state_reg_n_0_[6] ),
        .O(debug_mpu_state[1]));
  (* SOFT_HLUTNM = "soft_lutpair41" *) 
  LUT4 #(
    .INIT(16'hFFFE)) 
    \debug_mpu_state[2]_INST_0 
       (.I0(\FSM_onehot_mpu_state_reg_n_0_[5] ),
        .I1(\FSM_onehot_mpu_state_reg_n_0_[4] ),
        .I2(i2c_cmd_type),
        .I3(\FSM_onehot_mpu_state_reg_n_0_[6] ),
        .O(debug_mpu_state[2]));
  (* SOFT_HLUTNM = "soft_lutpair54" *) 
  LUT2 #(
    .INIT(4'hE)) 
    \debug_mpu_state[3]_INST_0 
       (.I0(\FSM_onehot_mpu_state_reg_n_0_[9] ),
        .I1(\FSM_onehot_mpu_state_reg_n_0_[8] ),
        .O(debug_mpu_state[3]));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 delay_cnt0_carry
       (.CI(1'b0),
        .CO({delay_cnt0_carry_n_0,delay_cnt0_carry_n_1,delay_cnt0_carry_n_2,delay_cnt0_carry_n_3}),
        .CYINIT(\delay_cnt_reg_n_0_[0] ),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(data0[4:1]),
        .S({\delay_cnt_reg_n_0_[4] ,\delay_cnt_reg_n_0_[3] ,\delay_cnt_reg_n_0_[2] ,\delay_cnt_reg_n_0_[1] }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 delay_cnt0_carry__0
       (.CI(delay_cnt0_carry_n_0),
        .CO({delay_cnt0_carry__0_n_0,delay_cnt0_carry__0_n_1,delay_cnt0_carry__0_n_2,delay_cnt0_carry__0_n_3}),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(data0[8:5]),
        .S({\delay_cnt_reg_n_0_[8] ,\delay_cnt_reg_n_0_[7] ,\delay_cnt_reg_n_0_[6] ,\delay_cnt_reg_n_0_[5] }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 delay_cnt0_carry__1
       (.CI(delay_cnt0_carry__0_n_0),
        .CO({delay_cnt0_carry__1_n_0,delay_cnt0_carry__1_n_1,delay_cnt0_carry__1_n_2,delay_cnt0_carry__1_n_3}),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(data0[12:9]),
        .S({\delay_cnt_reg_n_0_[12] ,\delay_cnt_reg_n_0_[11] ,\delay_cnt_reg_n_0_[10] ,\delay_cnt_reg_n_0_[9] }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 delay_cnt0_carry__2
       (.CI(delay_cnt0_carry__1_n_0),
        .CO({delay_cnt0_carry__2_n_0,delay_cnt0_carry__2_n_1,delay_cnt0_carry__2_n_2,delay_cnt0_carry__2_n_3}),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(data0[16:13]),
        .S({\delay_cnt_reg_n_0_[16] ,\delay_cnt_reg_n_0_[15] ,\delay_cnt_reg_n_0_[14] ,\delay_cnt_reg_n_0_[13] }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 delay_cnt0_carry__3
       (.CI(delay_cnt0_carry__2_n_0),
        .CO({delay_cnt0_carry__3_n_0,delay_cnt0_carry__3_n_1,delay_cnt0_carry__3_n_2,delay_cnt0_carry__3_n_3}),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(data0[20:17]),
        .S({\delay_cnt_reg_n_0_[20] ,\delay_cnt_reg_n_0_[19] ,\delay_cnt_reg_n_0_[18] ,\delay_cnt_reg_n_0_[17] }));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 delay_cnt0_carry__4
       (.CI(delay_cnt0_carry__3_n_0),
        .CO({NLW_delay_cnt0_carry__4_CO_UNCONNECTED[3:2],delay_cnt0_carry__4_n_2,delay_cnt0_carry__4_n_3}),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({NLW_delay_cnt0_carry__4_O_UNCONNECTED[3],data0[23:21]}),
        .S({1'b0,\delay_cnt_reg_n_0_[23] ,\delay_cnt_reg_n_0_[22] ,\delay_cnt_reg_n_0_[21] }));
  (* SOFT_HLUTNM = "soft_lutpair42" *) 
  LUT3 #(
    .INIT(8'h40)) 
    \delay_cnt[0]_i_1 
       (.I0(\delay_cnt_reg_n_0_[0] ),
        .I1(\FSM_onehot_mpu_state_reg_n_0_[3] ),
        .I2(\FSM_onehot_mpu_state[9]_i_3_n_0 ),
        .O(\delay_cnt[0]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair47" *) 
  LUT4 #(
    .INIT(16'hA800)) 
    \delay_cnt[10]_i_1 
       (.I0(\FSM_onehot_mpu_state_reg_n_0_[3] ),
        .I1(\FSM_onehot_mpu_state[9]_i_3_n_0 ),
        .I2(\delay_cnt_reg_n_0_[0] ),
        .I3(data0[10]),
        .O(\delay_cnt[10]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair47" *) 
  LUT4 #(
    .INIT(16'hA800)) 
    \delay_cnt[11]_i_1 
       (.I0(\FSM_onehot_mpu_state_reg_n_0_[3] ),
        .I1(\FSM_onehot_mpu_state[9]_i_3_n_0 ),
        .I2(\delay_cnt_reg_n_0_[0] ),
        .I3(data0[11]),
        .O(\delay_cnt[11]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair48" *) 
  LUT4 #(
    .INIT(16'hA800)) 
    \delay_cnt[12]_i_1 
       (.I0(\FSM_onehot_mpu_state_reg_n_0_[3] ),
        .I1(\FSM_onehot_mpu_state[9]_i_3_n_0 ),
        .I2(\delay_cnt_reg_n_0_[0] ),
        .I3(data0[12]),
        .O(\delay_cnt[12]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair48" *) 
  LUT4 #(
    .INIT(16'hA800)) 
    \delay_cnt[13]_i_1 
       (.I0(\FSM_onehot_mpu_state_reg_n_0_[3] ),
        .I1(\FSM_onehot_mpu_state[9]_i_3_n_0 ),
        .I2(\delay_cnt_reg_n_0_[0] ),
        .I3(data0[13]),
        .O(\delay_cnt[13]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair49" *) 
  LUT4 #(
    .INIT(16'hA800)) 
    \delay_cnt[14]_i_1 
       (.I0(\FSM_onehot_mpu_state_reg_n_0_[3] ),
        .I1(\FSM_onehot_mpu_state[9]_i_3_n_0 ),
        .I2(\delay_cnt_reg_n_0_[0] ),
        .I3(data0[14]),
        .O(\delay_cnt[14]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair49" *) 
  LUT4 #(
    .INIT(16'hA800)) 
    \delay_cnt[15]_i_1 
       (.I0(\FSM_onehot_mpu_state_reg_n_0_[3] ),
        .I1(\FSM_onehot_mpu_state[9]_i_3_n_0 ),
        .I2(\delay_cnt_reg_n_0_[0] ),
        .I3(data0[15]),
        .O(\delay_cnt[15]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair50" *) 
  LUT4 #(
    .INIT(16'hA800)) 
    \delay_cnt[16]_i_1 
       (.I0(\FSM_onehot_mpu_state_reg_n_0_[3] ),
        .I1(\FSM_onehot_mpu_state[9]_i_3_n_0 ),
        .I2(\delay_cnt_reg_n_0_[0] ),
        .I3(data0[16]),
        .O(\delay_cnt[16]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair50" *) 
  LUT4 #(
    .INIT(16'hA800)) 
    \delay_cnt[17]_i_1 
       (.I0(\FSM_onehot_mpu_state_reg_n_0_[3] ),
        .I1(\FSM_onehot_mpu_state[9]_i_3_n_0 ),
        .I2(\delay_cnt_reg_n_0_[0] ),
        .I3(data0[17]),
        .O(\delay_cnt[17]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair51" *) 
  LUT4 #(
    .INIT(16'hA800)) 
    \delay_cnt[18]_i_1 
       (.I0(\FSM_onehot_mpu_state_reg_n_0_[3] ),
        .I1(\FSM_onehot_mpu_state[9]_i_3_n_0 ),
        .I2(\delay_cnt_reg_n_0_[0] ),
        .I3(data0[18]),
        .O(\delay_cnt[18]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair51" *) 
  LUT4 #(
    .INIT(16'hA800)) 
    \delay_cnt[19]_i_1 
       (.I0(\FSM_onehot_mpu_state_reg_n_0_[3] ),
        .I1(\FSM_onehot_mpu_state[9]_i_3_n_0 ),
        .I2(\delay_cnt_reg_n_0_[0] ),
        .I3(data0[19]),
        .O(\delay_cnt[19]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair42" *) 
  LUT4 #(
    .INIT(16'hA800)) 
    \delay_cnt[1]_i_1 
       (.I0(\FSM_onehot_mpu_state_reg_n_0_[3] ),
        .I1(\FSM_onehot_mpu_state[9]_i_3_n_0 ),
        .I2(\delay_cnt_reg_n_0_[0] ),
        .I3(data0[1]),
        .O(\delay_cnt[1]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair52" *) 
  LUT4 #(
    .INIT(16'hA800)) 
    \delay_cnt[20]_i_1 
       (.I0(\FSM_onehot_mpu_state_reg_n_0_[3] ),
        .I1(\FSM_onehot_mpu_state[9]_i_3_n_0 ),
        .I2(\delay_cnt_reg_n_0_[0] ),
        .I3(data0[20]),
        .O(\delay_cnt[20]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair52" *) 
  LUT4 #(
    .INIT(16'hA800)) 
    \delay_cnt[21]_i_1 
       (.I0(\FSM_onehot_mpu_state_reg_n_0_[3] ),
        .I1(\FSM_onehot_mpu_state[9]_i_3_n_0 ),
        .I2(\delay_cnt_reg_n_0_[0] ),
        .I3(data0[21]),
        .O(\delay_cnt[21]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair53" *) 
  LUT4 #(
    .INIT(16'hA800)) 
    \delay_cnt[22]_i_1 
       (.I0(\FSM_onehot_mpu_state_reg_n_0_[3] ),
        .I1(\FSM_onehot_mpu_state[9]_i_3_n_0 ),
        .I2(\delay_cnt_reg_n_0_[0] ),
        .I3(data0[22]),
        .O(\delay_cnt[22]_i_1_n_0 ));
  LUT2 #(
    .INIT(4'hE)) 
    \delay_cnt[23]_i_1 
       (.I0(\FSM_onehot_mpu_state_reg_n_0_[3] ),
        .I1(\FSM_onehot_mpu_state_reg_n_0_[0] ),
        .O(\delay_cnt[23]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair53" *) 
  LUT4 #(
    .INIT(16'hA800)) 
    \delay_cnt[23]_i_2 
       (.I0(\FSM_onehot_mpu_state_reg_n_0_[3] ),
        .I1(\FSM_onehot_mpu_state[9]_i_3_n_0 ),
        .I2(\delay_cnt_reg_n_0_[0] ),
        .I3(data0[23]),
        .O(\delay_cnt[23]_i_2_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair43" *) 
  LUT4 #(
    .INIT(16'hA800)) 
    \delay_cnt[2]_i_1 
       (.I0(\FSM_onehot_mpu_state_reg_n_0_[3] ),
        .I1(\FSM_onehot_mpu_state[9]_i_3_n_0 ),
        .I2(\delay_cnt_reg_n_0_[0] ),
        .I3(data0[2]),
        .O(\delay_cnt[2]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair43" *) 
  LUT4 #(
    .INIT(16'hA800)) 
    \delay_cnt[3]_i_1 
       (.I0(\FSM_onehot_mpu_state_reg_n_0_[3] ),
        .I1(\FSM_onehot_mpu_state[9]_i_3_n_0 ),
        .I2(\delay_cnt_reg_n_0_[0] ),
        .I3(data0[3]),
        .O(\delay_cnt[3]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair44" *) 
  LUT4 #(
    .INIT(16'hA800)) 
    \delay_cnt[4]_i_1 
       (.I0(\FSM_onehot_mpu_state_reg_n_0_[3] ),
        .I1(\FSM_onehot_mpu_state[9]_i_3_n_0 ),
        .I2(\delay_cnt_reg_n_0_[0] ),
        .I3(data0[4]),
        .O(\delay_cnt[4]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair44" *) 
  LUT4 #(
    .INIT(16'hA800)) 
    \delay_cnt[5]_i_1 
       (.I0(\FSM_onehot_mpu_state_reg_n_0_[3] ),
        .I1(\FSM_onehot_mpu_state[9]_i_3_n_0 ),
        .I2(\delay_cnt_reg_n_0_[0] ),
        .I3(data0[5]),
        .O(\delay_cnt[5]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair45" *) 
  LUT4 #(
    .INIT(16'hA800)) 
    \delay_cnt[6]_i_1 
       (.I0(\FSM_onehot_mpu_state_reg_n_0_[3] ),
        .I1(\FSM_onehot_mpu_state[9]_i_3_n_0 ),
        .I2(\delay_cnt_reg_n_0_[0] ),
        .I3(data0[6]),
        .O(\delay_cnt[6]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair45" *) 
  LUT4 #(
    .INIT(16'hA800)) 
    \delay_cnt[7]_i_1 
       (.I0(\FSM_onehot_mpu_state_reg_n_0_[3] ),
        .I1(\FSM_onehot_mpu_state[9]_i_3_n_0 ),
        .I2(\delay_cnt_reg_n_0_[0] ),
        .I3(data0[7]),
        .O(\delay_cnt[7]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair46" *) 
  LUT4 #(
    .INIT(16'hA800)) 
    \delay_cnt[8]_i_1 
       (.I0(\FSM_onehot_mpu_state_reg_n_0_[3] ),
        .I1(\FSM_onehot_mpu_state[9]_i_3_n_0 ),
        .I2(\delay_cnt_reg_n_0_[0] ),
        .I3(data0[8]),
        .O(\delay_cnt[8]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair46" *) 
  LUT4 #(
    .INIT(16'hA800)) 
    \delay_cnt[9]_i_1 
       (.I0(\FSM_onehot_mpu_state_reg_n_0_[3] ),
        .I1(\FSM_onehot_mpu_state[9]_i_3_n_0 ),
        .I2(\delay_cnt_reg_n_0_[0] ),
        .I3(data0[9]),
        .O(\delay_cnt[9]_i_1_n_0 ));
  FDRE #(
    .INIT(1'b0)) 
    \delay_cnt_reg[0] 
       (.C(clk),
        .CE(\delay_cnt[23]_i_1_n_0 ),
        .D(\delay_cnt[0]_i_1_n_0 ),
        .Q(\delay_cnt_reg_n_0_[0] ),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \delay_cnt_reg[10] 
       (.C(clk),
        .CE(\delay_cnt[23]_i_1_n_0 ),
        .D(\delay_cnt[10]_i_1_n_0 ),
        .Q(\delay_cnt_reg_n_0_[10] ),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \delay_cnt_reg[11] 
       (.C(clk),
        .CE(\delay_cnt[23]_i_1_n_0 ),
        .D(\delay_cnt[11]_i_1_n_0 ),
        .Q(\delay_cnt_reg_n_0_[11] ),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \delay_cnt_reg[12] 
       (.C(clk),
        .CE(\delay_cnt[23]_i_1_n_0 ),
        .D(\delay_cnt[12]_i_1_n_0 ),
        .Q(\delay_cnt_reg_n_0_[12] ),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \delay_cnt_reg[13] 
       (.C(clk),
        .CE(\delay_cnt[23]_i_1_n_0 ),
        .D(\delay_cnt[13]_i_1_n_0 ),
        .Q(\delay_cnt_reg_n_0_[13] ),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \delay_cnt_reg[14] 
       (.C(clk),
        .CE(\delay_cnt[23]_i_1_n_0 ),
        .D(\delay_cnt[14]_i_1_n_0 ),
        .Q(\delay_cnt_reg_n_0_[14] ),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \delay_cnt_reg[15] 
       (.C(clk),
        .CE(\delay_cnt[23]_i_1_n_0 ),
        .D(\delay_cnt[15]_i_1_n_0 ),
        .Q(\delay_cnt_reg_n_0_[15] ),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \delay_cnt_reg[16] 
       (.C(clk),
        .CE(\delay_cnt[23]_i_1_n_0 ),
        .D(\delay_cnt[16]_i_1_n_0 ),
        .Q(\delay_cnt_reg_n_0_[16] ),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \delay_cnt_reg[17] 
       (.C(clk),
        .CE(\delay_cnt[23]_i_1_n_0 ),
        .D(\delay_cnt[17]_i_1_n_0 ),
        .Q(\delay_cnt_reg_n_0_[17] ),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \delay_cnt_reg[18] 
       (.C(clk),
        .CE(\delay_cnt[23]_i_1_n_0 ),
        .D(\delay_cnt[18]_i_1_n_0 ),
        .Q(\delay_cnt_reg_n_0_[18] ),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \delay_cnt_reg[19] 
       (.C(clk),
        .CE(\delay_cnt[23]_i_1_n_0 ),
        .D(\delay_cnt[19]_i_1_n_0 ),
        .Q(\delay_cnt_reg_n_0_[19] ),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \delay_cnt_reg[1] 
       (.C(clk),
        .CE(\delay_cnt[23]_i_1_n_0 ),
        .D(\delay_cnt[1]_i_1_n_0 ),
        .Q(\delay_cnt_reg_n_0_[1] ),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \delay_cnt_reg[20] 
       (.C(clk),
        .CE(\delay_cnt[23]_i_1_n_0 ),
        .D(\delay_cnt[20]_i_1_n_0 ),
        .Q(\delay_cnt_reg_n_0_[20] ),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \delay_cnt_reg[21] 
       (.C(clk),
        .CE(\delay_cnt[23]_i_1_n_0 ),
        .D(\delay_cnt[21]_i_1_n_0 ),
        .Q(\delay_cnt_reg_n_0_[21] ),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \delay_cnt_reg[22] 
       (.C(clk),
        .CE(\delay_cnt[23]_i_1_n_0 ),
        .D(\delay_cnt[22]_i_1_n_0 ),
        .Q(\delay_cnt_reg_n_0_[22] ),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \delay_cnt_reg[23] 
       (.C(clk),
        .CE(\delay_cnt[23]_i_1_n_0 ),
        .D(\delay_cnt[23]_i_2_n_0 ),
        .Q(\delay_cnt_reg_n_0_[23] ),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \delay_cnt_reg[2] 
       (.C(clk),
        .CE(\delay_cnt[23]_i_1_n_0 ),
        .D(\delay_cnt[2]_i_1_n_0 ),
        .Q(\delay_cnt_reg_n_0_[2] ),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \delay_cnt_reg[3] 
       (.C(clk),
        .CE(\delay_cnt[23]_i_1_n_0 ),
        .D(\delay_cnt[3]_i_1_n_0 ),
        .Q(\delay_cnt_reg_n_0_[3] ),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \delay_cnt_reg[4] 
       (.C(clk),
        .CE(\delay_cnt[23]_i_1_n_0 ),
        .D(\delay_cnt[4]_i_1_n_0 ),
        .Q(\delay_cnt_reg_n_0_[4] ),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \delay_cnt_reg[5] 
       (.C(clk),
        .CE(\delay_cnt[23]_i_1_n_0 ),
        .D(\delay_cnt[5]_i_1_n_0 ),
        .Q(\delay_cnt_reg_n_0_[5] ),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \delay_cnt_reg[6] 
       (.C(clk),
        .CE(\delay_cnt[23]_i_1_n_0 ),
        .D(\delay_cnt[6]_i_1_n_0 ),
        .Q(\delay_cnt_reg_n_0_[6] ),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \delay_cnt_reg[7] 
       (.C(clk),
        .CE(\delay_cnt[23]_i_1_n_0 ),
        .D(\delay_cnt[7]_i_1_n_0 ),
        .Q(\delay_cnt_reg_n_0_[7] ),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \delay_cnt_reg[8] 
       (.C(clk),
        .CE(\delay_cnt[23]_i_1_n_0 ),
        .D(\delay_cnt[8]_i_1_n_0 ),
        .Q(\delay_cnt_reg_n_0_[8] ),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \delay_cnt_reg[9] 
       (.C(clk),
        .CE(\delay_cnt[23]_i_1_n_0 ),
        .D(\delay_cnt[9]_i_1_n_0 ),
        .Q(\delay_cnt_reg_n_0_[9] ),
        .R(i2c_rst));
  LUT3 #(
    .INIT(8'hF8)) 
    \i2c_burst_len[3]_i_1 
       (.I0(reset_n),
        .I1(i2c_cmd_type),
        .I2(\i2c_burst_len_reg_n_0_[3] ),
        .O(\i2c_burst_len[3]_i_1_n_0 ));
  FDRE #(
    .INIT(1'b0)) 
    \i2c_burst_len_reg[3] 
       (.C(clk),
        .CE(1'b1),
        .D(\i2c_burst_len[3]_i_1_n_0 ),
        .Q(\i2c_burst_len_reg_n_0_[3] ),
        .R(1'b0));
  LUT4 #(
    .INIT(16'hFE00)) 
    \i2c_cmd_type[1]_i_1 
       (.I0(\FSM_onehot_mpu_state_reg_n_0_[4] ),
        .I1(\FSM_onehot_mpu_state_reg_n_0_[1] ),
        .I2(i2c_cmd_type),
        .I3(reset_n),
        .O(\i2c_cmd_type[1]_i_1_n_0 ));
  FDRE #(
    .INIT(1'b0)) 
    \i2c_cmd_type_reg[1] 
       (.C(clk),
        .CE(\i2c_cmd_type[1]_i_1_n_0 ),
        .D(i2c_cmd_type),
        .Q(\i2c_cmd_type_reg_n_0_[1] ),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    i2c_cmd_valid_r_reg
       (.C(clk),
        .CE(1'b1),
        .D(u_i2c_master_n_66),
        .Q(i2c_cmd_valid_r_reg_0),
        .R(i2c_rst));
  LUT6 #(
    .INIT(64'hFDFDFCFCFCFFFCFC)) 
    \i2c_reg_addr[0]_i_1 
       (.I0(\init_idx_reg_n_0_[2] ),
        .I1(\FSM_onehot_mpu_state_reg_n_0_[1] ),
        .I2(i2c_cmd_type),
        .I3(\init_idx_reg_n_0_[1] ),
        .I4(\FSM_onehot_mpu_state_reg_n_0_[4] ),
        .I5(\init_idx_reg_n_0_[0] ),
        .O(\i2c_reg_addr[0]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'hFDFCFCFCFDFFFCFC)) 
    \i2c_reg_addr[1]_i_1 
       (.I0(\init_idx_reg_n_0_[2] ),
        .I1(\FSM_onehot_mpu_state_reg_n_0_[1] ),
        .I2(i2c_cmd_type),
        .I3(\init_idx_reg_n_0_[1] ),
        .I4(\FSM_onehot_mpu_state_reg_n_0_[4] ),
        .I5(\init_idx_reg_n_0_[0] ),
        .O(\i2c_reg_addr[1]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair34" *) 
  LUT4 #(
    .INIT(16'h0040)) 
    \i2c_reg_addr[2]_i_1 
       (.I0(\init_idx_reg_n_0_[1] ),
        .I1(\init_idx_reg_n_0_[2] ),
        .I2(\FSM_onehot_mpu_state_reg_n_0_[4] ),
        .I3(\init_idx_reg_n_0_[0] ),
        .O(\i2c_reg_addr[2]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'hEFEFEEEEFFEFEEEE)) 
    \i2c_reg_addr[3]_i_1 
       (.I0(\FSM_onehot_mpu_state_reg_n_0_[1] ),
        .I1(i2c_cmd_type),
        .I2(\init_idx_reg_n_0_[2] ),
        .I3(\init_idx_reg_n_0_[0] ),
        .I4(\FSM_onehot_mpu_state_reg_n_0_[4] ),
        .I5(\init_idx_reg_n_0_[1] ),
        .O(\i2c_reg_addr[3]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair34" *) 
  LUT5 #(
    .INIT(32'hDFCCDECC)) 
    \i2c_reg_addr[4]_i_1 
       (.I0(\init_idx_reg_n_0_[1] ),
        .I1(i2c_cmd_type),
        .I2(\init_idx_reg_n_0_[2] ),
        .I3(\FSM_onehot_mpu_state_reg_n_0_[4] ),
        .I4(\init_idx_reg_n_0_[0] ),
        .O(\i2c_reg_addr[4]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'hFCFEFCFCFCFFFCFC)) 
    \i2c_reg_addr[5]_i_1 
       (.I0(\init_idx_reg_n_0_[2] ),
        .I1(\FSM_onehot_mpu_state_reg_n_0_[1] ),
        .I2(i2c_cmd_type),
        .I3(\init_idx_reg_n_0_[1] ),
        .I4(\FSM_onehot_mpu_state_reg_n_0_[4] ),
        .I5(\init_idx_reg_n_0_[0] ),
        .O(\i2c_reg_addr[5]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair35" *) 
  LUT5 #(
    .INIT(32'hFFFF0002)) 
    \i2c_reg_addr[6]_i_1 
       (.I0(\FSM_onehot_mpu_state_reg_n_0_[4] ),
        .I1(\init_idx_reg_n_0_[2] ),
        .I2(\init_idx_reg_n_0_[0] ),
        .I3(\init_idx_reg_n_0_[1] ),
        .I4(\FSM_onehot_mpu_state_reg_n_0_[1] ),
        .O(\i2c_reg_addr[6]_i_1_n_0 ));
  FDRE #(
    .INIT(1'b0)) 
    \i2c_reg_addr_reg[0] 
       (.C(clk),
        .CE(\i2c_cmd_type[1]_i_1_n_0 ),
        .D(\i2c_reg_addr[0]_i_1_n_0 ),
        .Q(\i2c_reg_addr_reg_n_0_[0] ),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \i2c_reg_addr_reg[1] 
       (.C(clk),
        .CE(\i2c_cmd_type[1]_i_1_n_0 ),
        .D(\i2c_reg_addr[1]_i_1_n_0 ),
        .Q(\i2c_reg_addr_reg_n_0_[1] ),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \i2c_reg_addr_reg[2] 
       (.C(clk),
        .CE(\i2c_cmd_type[1]_i_1_n_0 ),
        .D(\i2c_reg_addr[2]_i_1_n_0 ),
        .Q(\i2c_reg_addr_reg_n_0_[2] ),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \i2c_reg_addr_reg[3] 
       (.C(clk),
        .CE(\i2c_cmd_type[1]_i_1_n_0 ),
        .D(\i2c_reg_addr[3]_i_1_n_0 ),
        .Q(\i2c_reg_addr_reg_n_0_[3] ),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \i2c_reg_addr_reg[4] 
       (.C(clk),
        .CE(\i2c_cmd_type[1]_i_1_n_0 ),
        .D(\i2c_reg_addr[4]_i_1_n_0 ),
        .Q(\i2c_reg_addr_reg_n_0_[4] ),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \i2c_reg_addr_reg[5] 
       (.C(clk),
        .CE(\i2c_cmd_type[1]_i_1_n_0 ),
        .D(\i2c_reg_addr[5]_i_1_n_0 ),
        .Q(\i2c_reg_addr_reg_n_0_[5] ),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \i2c_reg_addr_reg[6] 
       (.C(clk),
        .CE(\i2c_cmd_type[1]_i_1_n_0 ),
        .D(\i2c_reg_addr[6]_i_1_n_0 ),
        .Q(\i2c_reg_addr_reg_n_0_[6] ),
        .R(1'b0));
  (* SOFT_HLUTNM = "soft_lutpair38" *) 
  LUT4 #(
    .INIT(16'h2030)) 
    \i2c_wr_data[0]_i_1 
       (.I0(\init_idx_reg_n_0_[0] ),
        .I1(\init_idx_reg_n_0_[1] ),
        .I2(\FSM_onehot_mpu_state_reg_n_0_[4] ),
        .I3(\init_idx_reg_n_0_[2] ),
        .O(i2c_wr_data0_in[0]));
  (* SOFT_HLUTNM = "soft_lutpair39" *) 
  LUT4 #(
    .INIT(16'h0008)) 
    \i2c_wr_data[1]_i_1 
       (.I0(\init_idx_reg_n_0_[0] ),
        .I1(\FSM_onehot_mpu_state_reg_n_0_[4] ),
        .I2(\init_idx_reg_n_0_[2] ),
        .I3(\init_idx_reg_n_0_[1] ),
        .O(i2c_wr_data0_in[1]));
  (* SOFT_HLUTNM = "soft_lutpair38" *) 
  LUT4 #(
    .INIT(16'h0440)) 
    \i2c_wr_data[2]_i_1 
       (.I0(\init_idx_reg_n_0_[2] ),
        .I1(\FSM_onehot_mpu_state_reg_n_0_[4] ),
        .I2(\init_idx_reg_n_0_[1] ),
        .I3(\init_idx_reg_n_0_[0] ),
        .O(i2c_wr_data0_in[2]));
  LUT4 #(
    .INIT(16'h0800)) 
    \i2c_wr_data[3]_i_1 
       (.I0(\init_idx_reg_n_0_[0] ),
        .I1(\FSM_onehot_mpu_state_reg_n_0_[4] ),
        .I2(\init_idx_reg_n_0_[2] ),
        .I3(\init_idx_reg_n_0_[1] ),
        .O(i2c_wr_data0_in[3]));
  (* SOFT_HLUTNM = "soft_lutpair39" *) 
  LUT4 #(
    .INIT(16'h2040)) 
    \i2c_wr_data[4]_i_1 
       (.I0(\init_idx_reg_n_0_[1] ),
        .I1(\init_idx_reg_n_0_[2] ),
        .I2(\FSM_onehot_mpu_state_reg_n_0_[4] ),
        .I3(\init_idx_reg_n_0_[0] ),
        .O(i2c_wr_data0_in[4]));
  LUT3 #(
    .INIT(8'hC8)) 
    \i2c_wr_data[7]_i_1 
       (.I0(\FSM_onehot_mpu_state_reg_n_0_[1] ),
        .I1(reset_n),
        .I2(\FSM_onehot_mpu_state_reg_n_0_[4] ),
        .O(\i2c_wr_data[7]_i_1_n_0 ));
  FDRE #(
    .INIT(1'b0)) 
    \i2c_wr_data_reg[0] 
       (.C(clk),
        .CE(\i2c_wr_data[7]_i_1_n_0 ),
        .D(i2c_wr_data0_in[0]),
        .Q(\i2c_wr_data_reg_n_0_[0] ),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \i2c_wr_data_reg[1] 
       (.C(clk),
        .CE(\i2c_wr_data[7]_i_1_n_0 ),
        .D(i2c_wr_data0_in[1]),
        .Q(\i2c_wr_data_reg_n_0_[1] ),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \i2c_wr_data_reg[2] 
       (.C(clk),
        .CE(\i2c_wr_data[7]_i_1_n_0 ),
        .D(i2c_wr_data0_in[2]),
        .Q(\i2c_wr_data_reg_n_0_[2] ),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \i2c_wr_data_reg[3] 
       (.C(clk),
        .CE(\i2c_wr_data[7]_i_1_n_0 ),
        .D(i2c_wr_data0_in[3]),
        .Q(\i2c_wr_data_reg_n_0_[3] ),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \i2c_wr_data_reg[4] 
       (.C(clk),
        .CE(\i2c_wr_data[7]_i_1_n_0 ),
        .D(i2c_wr_data0_in[4]),
        .Q(\i2c_wr_data_reg_n_0_[4] ),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \i2c_wr_data_reg[5] 
       (.C(clk),
        .CE(\i2c_wr_data[7]_i_1_n_0 ),
        .D(\i2c_reg_addr[2]_i_1_n_0 ),
        .Q(\i2c_wr_data_reg_n_0_[5] ),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \i2c_wr_data_reg[7] 
       (.C(clk),
        .CE(\i2c_wr_data[7]_i_1_n_0 ),
        .D(\FSM_onehot_mpu_state_reg_n_0_[1] ),
        .Q(\i2c_wr_data_reg_n_0_[7] ),
        .R(1'b0));
  (* SOFT_HLUTNM = "soft_lutpair35" *) 
  LUT3 #(
    .INIT(8'hDF)) 
    \init_idx[2]_i_3 
       (.I0(\init_idx_reg_n_0_[2] ),
        .I1(\init_idx_reg_n_0_[1] ),
        .I2(\init_idx_reg_n_0_[0] ),
        .O(\init_idx[2]_i_3_n_0 ));
  FDRE #(
    .INIT(1'b0)) 
    \init_idx_reg[0] 
       (.C(clk),
        .CE(1'b1),
        .D(u_i2c_master_n_16),
        .Q(\init_idx_reg_n_0_[0] ),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \init_idx_reg[1] 
       (.C(clk),
        .CE(1'b1),
        .D(u_i2c_master_n_15),
        .Q(\init_idx_reg_n_0_[1] ),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \init_idx_reg[2] 
       (.C(clk),
        .CE(1'b1),
        .D(u_i2c_master_n_14),
        .Q(\init_idx_reg_n_0_[2] ),
        .R(i2c_rst));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[103]_i_2 
       (.I0(gz_raw[7]),
        .O(\m_axis_tdata[103]_i_2_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[103]_i_3 
       (.I0(gz_raw[6]),
        .O(\m_axis_tdata[103]_i_3_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[103]_i_4 
       (.I0(gz_raw[5]),
        .O(\m_axis_tdata[103]_i_4_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[103]_i_5 
       (.I0(gz_raw[4]),
        .O(\m_axis_tdata[103]_i_5_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[107]_i_2 
       (.I0(gz_raw[11]),
        .O(\m_axis_tdata[107]_i_2_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[107]_i_3 
       (.I0(gz_raw[10]),
        .O(\m_axis_tdata[107]_i_3_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[107]_i_4 
       (.I0(gz_raw[9]),
        .O(\m_axis_tdata[107]_i_4_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[107]_i_5 
       (.I0(gz_raw[8]),
        .O(\m_axis_tdata[107]_i_5_n_0 ));
  LUT4 #(
    .INIT(16'hFFDF)) 
    \m_axis_tdata[111]_i_10 
       (.I0(timeout_cnt[9]),
        .I1(timeout_cnt[8]),
        .I2(timeout_cnt[10]),
        .I3(timeout_cnt[11]),
        .O(\m_axis_tdata[111]_i_10_n_0 ));
  LUT4 #(
    .INIT(16'hFFDF)) 
    \m_axis_tdata[111]_i_11 
       (.I0(timeout_cnt[12]),
        .I1(timeout_cnt[13]),
        .I2(timeout_cnt[15]),
        .I3(timeout_cnt[14]),
        .O(\m_axis_tdata[111]_i_11_n_0 ));
  LUT4 #(
    .INIT(16'hFFEF)) 
    \m_axis_tdata[111]_i_12 
       (.I0(timeout_cnt[5]),
        .I1(timeout_cnt[4]),
        .I2(timeout_cnt[7]),
        .I3(timeout_cnt[6]),
        .O(\m_axis_tdata[111]_i_12_n_0 ));
  LUT4 #(
    .INIT(16'hFFFE)) 
    \m_axis_tdata[111]_i_13 
       (.I0(timeout_cnt[1]),
        .I1(timeout_cnt[0]),
        .I2(timeout_cnt[3]),
        .I3(timeout_cnt[2]),
        .O(\m_axis_tdata[111]_i_13_n_0 ));
  LUT6 #(
    .INIT(64'hFFFFFFFFFFFFFFFE)) 
    \m_axis_tdata[111]_i_3 
       (.I0(\m_axis_tdata[111]_i_8_n_0 ),
        .I1(\m_axis_tdata[111]_i_9_n_0 ),
        .I2(\m_axis_tdata[111]_i_10_n_0 ),
        .I3(\m_axis_tdata[111]_i_11_n_0 ),
        .I4(\m_axis_tdata[111]_i_12_n_0 ),
        .I5(\m_axis_tdata[111]_i_13_n_0 ),
        .O(\m_axis_tdata[111]_i_3_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[111]_i_4 
       (.I0(gz_raw[15]),
        .O(\m_axis_tdata[111]_i_4_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[111]_i_5 
       (.I0(gz_raw[14]),
        .O(\m_axis_tdata[111]_i_5_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[111]_i_6 
       (.I0(gz_raw[13]),
        .O(\m_axis_tdata[111]_i_6_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[111]_i_7 
       (.I0(gz_raw[12]),
        .O(\m_axis_tdata[111]_i_7_n_0 ));
  LUT4 #(
    .INIT(16'hFFEF)) 
    \m_axis_tdata[111]_i_8 
       (.I0(timeout_cnt[17]),
        .I1(timeout_cnt[16]),
        .I2(timeout_cnt[19]),
        .I3(timeout_cnt[18]),
        .O(\m_axis_tdata[111]_i_8_n_0 ));
  LUT4 #(
    .INIT(16'hFFDF)) 
    \m_axis_tdata[111]_i_9 
       (.I0(timeout_cnt[20]),
        .I1(timeout_cnt[21]),
        .I2(timeout_cnt[23]),
        .I3(timeout_cnt[22]),
        .O(\m_axis_tdata[111]_i_9_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[19]_i_2 
       (.I0(ay_raw[3]),
        .O(\m_axis_tdata[19]_i_2_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[19]_i_3 
       (.I0(ay_raw[2]),
        .O(\m_axis_tdata[19]_i_3_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[19]_i_4 
       (.I0(ay_raw[1]),
        .O(\m_axis_tdata[19]_i_4_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[23]_i_2 
       (.I0(ay_raw[7]),
        .O(\m_axis_tdata[23]_i_2_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[23]_i_3 
       (.I0(ay_raw[6]),
        .O(\m_axis_tdata[23]_i_3_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[23]_i_4 
       (.I0(ay_raw[5]),
        .O(\m_axis_tdata[23]_i_4_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[23]_i_5 
       (.I0(ay_raw[4]),
        .O(\m_axis_tdata[23]_i_5_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[27]_i_2 
       (.I0(ay_raw[11]),
        .O(\m_axis_tdata[27]_i_2_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[27]_i_3 
       (.I0(ay_raw[10]),
        .O(\m_axis_tdata[27]_i_3_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[27]_i_4 
       (.I0(ay_raw[9]),
        .O(\m_axis_tdata[27]_i_4_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[27]_i_5 
       (.I0(ay_raw[8]),
        .O(\m_axis_tdata[27]_i_5_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[31]_i_2 
       (.I0(ay_raw[15]),
        .O(\m_axis_tdata[31]_i_2_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[31]_i_3 
       (.I0(ay_raw[14]),
        .O(\m_axis_tdata[31]_i_3_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[31]_i_4 
       (.I0(ay_raw[13]),
        .O(\m_axis_tdata[31]_i_4_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[31]_i_5 
       (.I0(ay_raw[12]),
        .O(\m_axis_tdata[31]_i_5_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[35]_i_2 
       (.I0(az_raw[3]),
        .O(\m_axis_tdata[35]_i_2_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[35]_i_3 
       (.I0(az_raw[2]),
        .O(\m_axis_tdata[35]_i_3_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[35]_i_4 
       (.I0(az_raw[1]),
        .O(\m_axis_tdata[35]_i_4_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[39]_i_2 
       (.I0(az_raw[7]),
        .O(\m_axis_tdata[39]_i_2_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[39]_i_3 
       (.I0(az_raw[6]),
        .O(\m_axis_tdata[39]_i_3_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[39]_i_4 
       (.I0(az_raw[5]),
        .O(\m_axis_tdata[39]_i_4_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[39]_i_5 
       (.I0(az_raw[4]),
        .O(\m_axis_tdata[39]_i_5_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[43]_i_2 
       (.I0(az_raw[11]),
        .O(\m_axis_tdata[43]_i_2_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[43]_i_3 
       (.I0(az_raw[10]),
        .O(\m_axis_tdata[43]_i_3_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[43]_i_4 
       (.I0(az_raw[9]),
        .O(\m_axis_tdata[43]_i_4_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[43]_i_5 
       (.I0(az_raw[8]),
        .O(\m_axis_tdata[43]_i_5_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[47]_i_2 
       (.I0(az_raw[15]),
        .O(\m_axis_tdata[47]_i_2_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[47]_i_3 
       (.I0(az_raw[14]),
        .O(\m_axis_tdata[47]_i_3_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[47]_i_4 
       (.I0(az_raw[13]),
        .O(\m_axis_tdata[47]_i_4_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[47]_i_5 
       (.I0(az_raw[12]),
        .O(\m_axis_tdata[47]_i_5_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[83]_i_2 
       (.I0(gy_raw[3]),
        .O(\m_axis_tdata[83]_i_2_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[83]_i_3 
       (.I0(gy_raw[2]),
        .O(\m_axis_tdata[83]_i_3_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[83]_i_4 
       (.I0(gy_raw[1]),
        .O(\m_axis_tdata[83]_i_4_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[87]_i_2 
       (.I0(gy_raw[7]),
        .O(\m_axis_tdata[87]_i_2_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[87]_i_3 
       (.I0(gy_raw[6]),
        .O(\m_axis_tdata[87]_i_3_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[87]_i_4 
       (.I0(gy_raw[5]),
        .O(\m_axis_tdata[87]_i_4_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[87]_i_5 
       (.I0(gy_raw[4]),
        .O(\m_axis_tdata[87]_i_5_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[91]_i_2 
       (.I0(gy_raw[11]),
        .O(\m_axis_tdata[91]_i_2_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[91]_i_3 
       (.I0(gy_raw[10]),
        .O(\m_axis_tdata[91]_i_3_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[91]_i_4 
       (.I0(gy_raw[9]),
        .O(\m_axis_tdata[91]_i_4_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[91]_i_5 
       (.I0(gy_raw[8]),
        .O(\m_axis_tdata[91]_i_5_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[95]_i_2 
       (.I0(gy_raw[15]),
        .O(\m_axis_tdata[95]_i_2_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[95]_i_3 
       (.I0(gy_raw[14]),
        .O(\m_axis_tdata[95]_i_3_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[95]_i_4 
       (.I0(gy_raw[13]),
        .O(\m_axis_tdata[95]_i_4_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[95]_i_5 
       (.I0(gy_raw[12]),
        .O(\m_axis_tdata[95]_i_5_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[99]_i_2 
       (.I0(gz_raw[3]),
        .O(\m_axis_tdata[99]_i_2_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[99]_i_3 
       (.I0(gz_raw[2]),
        .O(\m_axis_tdata[99]_i_3_n_0 ));
  LUT1 #(
    .INIT(2'h1)) 
    \m_axis_tdata[99]_i_4 
       (.I0(gz_raw[1]),
        .O(\m_axis_tdata[99]_i_4_n_0 ));
  FDRE \m_axis_tdata_reg[0] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[1]_0 [0]),
        .Q(m_axis_tdata[0]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[100] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(m_axis_tdata0[4]),
        .Q(m_axis_tdata[100]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[101] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(m_axis_tdata0[5]),
        .Q(m_axis_tdata[101]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[102] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(m_axis_tdata0[6]),
        .Q(m_axis_tdata[102]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[103] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(m_axis_tdata0[7]),
        .Q(m_axis_tdata[103]),
        .R(i2c_rst));
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
        .CE(u_i2c_master_n_40),
        .D(m_axis_tdata0[8]),
        .Q(m_axis_tdata[104]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[105] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(m_axis_tdata0[9]),
        .Q(m_axis_tdata[105]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[106] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(m_axis_tdata0[10]),
        .Q(m_axis_tdata[106]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[107] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(m_axis_tdata0[11]),
        .Q(m_axis_tdata[107]),
        .R(i2c_rst));
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
        .CE(u_i2c_master_n_40),
        .D(m_axis_tdata0[12]),
        .Q(m_axis_tdata[108]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[109] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(m_axis_tdata0[13]),
        .Q(m_axis_tdata[109]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[10] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[0]_1 [2]),
        .Q(m_axis_tdata[10]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[110] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(m_axis_tdata0[14]),
        .Q(m_axis_tdata[110]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[111] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(m_axis_tdata0[15]),
        .Q(m_axis_tdata[111]),
        .R(i2c_rst));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \m_axis_tdata_reg[111]_i_2 
       (.CI(\m_axis_tdata_reg[107]_i_1_n_0 ),
        .CO({\NLW_m_axis_tdata_reg[111]_i_2_CO_UNCONNECTED [3],\m_axis_tdata_reg[111]_i_2_n_1 ,\m_axis_tdata_reg[111]_i_2_n_2 ,\m_axis_tdata_reg[111]_i_2_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O(m_axis_tdata0[15:12]),
        .S({\m_axis_tdata[111]_i_4_n_0 ,\m_axis_tdata[111]_i_5_n_0 ,\m_axis_tdata[111]_i_6_n_0 ,\m_axis_tdata[111]_i_7_n_0 }));
  FDRE \m_axis_tdata_reg[11] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[0]_1 [3]),
        .Q(m_axis_tdata[11]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[12] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[0]_1 [4]),
        .Q(m_axis_tdata[12]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[13] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[0]_1 [5]),
        .Q(m_axis_tdata[13]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[14] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[0]_1 [6]),
        .Q(m_axis_tdata[14]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[15] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[0]_1 [7]),
        .Q(m_axis_tdata[15]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[16] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[19]_i_1_n_7 ),
        .Q(m_axis_tdata[16]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[17] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[19]_i_1_n_6 ),
        .Q(m_axis_tdata[17]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[18] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[19]_i_1_n_5 ),
        .Q(m_axis_tdata[18]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[19] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[19]_i_1_n_4 ),
        .Q(m_axis_tdata[19]),
        .R(i2c_rst));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \m_axis_tdata_reg[19]_i_1 
       (.CI(1'b0),
        .CO({\m_axis_tdata_reg[19]_i_1_n_0 ,\m_axis_tdata_reg[19]_i_1_n_1 ,\m_axis_tdata_reg[19]_i_1_n_2 ,\m_axis_tdata_reg[19]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b1}),
        .O({\m_axis_tdata_reg[19]_i_1_n_4 ,\m_axis_tdata_reg[19]_i_1_n_5 ,\m_axis_tdata_reg[19]_i_1_n_6 ,\m_axis_tdata_reg[19]_i_1_n_7 }),
        .S({\m_axis_tdata[19]_i_2_n_0 ,\m_axis_tdata[19]_i_3_n_0 ,\m_axis_tdata[19]_i_4_n_0 ,ay_raw[0]}));
  FDRE \m_axis_tdata_reg[1] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[1]_0 [1]),
        .Q(m_axis_tdata[1]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[20] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[23]_i_1_n_7 ),
        .Q(m_axis_tdata[20]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[21] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[23]_i_1_n_6 ),
        .Q(m_axis_tdata[21]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[22] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[23]_i_1_n_5 ),
        .Q(m_axis_tdata[22]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[23] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[23]_i_1_n_4 ),
        .Q(m_axis_tdata[23]),
        .R(i2c_rst));
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
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[27]_i_1_n_7 ),
        .Q(m_axis_tdata[24]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[25] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[27]_i_1_n_6 ),
        .Q(m_axis_tdata[25]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[26] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[27]_i_1_n_5 ),
        .Q(m_axis_tdata[26]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[27] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[27]_i_1_n_4 ),
        .Q(m_axis_tdata[27]),
        .R(i2c_rst));
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
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[31]_i_1_n_7 ),
        .Q(m_axis_tdata[28]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[29] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[31]_i_1_n_6 ),
        .Q(m_axis_tdata[29]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[2] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[1]_0 [2]),
        .Q(m_axis_tdata[2]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[30] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[31]_i_1_n_5 ),
        .Q(m_axis_tdata[30]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[31] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[31]_i_1_n_4 ),
        .Q(m_axis_tdata[31]),
        .R(i2c_rst));
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
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[35]_i_1_n_7 ),
        .Q(m_axis_tdata[32]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[33] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[35]_i_1_n_6 ),
        .Q(m_axis_tdata[33]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[34] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[35]_i_1_n_5 ),
        .Q(m_axis_tdata[34]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[35] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[35]_i_1_n_4 ),
        .Q(m_axis_tdata[35]),
        .R(i2c_rst));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \m_axis_tdata_reg[35]_i_1 
       (.CI(1'b0),
        .CO({\m_axis_tdata_reg[35]_i_1_n_0 ,\m_axis_tdata_reg[35]_i_1_n_1 ,\m_axis_tdata_reg[35]_i_1_n_2 ,\m_axis_tdata_reg[35]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b1}),
        .O({\m_axis_tdata_reg[35]_i_1_n_4 ,\m_axis_tdata_reg[35]_i_1_n_5 ,\m_axis_tdata_reg[35]_i_1_n_6 ,\m_axis_tdata_reg[35]_i_1_n_7 }),
        .S({\m_axis_tdata[35]_i_2_n_0 ,\m_axis_tdata[35]_i_3_n_0 ,\m_axis_tdata[35]_i_4_n_0 ,az_raw[0]}));
  FDRE \m_axis_tdata_reg[36] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[39]_i_1_n_7 ),
        .Q(m_axis_tdata[36]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[37] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[39]_i_1_n_6 ),
        .Q(m_axis_tdata[37]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[38] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[39]_i_1_n_5 ),
        .Q(m_axis_tdata[38]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[39] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[39]_i_1_n_4 ),
        .Q(m_axis_tdata[39]),
        .R(i2c_rst));
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
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[1]_0 [3]),
        .Q(m_axis_tdata[3]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[40] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[43]_i_1_n_7 ),
        .Q(m_axis_tdata[40]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[41] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[43]_i_1_n_6 ),
        .Q(m_axis_tdata[41]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[42] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[43]_i_1_n_5 ),
        .Q(m_axis_tdata[42]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[43] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[43]_i_1_n_4 ),
        .Q(m_axis_tdata[43]),
        .R(i2c_rst));
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
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[47]_i_1_n_7 ),
        .Q(m_axis_tdata[44]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[45] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[47]_i_1_n_6 ),
        .Q(m_axis_tdata[45]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[46] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[47]_i_1_n_5 ),
        .Q(m_axis_tdata[46]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[47] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[47]_i_1_n_4 ),
        .Q(m_axis_tdata[47]),
        .R(i2c_rst));
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
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[7]_2 [0]),
        .Q(m_axis_tdata[48]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[49] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[7]_2 [1]),
        .Q(m_axis_tdata[49]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[4] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[1]_0 [4]),
        .Q(m_axis_tdata[4]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[50] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[7]_2 [2]),
        .Q(m_axis_tdata[50]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[51] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[7]_2 [3]),
        .Q(m_axis_tdata[51]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[52] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[7]_2 [4]),
        .Q(m_axis_tdata[52]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[53] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[7]_2 [5]),
        .Q(m_axis_tdata[53]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[54] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[7]_2 [6]),
        .Q(m_axis_tdata[54]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[55] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[7]_2 [7]),
        .Q(m_axis_tdata[55]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[56] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[6]_3 [0]),
        .Q(m_axis_tdata[56]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[57] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[6]_3 [1]),
        .Q(m_axis_tdata[57]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[58] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[6]_3 [2]),
        .Q(m_axis_tdata[58]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[59] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[6]_3 [3]),
        .Q(m_axis_tdata[59]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[5] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[1]_0 [5]),
        .Q(m_axis_tdata[5]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[60] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[6]_3 [4]),
        .Q(m_axis_tdata[60]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[61] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[6]_3 [5]),
        .Q(m_axis_tdata[61]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[62] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[6]_3 [6]),
        .Q(m_axis_tdata[62]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[63] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[6]_3 [7]),
        .Q(m_axis_tdata[63]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[64] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[9]_4 [0]),
        .Q(m_axis_tdata[64]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[65] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[9]_4 [1]),
        .Q(m_axis_tdata[65]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[66] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[9]_4 [2]),
        .Q(m_axis_tdata[66]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[67] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[9]_4 [3]),
        .Q(m_axis_tdata[67]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[68] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[9]_4 [4]),
        .Q(m_axis_tdata[68]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[69] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[9]_4 [5]),
        .Q(m_axis_tdata[69]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[6] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[1]_0 [6]),
        .Q(m_axis_tdata[6]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[70] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[9]_4 [6]),
        .Q(m_axis_tdata[70]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[71] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[9]_4 [7]),
        .Q(m_axis_tdata[71]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[72] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[8]_5 [0]),
        .Q(m_axis_tdata[72]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[73] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[8]_5 [1]),
        .Q(m_axis_tdata[73]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[74] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[8]_5 [2]),
        .Q(m_axis_tdata[74]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[75] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[8]_5 [3]),
        .Q(m_axis_tdata[75]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[76] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[8]_5 [4]),
        .Q(m_axis_tdata[76]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[77] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[8]_5 [5]),
        .Q(m_axis_tdata[77]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[78] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[8]_5 [6]),
        .Q(m_axis_tdata[78]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[79] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[8]_5 [7]),
        .Q(m_axis_tdata[79]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[7] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[1]_0 [7]),
        .Q(m_axis_tdata[7]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[80] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[83]_i_1_n_7 ),
        .Q(m_axis_tdata[80]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[81] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[83]_i_1_n_6 ),
        .Q(m_axis_tdata[81]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[82] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[83]_i_1_n_5 ),
        .Q(m_axis_tdata[82]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[83] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[83]_i_1_n_4 ),
        .Q(m_axis_tdata[83]),
        .R(i2c_rst));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \m_axis_tdata_reg[83]_i_1 
       (.CI(1'b0),
        .CO({\m_axis_tdata_reg[83]_i_1_n_0 ,\m_axis_tdata_reg[83]_i_1_n_1 ,\m_axis_tdata_reg[83]_i_1_n_2 ,\m_axis_tdata_reg[83]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b1}),
        .O({\m_axis_tdata_reg[83]_i_1_n_4 ,\m_axis_tdata_reg[83]_i_1_n_5 ,\m_axis_tdata_reg[83]_i_1_n_6 ,\m_axis_tdata_reg[83]_i_1_n_7 }),
        .S({\m_axis_tdata[83]_i_2_n_0 ,\m_axis_tdata[83]_i_3_n_0 ,\m_axis_tdata[83]_i_4_n_0 ,gy_raw[0]}));
  FDRE \m_axis_tdata_reg[84] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[87]_i_1_n_7 ),
        .Q(m_axis_tdata[84]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[85] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[87]_i_1_n_6 ),
        .Q(m_axis_tdata[85]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[86] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[87]_i_1_n_5 ),
        .Q(m_axis_tdata[86]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[87] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[87]_i_1_n_4 ),
        .Q(m_axis_tdata[87]),
        .R(i2c_rst));
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
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[91]_i_1_n_7 ),
        .Q(m_axis_tdata[88]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[89] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[91]_i_1_n_6 ),
        .Q(m_axis_tdata[89]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[8] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[0]_1 [0]),
        .Q(m_axis_tdata[8]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[90] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[91]_i_1_n_5 ),
        .Q(m_axis_tdata[90]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[91] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[91]_i_1_n_4 ),
        .Q(m_axis_tdata[91]),
        .R(i2c_rst));
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
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[95]_i_1_n_7 ),
        .Q(m_axis_tdata[92]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[93] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[95]_i_1_n_6 ),
        .Q(m_axis_tdata[93]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[94] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[95]_i_1_n_5 ),
        .Q(m_axis_tdata[94]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[95] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\m_axis_tdata_reg[95]_i_1_n_4 ),
        .Q(m_axis_tdata[95]),
        .R(i2c_rst));
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
        .CE(u_i2c_master_n_40),
        .D(m_axis_tdata0[0]),
        .Q(m_axis_tdata[96]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[97] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(m_axis_tdata0[1]),
        .Q(m_axis_tdata[97]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[98] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(m_axis_tdata0[2]),
        .Q(m_axis_tdata[98]),
        .R(i2c_rst));
  FDRE \m_axis_tdata_reg[99] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(m_axis_tdata0[3]),
        .Q(m_axis_tdata[99]),
        .R(i2c_rst));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \m_axis_tdata_reg[99]_i_1 
       (.CI(1'b0),
        .CO({\m_axis_tdata_reg[99]_i_1_n_0 ,\m_axis_tdata_reg[99]_i_1_n_1 ,\m_axis_tdata_reg[99]_i_1_n_2 ,\m_axis_tdata_reg[99]_i_1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b1}),
        .O(m_axis_tdata0[3:0]),
        .S({\m_axis_tdata[99]_i_2_n_0 ,\m_axis_tdata[99]_i_3_n_0 ,\m_axis_tdata[99]_i_4_n_0 ,gz_raw[0]}));
  FDRE \m_axis_tdata_reg[9] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(\raw_bytes_reg[0]_1 [1]),
        .Q(m_axis_tdata[9]),
        .R(i2c_rst));
  FDRE \m_axis_tkeep_reg[0] 
       (.C(clk),
        .CE(u_i2c_master_n_40),
        .D(i2c_done),
        .Q(m_axis_tkeep),
        .R(i2c_rst));
  (* SOFT_HLUTNM = "soft_lutpair54" *) 
  LUT3 #(
    .INIT(8'hF8)) 
    m_valid_reg_i_2
       (.I0(\FSM_onehot_mpu_state_reg_n_0_[9] ),
        .I1(m_axis_tready),
        .I2(\FSM_onehot_mpu_state_reg_n_0_[0] ),
        .O(m_valid_reg_i_2_n_0));
  FDRE #(
    .INIT(1'b0)) 
    m_valid_reg_reg
       (.C(clk),
        .CE(1'b1),
        .D(u_i2c_master_n_13),
        .Q(m_axis_tvalid),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    mpu_int_m_reg
       (.C(clk),
        .CE(1'b1),
        .D(mpu_int),
        .Q(mpu_int_m),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    mpu_int_sync_reg
       (.C(clk),
        .CE(1'b1),
        .D(mpu_int_m),
        .Q(mpu_int_sync),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[0][0] 
       (.C(clk),
        .CE(\raw_bytes[0]_19 ),
        .D(\rd_data_reg[7] [0]),
        .Q(\raw_bytes_reg[0]_1 [0]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[0][1] 
       (.C(clk),
        .CE(\raw_bytes[0]_19 ),
        .D(\rd_data_reg[7] [1]),
        .Q(\raw_bytes_reg[0]_1 [1]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[0][2] 
       (.C(clk),
        .CE(\raw_bytes[0]_19 ),
        .D(\rd_data_reg[7] [2]),
        .Q(\raw_bytes_reg[0]_1 [2]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[0][3] 
       (.C(clk),
        .CE(\raw_bytes[0]_19 ),
        .D(\rd_data_reg[7] [3]),
        .Q(\raw_bytes_reg[0]_1 [3]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[0][4] 
       (.C(clk),
        .CE(\raw_bytes[0]_19 ),
        .D(\rd_data_reg[7] [4]),
        .Q(\raw_bytes_reg[0]_1 [4]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[0][5] 
       (.C(clk),
        .CE(\raw_bytes[0]_19 ),
        .D(\rd_data_reg[7] [5]),
        .Q(\raw_bytes_reg[0]_1 [5]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[0][6] 
       (.C(clk),
        .CE(\raw_bytes[0]_19 ),
        .D(\rd_data_reg[7] [6]),
        .Q(\raw_bytes_reg[0]_1 [6]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[0][7] 
       (.C(clk),
        .CE(\raw_bytes[0]_19 ),
        .D(\rd_data_reg[7] [7]),
        .Q(\raw_bytes_reg[0]_1 [7]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[10][0] 
       (.C(clk),
        .CE(\raw_bytes[10]_9 ),
        .D(\rd_data_reg[7] [0]),
        .Q(gy_raw[8]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[10][1] 
       (.C(clk),
        .CE(\raw_bytes[10]_9 ),
        .D(\rd_data_reg[7] [1]),
        .Q(gy_raw[9]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[10][2] 
       (.C(clk),
        .CE(\raw_bytes[10]_9 ),
        .D(\rd_data_reg[7] [2]),
        .Q(gy_raw[10]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[10][3] 
       (.C(clk),
        .CE(\raw_bytes[10]_9 ),
        .D(\rd_data_reg[7] [3]),
        .Q(gy_raw[11]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[10][4] 
       (.C(clk),
        .CE(\raw_bytes[10]_9 ),
        .D(\rd_data_reg[7] [4]),
        .Q(gy_raw[12]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[10][5] 
       (.C(clk),
        .CE(\raw_bytes[10]_9 ),
        .D(\rd_data_reg[7] [5]),
        .Q(gy_raw[13]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[10][6] 
       (.C(clk),
        .CE(\raw_bytes[10]_9 ),
        .D(\rd_data_reg[7] [6]),
        .Q(gy_raw[14]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[10][7] 
       (.C(clk),
        .CE(\raw_bytes[10]_9 ),
        .D(\rd_data_reg[7] [7]),
        .Q(gy_raw[15]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[11][0] 
       (.C(clk),
        .CE(\raw_bytes[11]_8 ),
        .D(\rd_data_reg[7] [0]),
        .Q(gy_raw[0]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[11][1] 
       (.C(clk),
        .CE(\raw_bytes[11]_8 ),
        .D(\rd_data_reg[7] [1]),
        .Q(gy_raw[1]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[11][2] 
       (.C(clk),
        .CE(\raw_bytes[11]_8 ),
        .D(\rd_data_reg[7] [2]),
        .Q(gy_raw[2]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[11][3] 
       (.C(clk),
        .CE(\raw_bytes[11]_8 ),
        .D(\rd_data_reg[7] [3]),
        .Q(gy_raw[3]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[11][4] 
       (.C(clk),
        .CE(\raw_bytes[11]_8 ),
        .D(\rd_data_reg[7] [4]),
        .Q(gy_raw[4]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[11][5] 
       (.C(clk),
        .CE(\raw_bytes[11]_8 ),
        .D(\rd_data_reg[7] [5]),
        .Q(gy_raw[5]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[11][6] 
       (.C(clk),
        .CE(\raw_bytes[11]_8 ),
        .D(\rd_data_reg[7] [6]),
        .Q(gy_raw[6]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[11][7] 
       (.C(clk),
        .CE(\raw_bytes[11]_8 ),
        .D(\rd_data_reg[7] [7]),
        .Q(gy_raw[7]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[12][0] 
       (.C(clk),
        .CE(\raw_bytes[12]_7 ),
        .D(\rd_data_reg[7] [0]),
        .Q(gz_raw[8]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[12][1] 
       (.C(clk),
        .CE(\raw_bytes[12]_7 ),
        .D(\rd_data_reg[7] [1]),
        .Q(gz_raw[9]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[12][2] 
       (.C(clk),
        .CE(\raw_bytes[12]_7 ),
        .D(\rd_data_reg[7] [2]),
        .Q(gz_raw[10]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[12][3] 
       (.C(clk),
        .CE(\raw_bytes[12]_7 ),
        .D(\rd_data_reg[7] [3]),
        .Q(gz_raw[11]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[12][4] 
       (.C(clk),
        .CE(\raw_bytes[12]_7 ),
        .D(\rd_data_reg[7] [4]),
        .Q(gz_raw[12]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[12][5] 
       (.C(clk),
        .CE(\raw_bytes[12]_7 ),
        .D(\rd_data_reg[7] [5]),
        .Q(gz_raw[13]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[12][6] 
       (.C(clk),
        .CE(\raw_bytes[12]_7 ),
        .D(\rd_data_reg[7] [6]),
        .Q(gz_raw[14]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[12][7] 
       (.C(clk),
        .CE(\raw_bytes[12]_7 ),
        .D(\rd_data_reg[7] [7]),
        .Q(gz_raw[15]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[13][0] 
       (.C(clk),
        .CE(\raw_bytes[13]_6 ),
        .D(\rd_data_reg[7] [0]),
        .Q(gz_raw[0]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[13][1] 
       (.C(clk),
        .CE(\raw_bytes[13]_6 ),
        .D(\rd_data_reg[7] [1]),
        .Q(gz_raw[1]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[13][2] 
       (.C(clk),
        .CE(\raw_bytes[13]_6 ),
        .D(\rd_data_reg[7] [2]),
        .Q(gz_raw[2]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[13][3] 
       (.C(clk),
        .CE(\raw_bytes[13]_6 ),
        .D(\rd_data_reg[7] [3]),
        .Q(gz_raw[3]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[13][4] 
       (.C(clk),
        .CE(\raw_bytes[13]_6 ),
        .D(\rd_data_reg[7] [4]),
        .Q(gz_raw[4]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[13][5] 
       (.C(clk),
        .CE(\raw_bytes[13]_6 ),
        .D(\rd_data_reg[7] [5]),
        .Q(gz_raw[5]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[13][6] 
       (.C(clk),
        .CE(\raw_bytes[13]_6 ),
        .D(\rd_data_reg[7] [6]),
        .Q(gz_raw[6]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[13][7] 
       (.C(clk),
        .CE(\raw_bytes[13]_6 ),
        .D(\rd_data_reg[7] [7]),
        .Q(gz_raw[7]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[1][0] 
       (.C(clk),
        .CE(\raw_bytes[1]_18 ),
        .D(\rd_data_reg[7] [0]),
        .Q(\raw_bytes_reg[1]_0 [0]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[1][1] 
       (.C(clk),
        .CE(\raw_bytes[1]_18 ),
        .D(\rd_data_reg[7] [1]),
        .Q(\raw_bytes_reg[1]_0 [1]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[1][2] 
       (.C(clk),
        .CE(\raw_bytes[1]_18 ),
        .D(\rd_data_reg[7] [2]),
        .Q(\raw_bytes_reg[1]_0 [2]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[1][3] 
       (.C(clk),
        .CE(\raw_bytes[1]_18 ),
        .D(\rd_data_reg[7] [3]),
        .Q(\raw_bytes_reg[1]_0 [3]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[1][4] 
       (.C(clk),
        .CE(\raw_bytes[1]_18 ),
        .D(\rd_data_reg[7] [4]),
        .Q(\raw_bytes_reg[1]_0 [4]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[1][5] 
       (.C(clk),
        .CE(\raw_bytes[1]_18 ),
        .D(\rd_data_reg[7] [5]),
        .Q(\raw_bytes_reg[1]_0 [5]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[1][6] 
       (.C(clk),
        .CE(\raw_bytes[1]_18 ),
        .D(\rd_data_reg[7] [6]),
        .Q(\raw_bytes_reg[1]_0 [6]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[1][7] 
       (.C(clk),
        .CE(\raw_bytes[1]_18 ),
        .D(\rd_data_reg[7] [7]),
        .Q(\raw_bytes_reg[1]_0 [7]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[2][0] 
       (.C(clk),
        .CE(\raw_bytes[2]_17 ),
        .D(\rd_data_reg[7] [0]),
        .Q(ay_raw[8]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[2][1] 
       (.C(clk),
        .CE(\raw_bytes[2]_17 ),
        .D(\rd_data_reg[7] [1]),
        .Q(ay_raw[9]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[2][2] 
       (.C(clk),
        .CE(\raw_bytes[2]_17 ),
        .D(\rd_data_reg[7] [2]),
        .Q(ay_raw[10]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[2][3] 
       (.C(clk),
        .CE(\raw_bytes[2]_17 ),
        .D(\rd_data_reg[7] [3]),
        .Q(ay_raw[11]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[2][4] 
       (.C(clk),
        .CE(\raw_bytes[2]_17 ),
        .D(\rd_data_reg[7] [4]),
        .Q(ay_raw[12]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[2][5] 
       (.C(clk),
        .CE(\raw_bytes[2]_17 ),
        .D(\rd_data_reg[7] [5]),
        .Q(ay_raw[13]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[2][6] 
       (.C(clk),
        .CE(\raw_bytes[2]_17 ),
        .D(\rd_data_reg[7] [6]),
        .Q(ay_raw[14]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[2][7] 
       (.C(clk),
        .CE(\raw_bytes[2]_17 ),
        .D(\rd_data_reg[7] [7]),
        .Q(ay_raw[15]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[3][0] 
       (.C(clk),
        .CE(\raw_bytes[3]_16 ),
        .D(\rd_data_reg[7] [0]),
        .Q(ay_raw[0]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[3][1] 
       (.C(clk),
        .CE(\raw_bytes[3]_16 ),
        .D(\rd_data_reg[7] [1]),
        .Q(ay_raw[1]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[3][2] 
       (.C(clk),
        .CE(\raw_bytes[3]_16 ),
        .D(\rd_data_reg[7] [2]),
        .Q(ay_raw[2]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[3][3] 
       (.C(clk),
        .CE(\raw_bytes[3]_16 ),
        .D(\rd_data_reg[7] [3]),
        .Q(ay_raw[3]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[3][4] 
       (.C(clk),
        .CE(\raw_bytes[3]_16 ),
        .D(\rd_data_reg[7] [4]),
        .Q(ay_raw[4]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[3][5] 
       (.C(clk),
        .CE(\raw_bytes[3]_16 ),
        .D(\rd_data_reg[7] [5]),
        .Q(ay_raw[5]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[3][6] 
       (.C(clk),
        .CE(\raw_bytes[3]_16 ),
        .D(\rd_data_reg[7] [6]),
        .Q(ay_raw[6]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[3][7] 
       (.C(clk),
        .CE(\raw_bytes[3]_16 ),
        .D(\rd_data_reg[7] [7]),
        .Q(ay_raw[7]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[4][0] 
       (.C(clk),
        .CE(\raw_bytes[4]_15 ),
        .D(\rd_data_reg[7] [0]),
        .Q(az_raw[8]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[4][1] 
       (.C(clk),
        .CE(\raw_bytes[4]_15 ),
        .D(\rd_data_reg[7] [1]),
        .Q(az_raw[9]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[4][2] 
       (.C(clk),
        .CE(\raw_bytes[4]_15 ),
        .D(\rd_data_reg[7] [2]),
        .Q(az_raw[10]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[4][3] 
       (.C(clk),
        .CE(\raw_bytes[4]_15 ),
        .D(\rd_data_reg[7] [3]),
        .Q(az_raw[11]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[4][4] 
       (.C(clk),
        .CE(\raw_bytes[4]_15 ),
        .D(\rd_data_reg[7] [4]),
        .Q(az_raw[12]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[4][5] 
       (.C(clk),
        .CE(\raw_bytes[4]_15 ),
        .D(\rd_data_reg[7] [5]),
        .Q(az_raw[13]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[4][6] 
       (.C(clk),
        .CE(\raw_bytes[4]_15 ),
        .D(\rd_data_reg[7] [6]),
        .Q(az_raw[14]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[4][7] 
       (.C(clk),
        .CE(\raw_bytes[4]_15 ),
        .D(\rd_data_reg[7] [7]),
        .Q(az_raw[15]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[5][0] 
       (.C(clk),
        .CE(\raw_bytes[5]_14 ),
        .D(\rd_data_reg[7] [0]),
        .Q(az_raw[0]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[5][1] 
       (.C(clk),
        .CE(\raw_bytes[5]_14 ),
        .D(\rd_data_reg[7] [1]),
        .Q(az_raw[1]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[5][2] 
       (.C(clk),
        .CE(\raw_bytes[5]_14 ),
        .D(\rd_data_reg[7] [2]),
        .Q(az_raw[2]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[5][3] 
       (.C(clk),
        .CE(\raw_bytes[5]_14 ),
        .D(\rd_data_reg[7] [3]),
        .Q(az_raw[3]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[5][4] 
       (.C(clk),
        .CE(\raw_bytes[5]_14 ),
        .D(\rd_data_reg[7] [4]),
        .Q(az_raw[4]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[5][5] 
       (.C(clk),
        .CE(\raw_bytes[5]_14 ),
        .D(\rd_data_reg[7] [5]),
        .Q(az_raw[5]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[5][6] 
       (.C(clk),
        .CE(\raw_bytes[5]_14 ),
        .D(\rd_data_reg[7] [6]),
        .Q(az_raw[6]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[5][7] 
       (.C(clk),
        .CE(\raw_bytes[5]_14 ),
        .D(\rd_data_reg[7] [7]),
        .Q(az_raw[7]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[6][0] 
       (.C(clk),
        .CE(\raw_bytes[6]_13 ),
        .D(\rd_data_reg[7] [0]),
        .Q(\raw_bytes_reg[6]_3 [0]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[6][1] 
       (.C(clk),
        .CE(\raw_bytes[6]_13 ),
        .D(\rd_data_reg[7] [1]),
        .Q(\raw_bytes_reg[6]_3 [1]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[6][2] 
       (.C(clk),
        .CE(\raw_bytes[6]_13 ),
        .D(\rd_data_reg[7] [2]),
        .Q(\raw_bytes_reg[6]_3 [2]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[6][3] 
       (.C(clk),
        .CE(\raw_bytes[6]_13 ),
        .D(\rd_data_reg[7] [3]),
        .Q(\raw_bytes_reg[6]_3 [3]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[6][4] 
       (.C(clk),
        .CE(\raw_bytes[6]_13 ),
        .D(\rd_data_reg[7] [4]),
        .Q(\raw_bytes_reg[6]_3 [4]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[6][5] 
       (.C(clk),
        .CE(\raw_bytes[6]_13 ),
        .D(\rd_data_reg[7] [5]),
        .Q(\raw_bytes_reg[6]_3 [5]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[6][6] 
       (.C(clk),
        .CE(\raw_bytes[6]_13 ),
        .D(\rd_data_reg[7] [6]),
        .Q(\raw_bytes_reg[6]_3 [6]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[6][7] 
       (.C(clk),
        .CE(\raw_bytes[6]_13 ),
        .D(\rd_data_reg[7] [7]),
        .Q(\raw_bytes_reg[6]_3 [7]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[7][0] 
       (.C(clk),
        .CE(\raw_bytes[7]_12 ),
        .D(\rd_data_reg[7] [0]),
        .Q(\raw_bytes_reg[7]_2 [0]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[7][1] 
       (.C(clk),
        .CE(\raw_bytes[7]_12 ),
        .D(\rd_data_reg[7] [1]),
        .Q(\raw_bytes_reg[7]_2 [1]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[7][2] 
       (.C(clk),
        .CE(\raw_bytes[7]_12 ),
        .D(\rd_data_reg[7] [2]),
        .Q(\raw_bytes_reg[7]_2 [2]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[7][3] 
       (.C(clk),
        .CE(\raw_bytes[7]_12 ),
        .D(\rd_data_reg[7] [3]),
        .Q(\raw_bytes_reg[7]_2 [3]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[7][4] 
       (.C(clk),
        .CE(\raw_bytes[7]_12 ),
        .D(\rd_data_reg[7] [4]),
        .Q(\raw_bytes_reg[7]_2 [4]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[7][5] 
       (.C(clk),
        .CE(\raw_bytes[7]_12 ),
        .D(\rd_data_reg[7] [5]),
        .Q(\raw_bytes_reg[7]_2 [5]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[7][6] 
       (.C(clk),
        .CE(\raw_bytes[7]_12 ),
        .D(\rd_data_reg[7] [6]),
        .Q(\raw_bytes_reg[7]_2 [6]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[7][7] 
       (.C(clk),
        .CE(\raw_bytes[7]_12 ),
        .D(\rd_data_reg[7] [7]),
        .Q(\raw_bytes_reg[7]_2 [7]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[8][0] 
       (.C(clk),
        .CE(\raw_bytes[8]_11 ),
        .D(\rd_data_reg[7] [0]),
        .Q(\raw_bytes_reg[8]_5 [0]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[8][1] 
       (.C(clk),
        .CE(\raw_bytes[8]_11 ),
        .D(\rd_data_reg[7] [1]),
        .Q(\raw_bytes_reg[8]_5 [1]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[8][2] 
       (.C(clk),
        .CE(\raw_bytes[8]_11 ),
        .D(\rd_data_reg[7] [2]),
        .Q(\raw_bytes_reg[8]_5 [2]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[8][3] 
       (.C(clk),
        .CE(\raw_bytes[8]_11 ),
        .D(\rd_data_reg[7] [3]),
        .Q(\raw_bytes_reg[8]_5 [3]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[8][4] 
       (.C(clk),
        .CE(\raw_bytes[8]_11 ),
        .D(\rd_data_reg[7] [4]),
        .Q(\raw_bytes_reg[8]_5 [4]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[8][5] 
       (.C(clk),
        .CE(\raw_bytes[8]_11 ),
        .D(\rd_data_reg[7] [5]),
        .Q(\raw_bytes_reg[8]_5 [5]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[8][6] 
       (.C(clk),
        .CE(\raw_bytes[8]_11 ),
        .D(\rd_data_reg[7] [6]),
        .Q(\raw_bytes_reg[8]_5 [6]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[8][7] 
       (.C(clk),
        .CE(\raw_bytes[8]_11 ),
        .D(\rd_data_reg[7] [7]),
        .Q(\raw_bytes_reg[8]_5 [7]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[9][0] 
       (.C(clk),
        .CE(\raw_bytes[9]_10 ),
        .D(\rd_data_reg[7] [0]),
        .Q(\raw_bytes_reg[9]_4 [0]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[9][1] 
       (.C(clk),
        .CE(\raw_bytes[9]_10 ),
        .D(\rd_data_reg[7] [1]),
        .Q(\raw_bytes_reg[9]_4 [1]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[9][2] 
       (.C(clk),
        .CE(\raw_bytes[9]_10 ),
        .D(\rd_data_reg[7] [2]),
        .Q(\raw_bytes_reg[9]_4 [2]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[9][3] 
       (.C(clk),
        .CE(\raw_bytes[9]_10 ),
        .D(\rd_data_reg[7] [3]),
        .Q(\raw_bytes_reg[9]_4 [3]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[9][4] 
       (.C(clk),
        .CE(\raw_bytes[9]_10 ),
        .D(\rd_data_reg[7] [4]),
        .Q(\raw_bytes_reg[9]_4 [4]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[9][5] 
       (.C(clk),
        .CE(\raw_bytes[9]_10 ),
        .D(\rd_data_reg[7] [5]),
        .Q(\raw_bytes_reg[9]_4 [5]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[9][6] 
       (.C(clk),
        .CE(\raw_bytes[9]_10 ),
        .D(\rd_data_reg[7] [6]),
        .Q(\raw_bytes_reg[9]_4 [6]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \raw_bytes_reg[9][7] 
       (.C(clk),
        .CE(\raw_bytes[9]_10 ),
        .D(\rd_data_reg[7] [7]),
        .Q(\raw_bytes_reg[9]_4 [7]),
        .R(i2c_rst));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \timeout_cnt0_inferred__0/i__carry 
       (.CI(1'b0),
        .CO({\timeout_cnt0_inferred__0/i__carry_n_0 ,\timeout_cnt0_inferred__0/i__carry_n_1 ,\timeout_cnt0_inferred__0/i__carry_n_2 ,\timeout_cnt0_inferred__0/i__carry_n_3 }),
        .CYINIT(timeout_cnt[0]),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({\timeout_cnt0_inferred__0/i__carry_n_4 ,\timeout_cnt0_inferred__0/i__carry_n_5 ,\timeout_cnt0_inferred__0/i__carry_n_6 ,\timeout_cnt0_inferred__0/i__carry_n_7 }),
        .S(timeout_cnt[4:1]));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \timeout_cnt0_inferred__0/i__carry__0 
       (.CI(\timeout_cnt0_inferred__0/i__carry_n_0 ),
        .CO({\timeout_cnt0_inferred__0/i__carry__0_n_0 ,\timeout_cnt0_inferred__0/i__carry__0_n_1 ,\timeout_cnt0_inferred__0/i__carry__0_n_2 ,\timeout_cnt0_inferred__0/i__carry__0_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({\timeout_cnt0_inferred__0/i__carry__0_n_4 ,\timeout_cnt0_inferred__0/i__carry__0_n_5 ,\timeout_cnt0_inferred__0/i__carry__0_n_6 ,\timeout_cnt0_inferred__0/i__carry__0_n_7 }),
        .S(timeout_cnt[8:5]));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \timeout_cnt0_inferred__0/i__carry__1 
       (.CI(\timeout_cnt0_inferred__0/i__carry__0_n_0 ),
        .CO({\timeout_cnt0_inferred__0/i__carry__1_n_0 ,\timeout_cnt0_inferred__0/i__carry__1_n_1 ,\timeout_cnt0_inferred__0/i__carry__1_n_2 ,\timeout_cnt0_inferred__0/i__carry__1_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({\timeout_cnt0_inferred__0/i__carry__1_n_4 ,\timeout_cnt0_inferred__0/i__carry__1_n_5 ,\timeout_cnt0_inferred__0/i__carry__1_n_6 ,\timeout_cnt0_inferred__0/i__carry__1_n_7 }),
        .S(timeout_cnt[12:9]));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \timeout_cnt0_inferred__0/i__carry__2 
       (.CI(\timeout_cnt0_inferred__0/i__carry__1_n_0 ),
        .CO({\timeout_cnt0_inferred__0/i__carry__2_n_0 ,\timeout_cnt0_inferred__0/i__carry__2_n_1 ,\timeout_cnt0_inferred__0/i__carry__2_n_2 ,\timeout_cnt0_inferred__0/i__carry__2_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({\timeout_cnt0_inferred__0/i__carry__2_n_4 ,\timeout_cnt0_inferred__0/i__carry__2_n_5 ,\timeout_cnt0_inferred__0/i__carry__2_n_6 ,\timeout_cnt0_inferred__0/i__carry__2_n_7 }),
        .S(timeout_cnt[16:13]));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \timeout_cnt0_inferred__0/i__carry__3 
       (.CI(\timeout_cnt0_inferred__0/i__carry__2_n_0 ),
        .CO({\timeout_cnt0_inferred__0/i__carry__3_n_0 ,\timeout_cnt0_inferred__0/i__carry__3_n_1 ,\timeout_cnt0_inferred__0/i__carry__3_n_2 ,\timeout_cnt0_inferred__0/i__carry__3_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({\timeout_cnt0_inferred__0/i__carry__3_n_4 ,\timeout_cnt0_inferred__0/i__carry__3_n_5 ,\timeout_cnt0_inferred__0/i__carry__3_n_6 ,\timeout_cnt0_inferred__0/i__carry__3_n_7 }),
        .S(timeout_cnt[20:17]));
  (* ADDER_THRESHOLD = "35" *) 
  CARRY4 \timeout_cnt0_inferred__0/i__carry__4 
       (.CI(\timeout_cnt0_inferred__0/i__carry__3_n_0 ),
        .CO({\NLW_timeout_cnt0_inferred__0/i__carry__4_CO_UNCONNECTED [3:2],\timeout_cnt0_inferred__0/i__carry__4_n_2 ,\timeout_cnt0_inferred__0/i__carry__4_n_3 }),
        .CYINIT(1'b0),
        .DI({1'b0,1'b0,1'b0,1'b0}),
        .O({\NLW_timeout_cnt0_inferred__0/i__carry__4_O_UNCONNECTED [3],\timeout_cnt0_inferred__0/i__carry__4_n_5 ,\timeout_cnt0_inferred__0/i__carry__4_n_6 ,\timeout_cnt0_inferred__0/i__carry__4_n_7 }),
        .S({1'b0,timeout_cnt[23:21]}));
  LUT6 #(
    .INIT(64'hFFFFFFFEFFFEFFFE)) 
    \timeout_cnt[23]_i_1 
       (.I0(i2c_cmd_type),
        .I1(\FSM_onehot_mpu_state_reg_n_0_[1] ),
        .I2(\FSM_onehot_mpu_state_reg_n_0_[4] ),
        .I3(\FSM_onehot_mpu_state_reg_n_0_[0] ),
        .I4(\timeout_cnt[23]_i_3_n_0 ),
        .I5(\m_axis_tdata[111]_i_3_n_0 ),
        .O(\timeout_cnt[23]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair37" *) 
  LUT3 #(
    .INIT(8'hFE)) 
    \timeout_cnt[23]_i_3 
       (.I0(\FSM_onehot_mpu_state_reg_n_0_[8] ),
        .I1(\FSM_onehot_mpu_state_reg_n_0_[5] ),
        .I2(\FSM_onehot_mpu_state_reg_n_0_[2] ),
        .O(\timeout_cnt[23]_i_3_n_0 ));
  FDRE #(
    .INIT(1'b0)) 
    \timeout_cnt_reg[0] 
       (.C(clk),
        .CE(\timeout_cnt[23]_i_1_n_0 ),
        .D(u_i2c_master_n_65),
        .Q(timeout_cnt[0]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \timeout_cnt_reg[10] 
       (.C(clk),
        .CE(\timeout_cnt[23]_i_1_n_0 ),
        .D(u_i2c_master_n_55),
        .Q(timeout_cnt[10]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \timeout_cnt_reg[11] 
       (.C(clk),
        .CE(\timeout_cnt[23]_i_1_n_0 ),
        .D(u_i2c_master_n_54),
        .Q(timeout_cnt[11]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \timeout_cnt_reg[12] 
       (.C(clk),
        .CE(\timeout_cnt[23]_i_1_n_0 ),
        .D(u_i2c_master_n_53),
        .Q(timeout_cnt[12]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \timeout_cnt_reg[13] 
       (.C(clk),
        .CE(\timeout_cnt[23]_i_1_n_0 ),
        .D(u_i2c_master_n_52),
        .Q(timeout_cnt[13]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \timeout_cnt_reg[14] 
       (.C(clk),
        .CE(\timeout_cnt[23]_i_1_n_0 ),
        .D(u_i2c_master_n_51),
        .Q(timeout_cnt[14]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \timeout_cnt_reg[15] 
       (.C(clk),
        .CE(\timeout_cnt[23]_i_1_n_0 ),
        .D(u_i2c_master_n_50),
        .Q(timeout_cnt[15]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \timeout_cnt_reg[16] 
       (.C(clk),
        .CE(\timeout_cnt[23]_i_1_n_0 ),
        .D(u_i2c_master_n_49),
        .Q(timeout_cnt[16]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \timeout_cnt_reg[17] 
       (.C(clk),
        .CE(\timeout_cnt[23]_i_1_n_0 ),
        .D(u_i2c_master_n_48),
        .Q(timeout_cnt[17]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \timeout_cnt_reg[18] 
       (.C(clk),
        .CE(\timeout_cnt[23]_i_1_n_0 ),
        .D(u_i2c_master_n_47),
        .Q(timeout_cnt[18]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \timeout_cnt_reg[19] 
       (.C(clk),
        .CE(\timeout_cnt[23]_i_1_n_0 ),
        .D(u_i2c_master_n_46),
        .Q(timeout_cnt[19]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \timeout_cnt_reg[1] 
       (.C(clk),
        .CE(\timeout_cnt[23]_i_1_n_0 ),
        .D(u_i2c_master_n_64),
        .Q(timeout_cnt[1]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \timeout_cnt_reg[20] 
       (.C(clk),
        .CE(\timeout_cnt[23]_i_1_n_0 ),
        .D(u_i2c_master_n_45),
        .Q(timeout_cnt[20]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \timeout_cnt_reg[21] 
       (.C(clk),
        .CE(\timeout_cnt[23]_i_1_n_0 ),
        .D(u_i2c_master_n_44),
        .Q(timeout_cnt[21]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \timeout_cnt_reg[22] 
       (.C(clk),
        .CE(\timeout_cnt[23]_i_1_n_0 ),
        .D(u_i2c_master_n_43),
        .Q(timeout_cnt[22]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \timeout_cnt_reg[23] 
       (.C(clk),
        .CE(\timeout_cnt[23]_i_1_n_0 ),
        .D(u_i2c_master_n_42),
        .Q(timeout_cnt[23]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \timeout_cnt_reg[2] 
       (.C(clk),
        .CE(\timeout_cnt[23]_i_1_n_0 ),
        .D(u_i2c_master_n_63),
        .Q(timeout_cnt[2]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \timeout_cnt_reg[3] 
       (.C(clk),
        .CE(\timeout_cnt[23]_i_1_n_0 ),
        .D(u_i2c_master_n_62),
        .Q(timeout_cnt[3]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \timeout_cnt_reg[4] 
       (.C(clk),
        .CE(\timeout_cnt[23]_i_1_n_0 ),
        .D(u_i2c_master_n_61),
        .Q(timeout_cnt[4]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \timeout_cnt_reg[5] 
       (.C(clk),
        .CE(\timeout_cnt[23]_i_1_n_0 ),
        .D(u_i2c_master_n_60),
        .Q(timeout_cnt[5]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \timeout_cnt_reg[6] 
       (.C(clk),
        .CE(\timeout_cnt[23]_i_1_n_0 ),
        .D(u_i2c_master_n_59),
        .Q(timeout_cnt[6]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \timeout_cnt_reg[7] 
       (.C(clk),
        .CE(\timeout_cnt[23]_i_1_n_0 ),
        .D(u_i2c_master_n_58),
        .Q(timeout_cnt[7]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \timeout_cnt_reg[8] 
       (.C(clk),
        .CE(\timeout_cnt[23]_i_1_n_0 ),
        .D(u_i2c_master_n_57),
        .Q(timeout_cnt[8]),
        .R(i2c_rst));
  FDRE #(
    .INIT(1'b0)) 
    \timeout_cnt_reg[9] 
       (.C(clk),
        .CE(\timeout_cnt[23]_i_1_n_0 ),
        .D(u_i2c_master_n_56),
        .Q(timeout_cnt[9]),
        .R(i2c_rst));
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_i2c_master u_i2c_master
       (.D(\FSM_onehot_mpu_state[0]_i_1_n_0 ),
        .E(u_i2c_master_n_24),
        .\FSM_onehot_mpu_state_reg[0] (m_valid_reg_i_2_n_0),
        .\FSM_onehot_mpu_state_reg[0]_0 (\delay_cnt_reg_n_0_[0] ),
        .\FSM_onehot_mpu_state_reg[0]_1 (\FSM_onehot_mpu_state[9]_i_3_n_0 ),
        .\FSM_onehot_mpu_state_reg[0]_2 (\timeout_cnt[23]_i_3_n_0 ),
        .\FSM_onehot_mpu_state_reg[5] (u_i2c_master_n_14),
        .\FSM_onehot_mpu_state_reg[5]_0 (u_i2c_master_n_15),
        .\FSM_onehot_mpu_state_reg[5]_1 (u_i2c_master_n_16),
        .\FSM_onehot_mpu_state_reg[6] (u_i2c_master_n_25),
        .\FSM_sequential_state_reg[1]_0 (\FSM_sequential_state_reg[1] ),
        .\FSM_sequential_state_reg[2]_0 (\FSM_sequential_state_reg[2] ),
        .O({\timeout_cnt0_inferred__0/i__carry_n_4 ,\timeout_cnt0_inferred__0/i__carry_n_5 ,\timeout_cnt0_inferred__0/i__carry_n_6 ,\timeout_cnt0_inferred__0/i__carry_n_7 }),
        .Q(Q),
        .SR(i2c_rst),
        .\bit_idx_reg[0]_0 (\bit_idx_reg[0] ),
        .\bit_idx_reg[1]_0 (\bit_idx_reg[1] ),
        .\bit_idx_reg[2]_0 (\bit_idx_reg[2] ),
        .\burst_len_reg_reg[3]_0 (\i2c_burst_len_reg_n_0_[3] ),
        .\byte_idx_reg[0] (\raw_bytes[13]_6 ),
        .\byte_idx_reg[0]_0 (\m_axis_tdata[111]_i_3_n_0 ),
        .\byte_idx_reg[0]_1 (\byte_idx[3]_i_4_n_0 ),
        .\byte_idx_reg[2] (\raw_bytes[12]_7 ),
        .\byte_idx_reg[2]_0 (\raw_bytes[11]_8 ),
        .\byte_idx_reg[2]_1 (\raw_bytes[10]_9 ),
        .\byte_idx_reg[2]_2 (\raw_bytes[9]_10 ),
        .\byte_idx_reg[2]_3 (\raw_bytes[4]_15 ),
        .\byte_idx_reg[2]_4 (\raw_bytes[3]_16 ),
        .\byte_idx_reg[2]_5 (\raw_bytes[1]_18 ),
        .\byte_idx_reg[2]_6 (\raw_bytes[0]_19 ),
        .\byte_idx_reg[3] (\raw_bytes[8]_11 ),
        .\byte_idx_reg[3]_0 (\raw_bytes[7]_12 ),
        .\byte_idx_reg[3]_1 (\raw_bytes[6]_13 ),
        .\byte_idx_reg[3]_2 (\raw_bytes[5]_14 ),
        .\byte_idx_reg[3]_3 (\raw_bytes[2]_17 ),
        .byte_valid_reg_0(byte_valid_reg),
        .clk(clk),
        .cmd_ready_reg_reg_0(cmd_ready_reg_reg),
        .cmd_ready_reg_reg_1(u_i2c_master_n_66),
        .\cmd_type_reg_reg[1]_0 (\i2c_cmd_type_reg_n_0_[1] ),
        .debug_i2c_busy(debug_i2c_busy),
        .debug_i2c_error(debug_i2c_error),
        .debug_i2c_state(debug_i2c_state),
        .done_reg_0(u_i2c_master_n_13),
        .done_reg_1(u_i2c_master_n_40),
        .done_reg_2({u_i2c_master_n_42,u_i2c_master_n_43,u_i2c_master_n_44,u_i2c_master_n_45,u_i2c_master_n_46,u_i2c_master_n_47,u_i2c_master_n_48,u_i2c_master_n_49,u_i2c_master_n_50,u_i2c_master_n_51,u_i2c_master_n_52,u_i2c_master_n_53,u_i2c_master_n_54,u_i2c_master_n_55,u_i2c_master_n_56,u_i2c_master_n_57,u_i2c_master_n_58,u_i2c_master_n_59,u_i2c_master_n_60,u_i2c_master_n_61,u_i2c_master_n_62,u_i2c_master_n_63,u_i2c_master_n_64,u_i2c_master_n_65}),
        .i2c_cmd_valid_r(i2c_cmd_valid_r),
        .i2c_done(i2c_done),
        .i2c_tick_reg_0(debug_i2c_tick),
        .\init_idx_reg[0] (\init_idx[2]_i_3_n_0 ),
        .\init_idx_reg[2] (\init_idx_reg_n_0_[0] ),
        .\init_idx_reg[2]_0 (\init_idx_reg_n_0_[1] ),
        .\init_idx_reg[2]_1 (\init_idx_reg_n_0_[2] ),
        .\m_axis_tkeep_reg[0] ({\FSM_onehot_mpu_state_reg_n_0_[8] ,i2c_cmd_type,\FSM_onehot_mpu_state_reg_n_0_[6] ,\FSM_onehot_mpu_state_reg_n_0_[5] ,\FSM_onehot_mpu_state_reg_n_0_[4] ,\FSM_onehot_mpu_state_reg_n_0_[3] ,\FSM_onehot_mpu_state_reg_n_0_[2] ,\FSM_onehot_mpu_state_reg_n_0_[1] ,\FSM_onehot_mpu_state_reg_n_0_[0] }),
        .m_axis_tvalid(m_axis_tvalid),
        .mpu_int_sync(mpu_int_sync),
        .\phase_state_reg[0]_0 (\phase_state_reg[0] ),
        .\phase_state_reg[1]_0 (\phase_state_reg[1] ),
        .\phase_state_reg[2]_0 (\phase_state_reg[2] ),
        .\q_cnt_reg[0]_0 (\q_cnt_reg[0] ),
        .\q_cnt_reg[1]_0 (\q_cnt_reg[1] ),
        .\raw_bytes_reg[0][0] (\byte_idx_reg[3]_0 ),
        .\rd_data_reg[7]_0 (\rd_data_reg[7] ),
        .\reg_addr_reg_reg[6]_0 ({\i2c_reg_addr_reg_n_0_[6] ,\i2c_reg_addr_reg_n_0_[5] ,\i2c_reg_addr_reg_n_0_[4] ,\i2c_reg_addr_reg_n_0_[3] ,\i2c_reg_addr_reg_n_0_[2] ,\i2c_reg_addr_reg_n_0_[1] ,\i2c_reg_addr_reg_n_0_[0] }),
        .reset_n(reset_n),
        .scl_t(scl_t),
        .sda_i(sda_i),
        .sda_t(sda_t),
        .\timeout_cnt_reg[0] (timeout_cnt[0]),
        .\timeout_cnt_reg[12] ({\timeout_cnt0_inferred__0/i__carry__1_n_4 ,\timeout_cnt0_inferred__0/i__carry__1_n_5 ,\timeout_cnt0_inferred__0/i__carry__1_n_6 ,\timeout_cnt0_inferred__0/i__carry__1_n_7 }),
        .\timeout_cnt_reg[16] ({\timeout_cnt0_inferred__0/i__carry__2_n_4 ,\timeout_cnt0_inferred__0/i__carry__2_n_5 ,\timeout_cnt0_inferred__0/i__carry__2_n_6 ,\timeout_cnt0_inferred__0/i__carry__2_n_7 }),
        .\timeout_cnt_reg[20] ({\timeout_cnt0_inferred__0/i__carry__3_n_4 ,\timeout_cnt0_inferred__0/i__carry__3_n_5 ,\timeout_cnt0_inferred__0/i__carry__3_n_6 ,\timeout_cnt0_inferred__0/i__carry__3_n_7 }),
        .\timeout_cnt_reg[23] ({\timeout_cnt0_inferred__0/i__carry__4_n_5 ,\timeout_cnt0_inferred__0/i__carry__4_n_6 ,\timeout_cnt0_inferred__0/i__carry__4_n_7 }),
        .\timeout_cnt_reg[8] ({\timeout_cnt0_inferred__0/i__carry__0_n_4 ,\timeout_cnt0_inferred__0/i__carry__0_n_5 ,\timeout_cnt0_inferred__0/i__carry__0_n_6 ,\timeout_cnt0_inferred__0/i__carry__0_n_7 }),
        .\wr_data_reg_reg[0]_0 (i2c_cmd_valid_r_reg_0),
        .\wr_data_reg_reg[7]_0 ({\i2c_wr_data_reg_n_0_[7] ,\i2c_wr_data_reg_n_0_[5] ,\i2c_wr_data_reg_n_0_[4] ,\i2c_wr_data_reg_n_0_[3] ,\i2c_wr_data_reg_n_0_[2] ,\i2c_wr_data_reg_n_0_[1] ,\i2c_wr_data_reg_n_0_[0] }));
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
