// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2025 Advanced Micro Devices, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2025.2 (win64) Build 6299465 Fri Nov 14 19:35:11 GMT 2025
// Date        : Sun Jun 21 18:08:49 2026
// Host        : Michel-PC running 64-bit major release  (build 9200)
// Command     : write_verilog -force -mode funcsim -rename_top decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix -prefix
//               decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_ drone_block_design_axi_mem_intercon_imp_auto_pc_0_sim_netlist.v
// Design      : drone_block_design_axi_mem_intercon_imp_auto_pc_0
// Purpose     : This verilog netlist is a functional simulation representation of the design and should not be modified
//               or synthesized. This netlist cannot be used for SDF annotated simulation.
// Device      : xc7z020clg400-1
// --------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_axi_data_fifo_v2_1_36_axic_fifo
   (dout,
    empty,
    SR,
    aresetn_0,
    m_axi_awvalid,
    length_counter_1_reg_1_sp_1,
    empty_fwft_i_reg,
    m_axi_wvalid,
    S_AXI_AREADY_I_reg,
    \areset_d_reg[1] ,
    aclk,
    m_axi_awlen,
    rd_en,
    aresetn,
    m_axi_awvalid_0,
    command_ongoing,
    m_axi_awready,
    length_counter_1_reg,
    first_mi_word,
    s_axi_wvalid,
    m_axi_wready,
    E,
    s_axi_awvalid,
    Q);
  output [3:0]dout;
  output empty;
  output [0:0]SR;
  output aresetn_0;
  output m_axi_awvalid;
  output length_counter_1_reg_1_sp_1;
  output empty_fwft_i_reg;
  output m_axi_wvalid;
  output S_AXI_AREADY_I_reg;
  output \areset_d_reg[1] ;
  input aclk;
  input [3:0]m_axi_awlen;
  input rd_en;
  input aresetn;
  input m_axi_awvalid_0;
  input command_ongoing;
  input m_axi_awready;
  input [1:0]length_counter_1_reg;
  input first_mi_word;
  input s_axi_wvalid;
  input m_axi_wready;
  input [0:0]E;
  input s_axi_awvalid;
  input [1:0]Q;

  wire [0:0]E;
  wire [1:0]Q;
  wire [0:0]SR;
  wire S_AXI_AREADY_I_reg;
  wire aclk;
  wire \areset_d_reg[1] ;
  wire aresetn;
  wire aresetn_0;
  wire command_ongoing;
  wire [3:0]dout;
  wire empty;
  wire empty_fwft_i_reg;
  wire first_mi_word;
  wire [1:0]length_counter_1_reg;
  wire length_counter_1_reg_1_sn_1;
  wire [3:0]m_axi_awlen;
  wire m_axi_awready;
  wire m_axi_awvalid;
  wire m_axi_awvalid_0;
  wire m_axi_wready;
  wire m_axi_wvalid;
  wire rd_en;
  wire s_axi_awvalid;
  wire s_axi_wvalid;

  assign length_counter_1_reg_1_sp_1 = length_counter_1_reg_1_sn_1;
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_axi_data_fifo_v2_1_36_fifo_gen inst
       (.E(E),
        .Q(Q),
        .SR(SR),
        .S_AXI_AREADY_I_reg(S_AXI_AREADY_I_reg),
        .aclk(aclk),
        .\areset_d_reg[1] (\areset_d_reg[1] ),
        .aresetn(aresetn),
        .aresetn_0(aresetn_0),
        .command_ongoing(command_ongoing),
        .dout(dout),
        .empty(empty),
        .empty_fwft_i_reg(empty_fwft_i_reg),
        .first_mi_word(first_mi_word),
        .length_counter_1_reg(length_counter_1_reg),
        .length_counter_1_reg_1_sp_1(length_counter_1_reg_1_sn_1),
        .m_axi_awlen(m_axi_awlen),
        .m_axi_awready(m_axi_awready),
        .m_axi_awvalid(m_axi_awvalid),
        .m_axi_awvalid_0(m_axi_awvalid_0),
        .m_axi_wready(m_axi_wready),
        .m_axi_wvalid(m_axi_wvalid),
        .rd_en(rd_en),
        .s_axi_awvalid(s_axi_awvalid),
        .s_axi_wvalid(s_axi_wvalid));
endmodule

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_axi_data_fifo_v2_1_36_fifo_gen
   (dout,
    empty,
    SR,
    aresetn_0,
    m_axi_awvalid,
    length_counter_1_reg_1_sp_1,
    empty_fwft_i_reg,
    m_axi_wvalid,
    S_AXI_AREADY_I_reg,
    \areset_d_reg[1] ,
    aclk,
    m_axi_awlen,
    rd_en,
    aresetn,
    m_axi_awvalid_0,
    command_ongoing,
    m_axi_awready,
    length_counter_1_reg,
    first_mi_word,
    s_axi_wvalid,
    m_axi_wready,
    E,
    s_axi_awvalid,
    Q);
  output [3:0]dout;
  output empty;
  output [0:0]SR;
  output aresetn_0;
  output m_axi_awvalid;
  output length_counter_1_reg_1_sp_1;
  output empty_fwft_i_reg;
  output m_axi_wvalid;
  output S_AXI_AREADY_I_reg;
  output \areset_d_reg[1] ;
  input aclk;
  input [3:0]m_axi_awlen;
  input rd_en;
  input aresetn;
  input m_axi_awvalid_0;
  input command_ongoing;
  input m_axi_awready;
  input [1:0]length_counter_1_reg;
  input first_mi_word;
  input s_axi_wvalid;
  input m_axi_wready;
  input [0:0]E;
  input s_axi_awvalid;
  input [1:0]Q;

  wire [0:0]E;
  wire [1:0]Q;
  wire [0:0]SR;
  wire S_AXI_AREADY_I_i_3_n_0;
  wire S_AXI_AREADY_I_reg;
  wire aclk;
  wire \areset_d_reg[1] ;
  wire aresetn;
  wire aresetn_0;
  wire cmd_push;
  wire command_ongoing;
  wire command_ongoing_i_2_n_0;
  wire [3:0]dout;
  wire empty;
  wire empty_fwft_i_reg;
  wire first_mi_word;
  wire full;
  wire [1:0]length_counter_1_reg;
  wire length_counter_1_reg_1_sn_1;
  wire [3:0]m_axi_awlen;
  wire m_axi_awready;
  wire m_axi_awvalid;
  wire m_axi_awvalid_0;
  wire m_axi_wready;
  wire m_axi_wvalid;
  wire rd_en;
  wire s_axi_awvalid;
  wire s_axi_wvalid;
  wire NLW_fifo_gen_inst_almost_empty_UNCONNECTED;
  wire NLW_fifo_gen_inst_almost_full_UNCONNECTED;
  wire NLW_fifo_gen_inst_axi_ar_dbiterr_UNCONNECTED;
  wire NLW_fifo_gen_inst_axi_ar_overflow_UNCONNECTED;
  wire NLW_fifo_gen_inst_axi_ar_prog_empty_UNCONNECTED;
  wire NLW_fifo_gen_inst_axi_ar_prog_full_UNCONNECTED;
  wire NLW_fifo_gen_inst_axi_ar_sbiterr_UNCONNECTED;
  wire NLW_fifo_gen_inst_axi_ar_underflow_UNCONNECTED;
  wire NLW_fifo_gen_inst_axi_aw_dbiterr_UNCONNECTED;
  wire NLW_fifo_gen_inst_axi_aw_overflow_UNCONNECTED;
  wire NLW_fifo_gen_inst_axi_aw_prog_empty_UNCONNECTED;
  wire NLW_fifo_gen_inst_axi_aw_prog_full_UNCONNECTED;
  wire NLW_fifo_gen_inst_axi_aw_sbiterr_UNCONNECTED;
  wire NLW_fifo_gen_inst_axi_aw_underflow_UNCONNECTED;
  wire NLW_fifo_gen_inst_axi_b_dbiterr_UNCONNECTED;
  wire NLW_fifo_gen_inst_axi_b_overflow_UNCONNECTED;
  wire NLW_fifo_gen_inst_axi_b_prog_empty_UNCONNECTED;
  wire NLW_fifo_gen_inst_axi_b_prog_full_UNCONNECTED;
  wire NLW_fifo_gen_inst_axi_b_sbiterr_UNCONNECTED;
  wire NLW_fifo_gen_inst_axi_b_underflow_UNCONNECTED;
  wire NLW_fifo_gen_inst_axi_r_dbiterr_UNCONNECTED;
  wire NLW_fifo_gen_inst_axi_r_overflow_UNCONNECTED;
  wire NLW_fifo_gen_inst_axi_r_prog_empty_UNCONNECTED;
  wire NLW_fifo_gen_inst_axi_r_prog_full_UNCONNECTED;
  wire NLW_fifo_gen_inst_axi_r_sbiterr_UNCONNECTED;
  wire NLW_fifo_gen_inst_axi_r_underflow_UNCONNECTED;
  wire NLW_fifo_gen_inst_axi_w_dbiterr_UNCONNECTED;
  wire NLW_fifo_gen_inst_axi_w_overflow_UNCONNECTED;
  wire NLW_fifo_gen_inst_axi_w_prog_empty_UNCONNECTED;
  wire NLW_fifo_gen_inst_axi_w_prog_full_UNCONNECTED;
  wire NLW_fifo_gen_inst_axi_w_sbiterr_UNCONNECTED;
  wire NLW_fifo_gen_inst_axi_w_underflow_UNCONNECTED;
  wire NLW_fifo_gen_inst_axis_dbiterr_UNCONNECTED;
  wire NLW_fifo_gen_inst_axis_overflow_UNCONNECTED;
  wire NLW_fifo_gen_inst_axis_prog_empty_UNCONNECTED;
  wire NLW_fifo_gen_inst_axis_prog_full_UNCONNECTED;
  wire NLW_fifo_gen_inst_axis_sbiterr_UNCONNECTED;
  wire NLW_fifo_gen_inst_axis_underflow_UNCONNECTED;
  wire NLW_fifo_gen_inst_dbiterr_UNCONNECTED;
  wire NLW_fifo_gen_inst_m_axi_arvalid_UNCONNECTED;
  wire NLW_fifo_gen_inst_m_axi_awvalid_UNCONNECTED;
  wire NLW_fifo_gen_inst_m_axi_bready_UNCONNECTED;
  wire NLW_fifo_gen_inst_m_axi_rready_UNCONNECTED;
  wire NLW_fifo_gen_inst_m_axi_wlast_UNCONNECTED;
  wire NLW_fifo_gen_inst_m_axi_wvalid_UNCONNECTED;
  wire NLW_fifo_gen_inst_m_axis_tlast_UNCONNECTED;
  wire NLW_fifo_gen_inst_m_axis_tvalid_UNCONNECTED;
  wire NLW_fifo_gen_inst_overflow_UNCONNECTED;
  wire NLW_fifo_gen_inst_prog_empty_UNCONNECTED;
  wire NLW_fifo_gen_inst_prog_full_UNCONNECTED;
  wire NLW_fifo_gen_inst_rd_rst_busy_UNCONNECTED;
  wire NLW_fifo_gen_inst_s_axi_arready_UNCONNECTED;
  wire NLW_fifo_gen_inst_s_axi_awready_UNCONNECTED;
  wire NLW_fifo_gen_inst_s_axi_bvalid_UNCONNECTED;
  wire NLW_fifo_gen_inst_s_axi_rlast_UNCONNECTED;
  wire NLW_fifo_gen_inst_s_axi_rvalid_UNCONNECTED;
  wire NLW_fifo_gen_inst_s_axi_wready_UNCONNECTED;
  wire NLW_fifo_gen_inst_s_axis_tready_UNCONNECTED;
  wire NLW_fifo_gen_inst_sbiterr_UNCONNECTED;
  wire NLW_fifo_gen_inst_underflow_UNCONNECTED;
  wire NLW_fifo_gen_inst_valid_UNCONNECTED;
  wire NLW_fifo_gen_inst_wr_ack_UNCONNECTED;
  wire NLW_fifo_gen_inst_wr_rst_busy_UNCONNECTED;
  wire [4:0]NLW_fifo_gen_inst_axi_ar_data_count_UNCONNECTED;
  wire [4:0]NLW_fifo_gen_inst_axi_ar_rd_data_count_UNCONNECTED;
  wire [4:0]NLW_fifo_gen_inst_axi_ar_wr_data_count_UNCONNECTED;
  wire [4:0]NLW_fifo_gen_inst_axi_aw_data_count_UNCONNECTED;
  wire [4:0]NLW_fifo_gen_inst_axi_aw_rd_data_count_UNCONNECTED;
  wire [4:0]NLW_fifo_gen_inst_axi_aw_wr_data_count_UNCONNECTED;
  wire [4:0]NLW_fifo_gen_inst_axi_b_data_count_UNCONNECTED;
  wire [4:0]NLW_fifo_gen_inst_axi_b_rd_data_count_UNCONNECTED;
  wire [4:0]NLW_fifo_gen_inst_axi_b_wr_data_count_UNCONNECTED;
  wire [10:0]NLW_fifo_gen_inst_axi_r_data_count_UNCONNECTED;
  wire [10:0]NLW_fifo_gen_inst_axi_r_rd_data_count_UNCONNECTED;
  wire [10:0]NLW_fifo_gen_inst_axi_r_wr_data_count_UNCONNECTED;
  wire [10:0]NLW_fifo_gen_inst_axi_w_data_count_UNCONNECTED;
  wire [10:0]NLW_fifo_gen_inst_axi_w_rd_data_count_UNCONNECTED;
  wire [10:0]NLW_fifo_gen_inst_axi_w_wr_data_count_UNCONNECTED;
  wire [10:0]NLW_fifo_gen_inst_axis_data_count_UNCONNECTED;
  wire [10:0]NLW_fifo_gen_inst_axis_rd_data_count_UNCONNECTED;
  wire [10:0]NLW_fifo_gen_inst_axis_wr_data_count_UNCONNECTED;
  wire [5:0]NLW_fifo_gen_inst_data_count_UNCONNECTED;
  wire [4:4]NLW_fifo_gen_inst_dout_UNCONNECTED;
  wire [31:0]NLW_fifo_gen_inst_m_axi_araddr_UNCONNECTED;
  wire [1:0]NLW_fifo_gen_inst_m_axi_arburst_UNCONNECTED;
  wire [3:0]NLW_fifo_gen_inst_m_axi_arcache_UNCONNECTED;
  wire [3:0]NLW_fifo_gen_inst_m_axi_arid_UNCONNECTED;
  wire [7:0]NLW_fifo_gen_inst_m_axi_arlen_UNCONNECTED;
  wire [1:0]NLW_fifo_gen_inst_m_axi_arlock_UNCONNECTED;
  wire [2:0]NLW_fifo_gen_inst_m_axi_arprot_UNCONNECTED;
  wire [3:0]NLW_fifo_gen_inst_m_axi_arqos_UNCONNECTED;
  wire [3:0]NLW_fifo_gen_inst_m_axi_arregion_UNCONNECTED;
  wire [2:0]NLW_fifo_gen_inst_m_axi_arsize_UNCONNECTED;
  wire [0:0]NLW_fifo_gen_inst_m_axi_aruser_UNCONNECTED;
  wire [31:0]NLW_fifo_gen_inst_m_axi_awaddr_UNCONNECTED;
  wire [1:0]NLW_fifo_gen_inst_m_axi_awburst_UNCONNECTED;
  wire [3:0]NLW_fifo_gen_inst_m_axi_awcache_UNCONNECTED;
  wire [3:0]NLW_fifo_gen_inst_m_axi_awid_UNCONNECTED;
  wire [7:0]NLW_fifo_gen_inst_m_axi_awlen_UNCONNECTED;
  wire [1:0]NLW_fifo_gen_inst_m_axi_awlock_UNCONNECTED;
  wire [2:0]NLW_fifo_gen_inst_m_axi_awprot_UNCONNECTED;
  wire [3:0]NLW_fifo_gen_inst_m_axi_awqos_UNCONNECTED;
  wire [3:0]NLW_fifo_gen_inst_m_axi_awregion_UNCONNECTED;
  wire [2:0]NLW_fifo_gen_inst_m_axi_awsize_UNCONNECTED;
  wire [0:0]NLW_fifo_gen_inst_m_axi_awuser_UNCONNECTED;
  wire [63:0]NLW_fifo_gen_inst_m_axi_wdata_UNCONNECTED;
  wire [3:0]NLW_fifo_gen_inst_m_axi_wid_UNCONNECTED;
  wire [7:0]NLW_fifo_gen_inst_m_axi_wstrb_UNCONNECTED;
  wire [0:0]NLW_fifo_gen_inst_m_axi_wuser_UNCONNECTED;
  wire [63:0]NLW_fifo_gen_inst_m_axis_tdata_UNCONNECTED;
  wire [3:0]NLW_fifo_gen_inst_m_axis_tdest_UNCONNECTED;
  wire [7:0]NLW_fifo_gen_inst_m_axis_tid_UNCONNECTED;
  wire [3:0]NLW_fifo_gen_inst_m_axis_tkeep_UNCONNECTED;
  wire [3:0]NLW_fifo_gen_inst_m_axis_tstrb_UNCONNECTED;
  wire [3:0]NLW_fifo_gen_inst_m_axis_tuser_UNCONNECTED;
  wire [5:0]NLW_fifo_gen_inst_rd_data_count_UNCONNECTED;
  wire [3:0]NLW_fifo_gen_inst_s_axi_bid_UNCONNECTED;
  wire [1:0]NLW_fifo_gen_inst_s_axi_bresp_UNCONNECTED;
  wire [0:0]NLW_fifo_gen_inst_s_axi_buser_UNCONNECTED;
  wire [63:0]NLW_fifo_gen_inst_s_axi_rdata_UNCONNECTED;
  wire [3:0]NLW_fifo_gen_inst_s_axi_rid_UNCONNECTED;
  wire [1:0]NLW_fifo_gen_inst_s_axi_rresp_UNCONNECTED;
  wire [0:0]NLW_fifo_gen_inst_s_axi_ruser_UNCONNECTED;
  wire [5:0]NLW_fifo_gen_inst_wr_data_count_UNCONNECTED;

  assign length_counter_1_reg_1_sp_1 = length_counter_1_reg_1_sn_1;
  LUT1 #(
    .INIT(2'h1)) 
    S_AXI_AREADY_I_i_1
       (.I0(aresetn),
        .O(SR));
  LUT6 #(
    .INIT(64'h22722272FFFF2272)) 
    S_AXI_AREADY_I_i_2
       (.I0(E),
        .I1(s_axi_awvalid),
        .I2(m_axi_awready),
        .I3(S_AXI_AREADY_I_i_3_n_0),
        .I4(Q[1]),
        .I5(Q[0]),
        .O(S_AXI_AREADY_I_reg));
  (* SOFT_HLUTNM = "soft_lutpair7" *) 
  LUT3 #(
    .INIT(8'h4F)) 
    S_AXI_AREADY_I_i_3
       (.I0(m_axi_awvalid_0),
        .I1(full),
        .I2(command_ongoing),
        .O(S_AXI_AREADY_I_i_3_n_0));
  (* SOFT_HLUTNM = "soft_lutpair5" *) 
  LUT5 #(
    .INIT(32'h00888A88)) 
    cmd_push_block_i_1
       (.I0(aresetn),
        .I1(m_axi_awvalid_0),
        .I2(full),
        .I3(command_ongoing),
        .I4(m_axi_awready),
        .O(aresetn_0));
  LUT6 #(
    .INIT(64'hF222FFFFD000D000)) 
    command_ongoing_i_1
       (.I0(Q[1]),
        .I1(Q[0]),
        .I2(E),
        .I3(s_axi_awvalid),
        .I4(command_ongoing_i_2_n_0),
        .I5(command_ongoing),
        .O(\areset_d_reg[1] ));
  (* SOFT_HLUTNM = "soft_lutpair5" *) 
  LUT4 #(
    .INIT(16'h8808)) 
    command_ongoing_i_2
       (.I0(m_axi_awready),
        .I1(command_ongoing),
        .I2(full),
        .I3(m_axi_awvalid_0),
        .O(command_ongoing_i_2_n_0));
  (* C_ADD_NGC_CONSTRAINT = "0" *) 
  (* C_APPLICATION_TYPE_AXIS = "0" *) 
  (* C_APPLICATION_TYPE_RACH = "0" *) 
  (* C_APPLICATION_TYPE_RDCH = "0" *) 
  (* C_APPLICATION_TYPE_WACH = "0" *) 
  (* C_APPLICATION_TYPE_WDCH = "0" *) 
  (* C_APPLICATION_TYPE_WRCH = "0" *) 
  (* C_AXIS_TDATA_WIDTH = "64" *) 
  (* C_AXIS_TDEST_WIDTH = "4" *) 
  (* C_AXIS_TID_WIDTH = "8" *) 
  (* C_AXIS_TKEEP_WIDTH = "4" *) 
  (* C_AXIS_TSTRB_WIDTH = "4" *) 
  (* C_AXIS_TUSER_WIDTH = "4" *) 
  (* C_AXIS_TYPE = "0" *) 
  (* C_AXI_ADDR_WIDTH = "32" *) 
  (* C_AXI_ARUSER_WIDTH = "1" *) 
  (* C_AXI_AWUSER_WIDTH = "1" *) 
  (* C_AXI_BUSER_WIDTH = "1" *) 
  (* C_AXI_DATA_WIDTH = "64" *) 
  (* C_AXI_ID_WIDTH = "4" *) 
  (* C_AXI_LEN_WIDTH = "8" *) 
  (* C_AXI_LOCK_WIDTH = "2" *) 
  (* C_AXI_RUSER_WIDTH = "1" *) 
  (* C_AXI_TYPE = "0" *) 
  (* C_AXI_WUSER_WIDTH = "1" *) 
  (* C_COMMON_CLOCK = "1" *) 
  (* C_COUNT_TYPE = "0" *) 
  (* C_DATA_COUNT_WIDTH = "6" *) 
  (* C_DEFAULT_VALUE = "BlankString" *) 
  (* C_DIN_WIDTH = "5" *) 
  (* C_DIN_WIDTH_AXIS = "1" *) 
  (* C_DIN_WIDTH_RACH = "32" *) 
  (* C_DIN_WIDTH_RDCH = "64" *) 
  (* C_DIN_WIDTH_WACH = "32" *) 
  (* C_DIN_WIDTH_WDCH = "64" *) 
  (* C_DIN_WIDTH_WRCH = "2" *) 
  (* C_DOUT_RST_VAL = "0" *) 
  (* C_DOUT_WIDTH = "5" *) 
  (* C_ENABLE_RLOCS = "0" *) 
  (* C_ENABLE_RST_SYNC = "1" *) 
  (* C_EN_SAFETY_CKT = "0" *) 
  (* C_ERROR_INJECTION_TYPE = "0" *) 
  (* C_ERROR_INJECTION_TYPE_AXIS = "0" *) 
  (* C_ERROR_INJECTION_TYPE_RACH = "0" *) 
  (* C_ERROR_INJECTION_TYPE_RDCH = "0" *) 
  (* C_ERROR_INJECTION_TYPE_WACH = "0" *) 
  (* C_ERROR_INJECTION_TYPE_WDCH = "0" *) 
  (* C_ERROR_INJECTION_TYPE_WRCH = "0" *) 
  (* C_FAMILY = "zynq" *) 
  (* C_FULL_FLAGS_RST_VAL = "0" *) 
  (* C_HAS_ALMOST_EMPTY = "0" *) 
  (* C_HAS_ALMOST_FULL = "0" *) 
  (* C_HAS_AXIS_TDATA = "0" *) 
  (* C_HAS_AXIS_TDEST = "0" *) 
  (* C_HAS_AXIS_TID = "0" *) 
  (* C_HAS_AXIS_TKEEP = "0" *) 
  (* C_HAS_AXIS_TLAST = "0" *) 
  (* C_HAS_AXIS_TREADY = "1" *) 
  (* C_HAS_AXIS_TSTRB = "0" *) 
  (* C_HAS_AXIS_TUSER = "0" *) 
  (* C_HAS_AXI_ARUSER = "0" *) 
  (* C_HAS_AXI_AWUSER = "0" *) 
  (* C_HAS_AXI_BUSER = "0" *) 
  (* C_HAS_AXI_ID = "0" *) 
  (* C_HAS_AXI_RD_CHANNEL = "0" *) 
  (* C_HAS_AXI_RUSER = "0" *) 
  (* C_HAS_AXI_WR_CHANNEL = "0" *) 
  (* C_HAS_AXI_WUSER = "0" *) 
  (* C_HAS_BACKUP = "0" *) 
  (* C_HAS_DATA_COUNT = "0" *) 
  (* C_HAS_DATA_COUNTS_AXIS = "0" *) 
  (* C_HAS_DATA_COUNTS_RACH = "0" *) 
  (* C_HAS_DATA_COUNTS_RDCH = "0" *) 
  (* C_HAS_DATA_COUNTS_WACH = "0" *) 
  (* C_HAS_DATA_COUNTS_WDCH = "0" *) 
  (* C_HAS_DATA_COUNTS_WRCH = "0" *) 
  (* C_HAS_INT_CLK = "0" *) 
  (* C_HAS_MASTER_CE = "0" *) 
  (* C_HAS_MEMINIT_FILE = "0" *) 
  (* C_HAS_OVERFLOW = "0" *) 
  (* C_HAS_PROG_FLAGS_AXIS = "0" *) 
  (* C_HAS_PROG_FLAGS_RACH = "0" *) 
  (* C_HAS_PROG_FLAGS_RDCH = "0" *) 
  (* C_HAS_PROG_FLAGS_WACH = "0" *) 
  (* C_HAS_PROG_FLAGS_WDCH = "0" *) 
  (* C_HAS_PROG_FLAGS_WRCH = "0" *) 
  (* C_HAS_RD_DATA_COUNT = "0" *) 
  (* C_HAS_RD_RST = "0" *) 
  (* C_HAS_RST = "1" *) 
  (* C_HAS_SLAVE_CE = "0" *) 
  (* C_HAS_SRST = "0" *) 
  (* C_HAS_UNDERFLOW = "0" *) 
  (* C_HAS_VALID = "0" *) 
  (* C_HAS_WR_ACK = "0" *) 
  (* C_HAS_WR_DATA_COUNT = "0" *) 
  (* C_HAS_WR_RST = "0" *) 
  (* C_IMPLEMENTATION_TYPE = "0" *) 
  (* C_IMPLEMENTATION_TYPE_AXIS = "1" *) 
  (* C_IMPLEMENTATION_TYPE_RACH = "1" *) 
  (* C_IMPLEMENTATION_TYPE_RDCH = "1" *) 
  (* C_IMPLEMENTATION_TYPE_WACH = "1" *) 
  (* C_IMPLEMENTATION_TYPE_WDCH = "1" *) 
  (* C_IMPLEMENTATION_TYPE_WRCH = "1" *) 
  (* C_INIT_WR_PNTR_VAL = "0" *) 
  (* C_INTERFACE_TYPE = "0" *) 
  (* C_MEMORY_TYPE = "2" *) 
  (* C_MIF_FILE_NAME = "BlankString" *) 
  (* C_MSGON_VAL = "1" *) 
  (* C_OPTIMIZATION_MODE = "0" *) 
  (* C_OVERFLOW_LOW = "0" *) 
  (* C_POWER_SAVING_MODE = "0" *) 
  (* C_PRELOAD_LATENCY = "0" *) 
  (* C_PRELOAD_REGS = "1" *) 
  (* C_PRIM_FIFO_TYPE = "512x36" *) 
  (* C_PRIM_FIFO_TYPE_AXIS = "512x36" *) 
  (* C_PRIM_FIFO_TYPE_RACH = "512x36" *) 
  (* C_PRIM_FIFO_TYPE_RDCH = "512x36" *) 
  (* C_PRIM_FIFO_TYPE_WACH = "512x36" *) 
  (* C_PRIM_FIFO_TYPE_WDCH = "512x36" *) 
  (* C_PRIM_FIFO_TYPE_WRCH = "512x36" *) 
  (* C_PROG_EMPTY_THRESH_ASSERT_VAL = "4" *) 
  (* C_PROG_EMPTY_THRESH_ASSERT_VAL_AXIS = "1022" *) 
  (* C_PROG_EMPTY_THRESH_ASSERT_VAL_RACH = "1022" *) 
  (* C_PROG_EMPTY_THRESH_ASSERT_VAL_RDCH = "1022" *) 
  (* C_PROG_EMPTY_THRESH_ASSERT_VAL_WACH = "1022" *) 
  (* C_PROG_EMPTY_THRESH_ASSERT_VAL_WDCH = "1022" *) 
  (* C_PROG_EMPTY_THRESH_ASSERT_VAL_WRCH = "1022" *) 
  (* C_PROG_EMPTY_THRESH_NEGATE_VAL = "5" *) 
  (* C_PROG_EMPTY_TYPE = "0" *) 
  (* C_PROG_EMPTY_TYPE_AXIS = "0" *) 
  (* C_PROG_EMPTY_TYPE_RACH = "0" *) 
  (* C_PROG_EMPTY_TYPE_RDCH = "0" *) 
  (* C_PROG_EMPTY_TYPE_WACH = "0" *) 
  (* C_PROG_EMPTY_TYPE_WDCH = "0" *) 
  (* C_PROG_EMPTY_TYPE_WRCH = "0" *) 
  (* C_PROG_FULL_THRESH_ASSERT_VAL = "31" *) 
  (* C_PROG_FULL_THRESH_ASSERT_VAL_AXIS = "1023" *) 
  (* C_PROG_FULL_THRESH_ASSERT_VAL_RACH = "1023" *) 
  (* C_PROG_FULL_THRESH_ASSERT_VAL_RDCH = "1023" *) 
  (* C_PROG_FULL_THRESH_ASSERT_VAL_WACH = "1023" *) 
  (* C_PROG_FULL_THRESH_ASSERT_VAL_WDCH = "1023" *) 
  (* C_PROG_FULL_THRESH_ASSERT_VAL_WRCH = "1023" *) 
  (* C_PROG_FULL_THRESH_NEGATE_VAL = "30" *) 
  (* C_PROG_FULL_TYPE = "0" *) 
  (* C_PROG_FULL_TYPE_AXIS = "0" *) 
  (* C_PROG_FULL_TYPE_RACH = "0" *) 
  (* C_PROG_FULL_TYPE_RDCH = "0" *) 
  (* C_PROG_FULL_TYPE_WACH = "0" *) 
  (* C_PROG_FULL_TYPE_WDCH = "0" *) 
  (* C_PROG_FULL_TYPE_WRCH = "0" *) 
  (* C_RACH_TYPE = "0" *) 
  (* C_RDCH_TYPE = "0" *) 
  (* C_RD_DATA_COUNT_WIDTH = "6" *) 
  (* C_RD_DEPTH = "32" *) 
  (* C_RD_FREQ = "1" *) 
  (* C_RD_PNTR_WIDTH = "5" *) 
  (* C_REG_SLICE_MODE_AXIS = "0" *) 
  (* C_REG_SLICE_MODE_RACH = "0" *) 
  (* C_REG_SLICE_MODE_RDCH = "0" *) 
  (* C_REG_SLICE_MODE_WACH = "0" *) 
  (* C_REG_SLICE_MODE_WDCH = "0" *) 
  (* C_REG_SLICE_MODE_WRCH = "0" *) 
  (* C_SELECT_XPM = "0" *) 
  (* C_SYNCHRONIZER_STAGE = "3" *) 
  (* C_UNDERFLOW_LOW = "0" *) 
  (* C_USE_COMMON_OVERFLOW = "0" *) 
  (* C_USE_COMMON_UNDERFLOW = "0" *) 
  (* C_USE_DEFAULT_SETTINGS = "0" *) 
  (* C_USE_DOUT_RST = "0" *) 
  (* C_USE_ECC = "0" *) 
  (* C_USE_ECC_AXIS = "0" *) 
  (* C_USE_ECC_RACH = "0" *) 
  (* C_USE_ECC_RDCH = "0" *) 
  (* C_USE_ECC_WACH = "0" *) 
  (* C_USE_ECC_WDCH = "0" *) 
  (* C_USE_ECC_WRCH = "0" *) 
  (* C_USE_EMBEDDED_REG = "0" *) 
  (* C_USE_FIFO16_FLAGS = "0" *) 
  (* C_USE_FWFT_DATA_COUNT = "1" *) 
  (* C_USE_PIPELINE_REG = "0" *) 
  (* C_VALID_LOW = "0" *) 
  (* C_WACH_TYPE = "0" *) 
  (* C_WDCH_TYPE = "0" *) 
  (* C_WRCH_TYPE = "0" *) 
  (* C_WR_ACK_LOW = "0" *) 
  (* C_WR_DATA_COUNT_WIDTH = "6" *) 
  (* C_WR_DEPTH = "32" *) 
  (* C_WR_DEPTH_AXIS = "1024" *) 
  (* C_WR_DEPTH_RACH = "16" *) 
  (* C_WR_DEPTH_RDCH = "1024" *) 
  (* C_WR_DEPTH_WACH = "16" *) 
  (* C_WR_DEPTH_WDCH = "1024" *) 
  (* C_WR_DEPTH_WRCH = "16" *) 
  (* C_WR_FREQ = "1" *) 
  (* C_WR_PNTR_WIDTH = "5" *) 
  (* C_WR_PNTR_WIDTH_AXIS = "10" *) 
  (* C_WR_PNTR_WIDTH_RACH = "4" *) 
  (* C_WR_PNTR_WIDTH_RDCH = "10" *) 
  (* C_WR_PNTR_WIDTH_WACH = "4" *) 
  (* C_WR_PNTR_WIDTH_WDCH = "10" *) 
  (* C_WR_PNTR_WIDTH_WRCH = "4" *) 
  (* C_WR_RESPONSE_LATENCY = "1" *) 
  (* KEEP_HIERARCHY = "SOFT" *) 
  (* is_du_within_envelope = "true" *) 
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_fifo_generator_v13_2_14 fifo_gen_inst
       (.almost_empty(NLW_fifo_gen_inst_almost_empty_UNCONNECTED),
        .almost_full(NLW_fifo_gen_inst_almost_full_UNCONNECTED),
        .axi_ar_data_count(NLW_fifo_gen_inst_axi_ar_data_count_UNCONNECTED[4:0]),
        .axi_ar_dbiterr(NLW_fifo_gen_inst_axi_ar_dbiterr_UNCONNECTED),
        .axi_ar_injectdbiterr(1'b0),
        .axi_ar_injectsbiterr(1'b0),
        .axi_ar_overflow(NLW_fifo_gen_inst_axi_ar_overflow_UNCONNECTED),
        .axi_ar_prog_empty(NLW_fifo_gen_inst_axi_ar_prog_empty_UNCONNECTED),
        .axi_ar_prog_empty_thresh({1'b0,1'b0,1'b0,1'b0}),
        .axi_ar_prog_full(NLW_fifo_gen_inst_axi_ar_prog_full_UNCONNECTED),
        .axi_ar_prog_full_thresh({1'b0,1'b0,1'b0,1'b0}),
        .axi_ar_rd_data_count(NLW_fifo_gen_inst_axi_ar_rd_data_count_UNCONNECTED[4:0]),
        .axi_ar_sbiterr(NLW_fifo_gen_inst_axi_ar_sbiterr_UNCONNECTED),
        .axi_ar_underflow(NLW_fifo_gen_inst_axi_ar_underflow_UNCONNECTED),
        .axi_ar_wr_data_count(NLW_fifo_gen_inst_axi_ar_wr_data_count_UNCONNECTED[4:0]),
        .axi_aw_data_count(NLW_fifo_gen_inst_axi_aw_data_count_UNCONNECTED[4:0]),
        .axi_aw_dbiterr(NLW_fifo_gen_inst_axi_aw_dbiterr_UNCONNECTED),
        .axi_aw_injectdbiterr(1'b0),
        .axi_aw_injectsbiterr(1'b0),
        .axi_aw_overflow(NLW_fifo_gen_inst_axi_aw_overflow_UNCONNECTED),
        .axi_aw_prog_empty(NLW_fifo_gen_inst_axi_aw_prog_empty_UNCONNECTED),
        .axi_aw_prog_empty_thresh({1'b0,1'b0,1'b0,1'b0}),
        .axi_aw_prog_full(NLW_fifo_gen_inst_axi_aw_prog_full_UNCONNECTED),
        .axi_aw_prog_full_thresh({1'b0,1'b0,1'b0,1'b0}),
        .axi_aw_rd_data_count(NLW_fifo_gen_inst_axi_aw_rd_data_count_UNCONNECTED[4:0]),
        .axi_aw_sbiterr(NLW_fifo_gen_inst_axi_aw_sbiterr_UNCONNECTED),
        .axi_aw_underflow(NLW_fifo_gen_inst_axi_aw_underflow_UNCONNECTED),
        .axi_aw_wr_data_count(NLW_fifo_gen_inst_axi_aw_wr_data_count_UNCONNECTED[4:0]),
        .axi_b_data_count(NLW_fifo_gen_inst_axi_b_data_count_UNCONNECTED[4:0]),
        .axi_b_dbiterr(NLW_fifo_gen_inst_axi_b_dbiterr_UNCONNECTED),
        .axi_b_injectdbiterr(1'b0),
        .axi_b_injectsbiterr(1'b0),
        .axi_b_overflow(NLW_fifo_gen_inst_axi_b_overflow_UNCONNECTED),
        .axi_b_prog_empty(NLW_fifo_gen_inst_axi_b_prog_empty_UNCONNECTED),
        .axi_b_prog_empty_thresh({1'b0,1'b0,1'b0,1'b0}),
        .axi_b_prog_full(NLW_fifo_gen_inst_axi_b_prog_full_UNCONNECTED),
        .axi_b_prog_full_thresh({1'b0,1'b0,1'b0,1'b0}),
        .axi_b_rd_data_count(NLW_fifo_gen_inst_axi_b_rd_data_count_UNCONNECTED[4:0]),
        .axi_b_sbiterr(NLW_fifo_gen_inst_axi_b_sbiterr_UNCONNECTED),
        .axi_b_underflow(NLW_fifo_gen_inst_axi_b_underflow_UNCONNECTED),
        .axi_b_wr_data_count(NLW_fifo_gen_inst_axi_b_wr_data_count_UNCONNECTED[4:0]),
        .axi_r_data_count(NLW_fifo_gen_inst_axi_r_data_count_UNCONNECTED[10:0]),
        .axi_r_dbiterr(NLW_fifo_gen_inst_axi_r_dbiterr_UNCONNECTED),
        .axi_r_injectdbiterr(1'b0),
        .axi_r_injectsbiterr(1'b0),
        .axi_r_overflow(NLW_fifo_gen_inst_axi_r_overflow_UNCONNECTED),
        .axi_r_prog_empty(NLW_fifo_gen_inst_axi_r_prog_empty_UNCONNECTED),
        .axi_r_prog_empty_thresh({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .axi_r_prog_full(NLW_fifo_gen_inst_axi_r_prog_full_UNCONNECTED),
        .axi_r_prog_full_thresh({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .axi_r_rd_data_count(NLW_fifo_gen_inst_axi_r_rd_data_count_UNCONNECTED[10:0]),
        .axi_r_sbiterr(NLW_fifo_gen_inst_axi_r_sbiterr_UNCONNECTED),
        .axi_r_underflow(NLW_fifo_gen_inst_axi_r_underflow_UNCONNECTED),
        .axi_r_wr_data_count(NLW_fifo_gen_inst_axi_r_wr_data_count_UNCONNECTED[10:0]),
        .axi_w_data_count(NLW_fifo_gen_inst_axi_w_data_count_UNCONNECTED[10:0]),
        .axi_w_dbiterr(NLW_fifo_gen_inst_axi_w_dbiterr_UNCONNECTED),
        .axi_w_injectdbiterr(1'b0),
        .axi_w_injectsbiterr(1'b0),
        .axi_w_overflow(NLW_fifo_gen_inst_axi_w_overflow_UNCONNECTED),
        .axi_w_prog_empty(NLW_fifo_gen_inst_axi_w_prog_empty_UNCONNECTED),
        .axi_w_prog_empty_thresh({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .axi_w_prog_full(NLW_fifo_gen_inst_axi_w_prog_full_UNCONNECTED),
        .axi_w_prog_full_thresh({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .axi_w_rd_data_count(NLW_fifo_gen_inst_axi_w_rd_data_count_UNCONNECTED[10:0]),
        .axi_w_sbiterr(NLW_fifo_gen_inst_axi_w_sbiterr_UNCONNECTED),
        .axi_w_underflow(NLW_fifo_gen_inst_axi_w_underflow_UNCONNECTED),
        .axi_w_wr_data_count(NLW_fifo_gen_inst_axi_w_wr_data_count_UNCONNECTED[10:0]),
        .axis_data_count(NLW_fifo_gen_inst_axis_data_count_UNCONNECTED[10:0]),
        .axis_dbiterr(NLW_fifo_gen_inst_axis_dbiterr_UNCONNECTED),
        .axis_injectdbiterr(1'b0),
        .axis_injectsbiterr(1'b0),
        .axis_overflow(NLW_fifo_gen_inst_axis_overflow_UNCONNECTED),
        .axis_prog_empty(NLW_fifo_gen_inst_axis_prog_empty_UNCONNECTED),
        .axis_prog_empty_thresh({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .axis_prog_full(NLW_fifo_gen_inst_axis_prog_full_UNCONNECTED),
        .axis_prog_full_thresh({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .axis_rd_data_count(NLW_fifo_gen_inst_axis_rd_data_count_UNCONNECTED[10:0]),
        .axis_sbiterr(NLW_fifo_gen_inst_axis_sbiterr_UNCONNECTED),
        .axis_underflow(NLW_fifo_gen_inst_axis_underflow_UNCONNECTED),
        .axis_wr_data_count(NLW_fifo_gen_inst_axis_wr_data_count_UNCONNECTED[10:0]),
        .backup(1'b0),
        .backup_marker(1'b0),
        .clk(aclk),
        .data_count(NLW_fifo_gen_inst_data_count_UNCONNECTED[5:0]),
        .dbiterr(NLW_fifo_gen_inst_dbiterr_UNCONNECTED),
        .din({1'b0,m_axi_awlen}),
        .dout({NLW_fifo_gen_inst_dout_UNCONNECTED[4],dout}),
        .empty(empty),
        .full(full),
        .injectdbiterr(1'b0),
        .injectsbiterr(1'b0),
        .int_clk(1'b0),
        .m_aclk(1'b0),
        .m_aclk_en(1'b0),
        .m_axi_araddr(NLW_fifo_gen_inst_m_axi_araddr_UNCONNECTED[31:0]),
        .m_axi_arburst(NLW_fifo_gen_inst_m_axi_arburst_UNCONNECTED[1:0]),
        .m_axi_arcache(NLW_fifo_gen_inst_m_axi_arcache_UNCONNECTED[3:0]),
        .m_axi_arid(NLW_fifo_gen_inst_m_axi_arid_UNCONNECTED[3:0]),
        .m_axi_arlen(NLW_fifo_gen_inst_m_axi_arlen_UNCONNECTED[7:0]),
        .m_axi_arlock(NLW_fifo_gen_inst_m_axi_arlock_UNCONNECTED[1:0]),
        .m_axi_arprot(NLW_fifo_gen_inst_m_axi_arprot_UNCONNECTED[2:0]),
        .m_axi_arqos(NLW_fifo_gen_inst_m_axi_arqos_UNCONNECTED[3:0]),
        .m_axi_arready(1'b0),
        .m_axi_arregion(NLW_fifo_gen_inst_m_axi_arregion_UNCONNECTED[3:0]),
        .m_axi_arsize(NLW_fifo_gen_inst_m_axi_arsize_UNCONNECTED[2:0]),
        .m_axi_aruser(NLW_fifo_gen_inst_m_axi_aruser_UNCONNECTED[0]),
        .m_axi_arvalid(NLW_fifo_gen_inst_m_axi_arvalid_UNCONNECTED),
        .m_axi_awaddr(NLW_fifo_gen_inst_m_axi_awaddr_UNCONNECTED[31:0]),
        .m_axi_awburst(NLW_fifo_gen_inst_m_axi_awburst_UNCONNECTED[1:0]),
        .m_axi_awcache(NLW_fifo_gen_inst_m_axi_awcache_UNCONNECTED[3:0]),
        .m_axi_awid(NLW_fifo_gen_inst_m_axi_awid_UNCONNECTED[3:0]),
        .m_axi_awlen(NLW_fifo_gen_inst_m_axi_awlen_UNCONNECTED[7:0]),
        .m_axi_awlock(NLW_fifo_gen_inst_m_axi_awlock_UNCONNECTED[1:0]),
        .m_axi_awprot(NLW_fifo_gen_inst_m_axi_awprot_UNCONNECTED[2:0]),
        .m_axi_awqos(NLW_fifo_gen_inst_m_axi_awqos_UNCONNECTED[3:0]),
        .m_axi_awready(1'b0),
        .m_axi_awregion(NLW_fifo_gen_inst_m_axi_awregion_UNCONNECTED[3:0]),
        .m_axi_awsize(NLW_fifo_gen_inst_m_axi_awsize_UNCONNECTED[2:0]),
        .m_axi_awuser(NLW_fifo_gen_inst_m_axi_awuser_UNCONNECTED[0]),
        .m_axi_awvalid(NLW_fifo_gen_inst_m_axi_awvalid_UNCONNECTED),
        .m_axi_bid({1'b0,1'b0,1'b0,1'b0}),
        .m_axi_bready(NLW_fifo_gen_inst_m_axi_bready_UNCONNECTED),
        .m_axi_bresp({1'b0,1'b0}),
        .m_axi_buser(1'b0),
        .m_axi_bvalid(1'b0),
        .m_axi_rdata({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .m_axi_rid({1'b0,1'b0,1'b0,1'b0}),
        .m_axi_rlast(1'b0),
        .m_axi_rready(NLW_fifo_gen_inst_m_axi_rready_UNCONNECTED),
        .m_axi_rresp({1'b0,1'b0}),
        .m_axi_ruser(1'b0),
        .m_axi_rvalid(1'b0),
        .m_axi_wdata(NLW_fifo_gen_inst_m_axi_wdata_UNCONNECTED[63:0]),
        .m_axi_wid(NLW_fifo_gen_inst_m_axi_wid_UNCONNECTED[3:0]),
        .m_axi_wlast(NLW_fifo_gen_inst_m_axi_wlast_UNCONNECTED),
        .m_axi_wready(1'b0),
        .m_axi_wstrb(NLW_fifo_gen_inst_m_axi_wstrb_UNCONNECTED[7:0]),
        .m_axi_wuser(NLW_fifo_gen_inst_m_axi_wuser_UNCONNECTED[0]),
        .m_axi_wvalid(NLW_fifo_gen_inst_m_axi_wvalid_UNCONNECTED),
        .m_axis_tdata(NLW_fifo_gen_inst_m_axis_tdata_UNCONNECTED[63:0]),
        .m_axis_tdest(NLW_fifo_gen_inst_m_axis_tdest_UNCONNECTED[3:0]),
        .m_axis_tid(NLW_fifo_gen_inst_m_axis_tid_UNCONNECTED[7:0]),
        .m_axis_tkeep(NLW_fifo_gen_inst_m_axis_tkeep_UNCONNECTED[3:0]),
        .m_axis_tlast(NLW_fifo_gen_inst_m_axis_tlast_UNCONNECTED),
        .m_axis_tready(1'b0),
        .m_axis_tstrb(NLW_fifo_gen_inst_m_axis_tstrb_UNCONNECTED[3:0]),
        .m_axis_tuser(NLW_fifo_gen_inst_m_axis_tuser_UNCONNECTED[3:0]),
        .m_axis_tvalid(NLW_fifo_gen_inst_m_axis_tvalid_UNCONNECTED),
        .overflow(NLW_fifo_gen_inst_overflow_UNCONNECTED),
        .prog_empty(NLW_fifo_gen_inst_prog_empty_UNCONNECTED),
        .prog_empty_thresh({1'b0,1'b0,1'b0,1'b0,1'b0}),
        .prog_empty_thresh_assert({1'b0,1'b0,1'b0,1'b0,1'b0}),
        .prog_empty_thresh_negate({1'b0,1'b0,1'b0,1'b0,1'b0}),
        .prog_full(NLW_fifo_gen_inst_prog_full_UNCONNECTED),
        .prog_full_thresh({1'b0,1'b0,1'b0,1'b0,1'b0}),
        .prog_full_thresh_assert({1'b0,1'b0,1'b0,1'b0,1'b0}),
        .prog_full_thresh_negate({1'b0,1'b0,1'b0,1'b0,1'b0}),
        .rd_clk(1'b0),
        .rd_data_count(NLW_fifo_gen_inst_rd_data_count_UNCONNECTED[5:0]),
        .rd_en(rd_en),
        .rd_rst(1'b0),
        .rd_rst_busy(NLW_fifo_gen_inst_rd_rst_busy_UNCONNECTED),
        .rst(SR),
        .s_aclk(1'b0),
        .s_aclk_en(1'b0),
        .s_aresetn(1'b0),
        .s_axi_araddr({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .s_axi_arburst({1'b0,1'b0}),
        .s_axi_arcache({1'b0,1'b0,1'b0,1'b0}),
        .s_axi_arid({1'b0,1'b0,1'b0,1'b0}),
        .s_axi_arlen({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .s_axi_arlock({1'b0,1'b0}),
        .s_axi_arprot({1'b0,1'b0,1'b0}),
        .s_axi_arqos({1'b0,1'b0,1'b0,1'b0}),
        .s_axi_arready(NLW_fifo_gen_inst_s_axi_arready_UNCONNECTED),
        .s_axi_arregion({1'b0,1'b0,1'b0,1'b0}),
        .s_axi_arsize({1'b0,1'b0,1'b0}),
        .s_axi_aruser(1'b0),
        .s_axi_arvalid(1'b0),
        .s_axi_awaddr({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .s_axi_awburst({1'b0,1'b0}),
        .s_axi_awcache({1'b0,1'b0,1'b0,1'b0}),
        .s_axi_awid({1'b0,1'b0,1'b0,1'b0}),
        .s_axi_awlen({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .s_axi_awlock({1'b0,1'b0}),
        .s_axi_awprot({1'b0,1'b0,1'b0}),
        .s_axi_awqos({1'b0,1'b0,1'b0,1'b0}),
        .s_axi_awready(NLW_fifo_gen_inst_s_axi_awready_UNCONNECTED),
        .s_axi_awregion({1'b0,1'b0,1'b0,1'b0}),
        .s_axi_awsize({1'b0,1'b0,1'b0}),
        .s_axi_awuser(1'b0),
        .s_axi_awvalid(1'b0),
        .s_axi_bid(NLW_fifo_gen_inst_s_axi_bid_UNCONNECTED[3:0]),
        .s_axi_bready(1'b0),
        .s_axi_bresp(NLW_fifo_gen_inst_s_axi_bresp_UNCONNECTED[1:0]),
        .s_axi_buser(NLW_fifo_gen_inst_s_axi_buser_UNCONNECTED[0]),
        .s_axi_bvalid(NLW_fifo_gen_inst_s_axi_bvalid_UNCONNECTED),
        .s_axi_rdata(NLW_fifo_gen_inst_s_axi_rdata_UNCONNECTED[63:0]),
        .s_axi_rid(NLW_fifo_gen_inst_s_axi_rid_UNCONNECTED[3:0]),
        .s_axi_rlast(NLW_fifo_gen_inst_s_axi_rlast_UNCONNECTED),
        .s_axi_rready(1'b0),
        .s_axi_rresp(NLW_fifo_gen_inst_s_axi_rresp_UNCONNECTED[1:0]),
        .s_axi_ruser(NLW_fifo_gen_inst_s_axi_ruser_UNCONNECTED[0]),
        .s_axi_rvalid(NLW_fifo_gen_inst_s_axi_rvalid_UNCONNECTED),
        .s_axi_wdata({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .s_axi_wid({1'b0,1'b0,1'b0,1'b0}),
        .s_axi_wlast(1'b0),
        .s_axi_wready(NLW_fifo_gen_inst_s_axi_wready_UNCONNECTED),
        .s_axi_wstrb({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .s_axi_wuser(1'b0),
        .s_axi_wvalid(1'b0),
        .s_axis_tdata({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .s_axis_tdest({1'b0,1'b0,1'b0,1'b0}),
        .s_axis_tid({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .s_axis_tkeep({1'b0,1'b0,1'b0,1'b0}),
        .s_axis_tlast(1'b0),
        .s_axis_tready(NLW_fifo_gen_inst_s_axis_tready_UNCONNECTED),
        .s_axis_tstrb({1'b0,1'b0,1'b0,1'b0}),
        .s_axis_tuser({1'b0,1'b0,1'b0,1'b0}),
        .s_axis_tvalid(1'b0),
        .sbiterr(NLW_fifo_gen_inst_sbiterr_UNCONNECTED),
        .sleep(1'b0),
        .srst(1'b0),
        .underflow(NLW_fifo_gen_inst_underflow_UNCONNECTED),
        .valid(NLW_fifo_gen_inst_valid_UNCONNECTED),
        .wr_ack(NLW_fifo_gen_inst_wr_ack_UNCONNECTED),
        .wr_clk(1'b0),
        .wr_data_count(NLW_fifo_gen_inst_wr_data_count_UNCONNECTED[5:0]),
        .wr_en(cmd_push),
        .wr_rst(1'b0),
        .wr_rst_busy(NLW_fifo_gen_inst_wr_rst_busy_UNCONNECTED));
  (* SOFT_HLUTNM = "soft_lutpair7" *) 
  LUT3 #(
    .INIT(8'h02)) 
    fifo_gen_inst_i_1
       (.I0(command_ongoing),
        .I1(full),
        .I2(m_axi_awvalid_0),
        .O(cmd_push));
  LUT6 #(
    .INIT(64'hE4E4CC664E4ECC66)) 
    \length_counter_1[1]_i_1 
       (.I0(empty_fwft_i_reg),
        .I1(length_counter_1_reg[1]),
        .I2(dout[1]),
        .I3(length_counter_1_reg[0]),
        .I4(first_mi_word),
        .I5(dout[0]),
        .O(length_counter_1_reg_1_sn_1));
  LUT3 #(
    .INIT(8'hA2)) 
    m_axi_awvalid_INST_0
       (.I0(command_ongoing),
        .I1(full),
        .I2(m_axi_awvalid_0),
        .O(m_axi_awvalid));
  (* SOFT_HLUTNM = "soft_lutpair6" *) 
  LUT2 #(
    .INIT(4'h2)) 
    m_axi_wvalid_INST_0
       (.I0(s_axi_wvalid),
        .I1(empty),
        .O(m_axi_wvalid));
  (* SOFT_HLUTNM = "soft_lutpair6" *) 
  LUT3 #(
    .INIT(8'h40)) 
    s_axi_wready_INST_0
       (.I0(empty),
        .I1(s_axi_wvalid),
        .I2(m_axi_wready),
        .O(empty_fwft_i_reg));
endmodule

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_axi_protocol_converter_v2_1_37_a_axi3_conv
   (dout,
    empty,
    SR,
    m_axi_awlen,
    m_axi_awlock,
    E,
    m_axi_awvalid,
    length_counter_1_reg_1_sp_1,
    empty_fwft_i_reg,
    m_axi_wvalid,
    m_axi_awaddr,
    m_axi_awsize,
    m_axi_awburst,
    m_axi_awcache,
    m_axi_awprot,
    m_axi_awqos,
    aclk,
    rd_en,
    s_axi_awlock,
    aresetn,
    m_axi_awready,
    length_counter_1_reg,
    first_mi_word,
    s_axi_wvalid,
    m_axi_wready,
    s_axi_awvalid,
    s_axi_awaddr,
    s_axi_awlen,
    s_axi_awsize,
    s_axi_awburst,
    s_axi_awcache,
    s_axi_awprot,
    s_axi_awqos);
  output [3:0]dout;
  output empty;
  output [0:0]SR;
  output [3:0]m_axi_awlen;
  output [0:0]m_axi_awlock;
  output [0:0]E;
  output m_axi_awvalid;
  output length_counter_1_reg_1_sp_1;
  output empty_fwft_i_reg;
  output m_axi_wvalid;
  output [31:0]m_axi_awaddr;
  output [2:0]m_axi_awsize;
  output [1:0]m_axi_awburst;
  output [3:0]m_axi_awcache;
  output [2:0]m_axi_awprot;
  output [3:0]m_axi_awqos;
  input aclk;
  input rd_en;
  input [0:0]s_axi_awlock;
  input aresetn;
  input m_axi_awready;
  input [1:0]length_counter_1_reg;
  input first_mi_word;
  input s_axi_wvalid;
  input m_axi_wready;
  input s_axi_awvalid;
  input [31:0]s_axi_awaddr;
  input [3:0]s_axi_awlen;
  input [2:0]s_axi_awsize;
  input [1:0]s_axi_awburst;
  input [3:0]s_axi_awcache;
  input [2:0]s_axi_awprot;
  input [3:0]s_axi_awqos;

  wire [0:0]E;
  wire [0:0]SR;
  wire \USE_BURSTS.cmd_queue_n_11 ;
  wire \USE_BURSTS.cmd_queue_n_12 ;
  wire \USE_BURSTS.cmd_queue_n_6 ;
  wire aclk;
  wire [1:0]areset_d;
  wire aresetn;
  wire cmd_push_block_reg_n_0;
  wire command_ongoing;
  wire [3:0]dout;
  wire empty;
  wire empty_fwft_i_reg;
  wire first_mi_word;
  wire [1:0]length_counter_1_reg;
  wire length_counter_1_reg_1_sn_1;
  wire [31:0]m_axi_awaddr;
  wire [1:0]m_axi_awburst;
  wire [3:0]m_axi_awcache;
  wire [3:0]m_axi_awlen;
  wire [0:0]m_axi_awlock;
  wire [2:0]m_axi_awprot;
  wire [3:0]m_axi_awqos;
  wire m_axi_awready;
  wire [2:0]m_axi_awsize;
  wire m_axi_awvalid;
  wire m_axi_wready;
  wire m_axi_wvalid;
  wire rd_en;
  wire [31:0]s_axi_awaddr;
  wire [1:0]s_axi_awburst;
  wire [3:0]s_axi_awcache;
  wire [3:0]s_axi_awlen;
  wire [0:0]s_axi_awlock;
  wire [2:0]s_axi_awprot;
  wire [3:0]s_axi_awqos;
  wire [2:0]s_axi_awsize;
  wire s_axi_awvalid;
  wire s_axi_wvalid;

  assign length_counter_1_reg_1_sp_1 = length_counter_1_reg_1_sn_1;
  FDRE \S_AXI_AADDR_Q_reg[0] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awaddr[0]),
        .Q(m_axi_awaddr[0]),
        .R(SR));
  FDRE \S_AXI_AADDR_Q_reg[10] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awaddr[10]),
        .Q(m_axi_awaddr[10]),
        .R(SR));
  FDRE \S_AXI_AADDR_Q_reg[11] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awaddr[11]),
        .Q(m_axi_awaddr[11]),
        .R(SR));
  FDRE \S_AXI_AADDR_Q_reg[12] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awaddr[12]),
        .Q(m_axi_awaddr[12]),
        .R(SR));
  FDRE \S_AXI_AADDR_Q_reg[13] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awaddr[13]),
        .Q(m_axi_awaddr[13]),
        .R(SR));
  FDRE \S_AXI_AADDR_Q_reg[14] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awaddr[14]),
        .Q(m_axi_awaddr[14]),
        .R(SR));
  FDRE \S_AXI_AADDR_Q_reg[15] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awaddr[15]),
        .Q(m_axi_awaddr[15]),
        .R(SR));
  FDRE \S_AXI_AADDR_Q_reg[16] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awaddr[16]),
        .Q(m_axi_awaddr[16]),
        .R(SR));
  FDRE \S_AXI_AADDR_Q_reg[17] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awaddr[17]),
        .Q(m_axi_awaddr[17]),
        .R(SR));
  FDRE \S_AXI_AADDR_Q_reg[18] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awaddr[18]),
        .Q(m_axi_awaddr[18]),
        .R(SR));
  FDRE \S_AXI_AADDR_Q_reg[19] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awaddr[19]),
        .Q(m_axi_awaddr[19]),
        .R(SR));
  FDRE \S_AXI_AADDR_Q_reg[1] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awaddr[1]),
        .Q(m_axi_awaddr[1]),
        .R(SR));
  FDRE \S_AXI_AADDR_Q_reg[20] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awaddr[20]),
        .Q(m_axi_awaddr[20]),
        .R(SR));
  FDRE \S_AXI_AADDR_Q_reg[21] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awaddr[21]),
        .Q(m_axi_awaddr[21]),
        .R(SR));
  FDRE \S_AXI_AADDR_Q_reg[22] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awaddr[22]),
        .Q(m_axi_awaddr[22]),
        .R(SR));
  FDRE \S_AXI_AADDR_Q_reg[23] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awaddr[23]),
        .Q(m_axi_awaddr[23]),
        .R(SR));
  FDRE \S_AXI_AADDR_Q_reg[24] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awaddr[24]),
        .Q(m_axi_awaddr[24]),
        .R(SR));
  FDRE \S_AXI_AADDR_Q_reg[25] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awaddr[25]),
        .Q(m_axi_awaddr[25]),
        .R(SR));
  FDRE \S_AXI_AADDR_Q_reg[26] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awaddr[26]),
        .Q(m_axi_awaddr[26]),
        .R(SR));
  FDRE \S_AXI_AADDR_Q_reg[27] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awaddr[27]),
        .Q(m_axi_awaddr[27]),
        .R(SR));
  FDRE \S_AXI_AADDR_Q_reg[28] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awaddr[28]),
        .Q(m_axi_awaddr[28]),
        .R(SR));
  FDRE \S_AXI_AADDR_Q_reg[29] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awaddr[29]),
        .Q(m_axi_awaddr[29]),
        .R(SR));
  FDRE \S_AXI_AADDR_Q_reg[2] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awaddr[2]),
        .Q(m_axi_awaddr[2]),
        .R(SR));
  FDRE \S_AXI_AADDR_Q_reg[30] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awaddr[30]),
        .Q(m_axi_awaddr[30]),
        .R(SR));
  FDRE \S_AXI_AADDR_Q_reg[31] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awaddr[31]),
        .Q(m_axi_awaddr[31]),
        .R(SR));
  FDRE \S_AXI_AADDR_Q_reg[3] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awaddr[3]),
        .Q(m_axi_awaddr[3]),
        .R(SR));
  FDRE \S_AXI_AADDR_Q_reg[4] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awaddr[4]),
        .Q(m_axi_awaddr[4]),
        .R(SR));
  FDRE \S_AXI_AADDR_Q_reg[5] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awaddr[5]),
        .Q(m_axi_awaddr[5]),
        .R(SR));
  FDRE \S_AXI_AADDR_Q_reg[6] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awaddr[6]),
        .Q(m_axi_awaddr[6]),
        .R(SR));
  FDRE \S_AXI_AADDR_Q_reg[7] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awaddr[7]),
        .Q(m_axi_awaddr[7]),
        .R(SR));
  FDRE \S_AXI_AADDR_Q_reg[8] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awaddr[8]),
        .Q(m_axi_awaddr[8]),
        .R(SR));
  FDRE \S_AXI_AADDR_Q_reg[9] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awaddr[9]),
        .Q(m_axi_awaddr[9]),
        .R(SR));
  FDRE \S_AXI_ABURST_Q_reg[0] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awburst[0]),
        .Q(m_axi_awburst[0]),
        .R(SR));
  FDRE \S_AXI_ABURST_Q_reg[1] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awburst[1]),
        .Q(m_axi_awburst[1]),
        .R(SR));
  FDRE \S_AXI_ACACHE_Q_reg[0] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awcache[0]),
        .Q(m_axi_awcache[0]),
        .R(SR));
  FDRE \S_AXI_ACACHE_Q_reg[1] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awcache[1]),
        .Q(m_axi_awcache[1]),
        .R(SR));
  FDRE \S_AXI_ACACHE_Q_reg[2] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awcache[2]),
        .Q(m_axi_awcache[2]),
        .R(SR));
  FDRE \S_AXI_ACACHE_Q_reg[3] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awcache[3]),
        .Q(m_axi_awcache[3]),
        .R(SR));
  FDRE \S_AXI_ALEN_Q_reg[0] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awlen[0]),
        .Q(m_axi_awlen[0]),
        .R(SR));
  FDRE \S_AXI_ALEN_Q_reg[1] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awlen[1]),
        .Q(m_axi_awlen[1]),
        .R(SR));
  FDRE \S_AXI_ALEN_Q_reg[2] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awlen[2]),
        .Q(m_axi_awlen[2]),
        .R(SR));
  FDRE \S_AXI_ALEN_Q_reg[3] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awlen[3]),
        .Q(m_axi_awlen[3]),
        .R(SR));
  FDRE \S_AXI_ALOCK_Q_reg[0] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awlock),
        .Q(m_axi_awlock),
        .R(SR));
  FDRE \S_AXI_APROT_Q_reg[0] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awprot[0]),
        .Q(m_axi_awprot[0]),
        .R(SR));
  FDRE \S_AXI_APROT_Q_reg[1] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awprot[1]),
        .Q(m_axi_awprot[1]),
        .R(SR));
  FDRE \S_AXI_APROT_Q_reg[2] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awprot[2]),
        .Q(m_axi_awprot[2]),
        .R(SR));
  FDRE \S_AXI_AQOS_Q_reg[0] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awqos[0]),
        .Q(m_axi_awqos[0]),
        .R(SR));
  FDRE \S_AXI_AQOS_Q_reg[1] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awqos[1]),
        .Q(m_axi_awqos[1]),
        .R(SR));
  FDRE \S_AXI_AQOS_Q_reg[2] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awqos[2]),
        .Q(m_axi_awqos[2]),
        .R(SR));
  FDRE \S_AXI_AQOS_Q_reg[3] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awqos[3]),
        .Q(m_axi_awqos[3]),
        .R(SR));
  FDRE #(
    .INIT(1'b0)) 
    S_AXI_AREADY_I_reg
       (.C(aclk),
        .CE(1'b1),
        .D(\USE_BURSTS.cmd_queue_n_11 ),
        .Q(E),
        .R(SR));
  FDRE \S_AXI_ASIZE_Q_reg[0] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awsize[0]),
        .Q(m_axi_awsize[0]),
        .R(SR));
  FDRE \S_AXI_ASIZE_Q_reg[1] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awsize[1]),
        .Q(m_axi_awsize[1]),
        .R(SR));
  FDRE \S_AXI_ASIZE_Q_reg[2] 
       (.C(aclk),
        .CE(E),
        .D(s_axi_awsize[2]),
        .Q(m_axi_awsize[2]),
        .R(SR));
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_axi_data_fifo_v2_1_36_axic_fifo \USE_BURSTS.cmd_queue 
       (.E(E),
        .Q(areset_d),
        .SR(SR),
        .S_AXI_AREADY_I_reg(\USE_BURSTS.cmd_queue_n_11 ),
        .aclk(aclk),
        .\areset_d_reg[1] (\USE_BURSTS.cmd_queue_n_12 ),
        .aresetn(aresetn),
        .aresetn_0(\USE_BURSTS.cmd_queue_n_6 ),
        .command_ongoing(command_ongoing),
        .dout(dout),
        .empty(empty),
        .empty_fwft_i_reg(empty_fwft_i_reg),
        .first_mi_word(first_mi_word),
        .length_counter_1_reg(length_counter_1_reg),
        .length_counter_1_reg_1_sp_1(length_counter_1_reg_1_sn_1),
        .m_axi_awlen(m_axi_awlen),
        .m_axi_awready(m_axi_awready),
        .m_axi_awvalid(m_axi_awvalid),
        .m_axi_awvalid_0(cmd_push_block_reg_n_0),
        .m_axi_wready(m_axi_wready),
        .m_axi_wvalid(m_axi_wvalid),
        .rd_en(rd_en),
        .s_axi_awvalid(s_axi_awvalid),
        .s_axi_wvalid(s_axi_wvalid));
  FDRE #(
    .INIT(1'b0)) 
    \areset_d_reg[0] 
       (.C(aclk),
        .CE(1'b1),
        .D(SR),
        .Q(areset_d[0]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \areset_d_reg[1] 
       (.C(aclk),
        .CE(1'b1),
        .D(areset_d[0]),
        .Q(areset_d[1]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    cmd_push_block_reg
       (.C(aclk),
        .CE(1'b1),
        .D(\USE_BURSTS.cmd_queue_n_6 ),
        .Q(cmd_push_block_reg_n_0),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    command_ongoing_reg
       (.C(aclk),
        .CE(1'b1),
        .D(\USE_BURSTS.cmd_queue_n_12 ),
        .Q(command_ongoing),
        .R(SR));
endmodule

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_axi_protocol_converter_v2_1_37_axi3_conv
   (m_axi_awlen,
    m_axi_awaddr,
    E,
    m_axi_awsize,
    m_axi_awburst,
    m_axi_awlock,
    m_axi_awcache,
    m_axi_awprot,
    m_axi_awqos,
    m_axi_awvalid,
    empty_fwft_i_reg,
    m_axi_wvalid,
    m_axi_wlast,
    aresetn,
    m_axi_awready,
    aclk,
    s_axi_awaddr,
    s_axi_awlen,
    s_axi_awsize,
    s_axi_awburst,
    s_axi_awlock,
    s_axi_awcache,
    s_axi_awprot,
    s_axi_awqos,
    m_axi_wready,
    s_axi_wvalid,
    s_axi_awvalid);
  output [3:0]m_axi_awlen;
  output [31:0]m_axi_awaddr;
  output [0:0]E;
  output [2:0]m_axi_awsize;
  output [1:0]m_axi_awburst;
  output [0:0]m_axi_awlock;
  output [3:0]m_axi_awcache;
  output [2:0]m_axi_awprot;
  output [3:0]m_axi_awqos;
  output m_axi_awvalid;
  output empty_fwft_i_reg;
  output m_axi_wvalid;
  output m_axi_wlast;
  input aresetn;
  input m_axi_awready;
  input aclk;
  input [31:0]s_axi_awaddr;
  input [3:0]s_axi_awlen;
  input [2:0]s_axi_awsize;
  input [1:0]s_axi_awburst;
  input [0:0]s_axi_awlock;
  input [3:0]s_axi_awcache;
  input [2:0]s_axi_awprot;
  input [3:0]s_axi_awqos;
  input m_axi_wready;
  input s_axi_wvalid;
  input s_axi_awvalid;

  wire [0:0]E;
  wire \USE_BURSTS.cmd_queue/inst/empty ;
  wire [3:0]\USE_WRITE.wr_cmd_length ;
  wire \USE_WRITE.wr_cmd_ready ;
  wire \USE_WRITE.write_addr_inst_n_13 ;
  wire \USE_WRITE.write_addr_inst_n_5 ;
  wire aclk;
  wire aresetn;
  wire empty_fwft_i_reg;
  wire first_mi_word;
  wire [1:0]length_counter_1_reg;
  wire [31:0]m_axi_awaddr;
  wire [1:0]m_axi_awburst;
  wire [3:0]m_axi_awcache;
  wire [3:0]m_axi_awlen;
  wire [0:0]m_axi_awlock;
  wire [2:0]m_axi_awprot;
  wire [3:0]m_axi_awqos;
  wire m_axi_awready;
  wire [2:0]m_axi_awsize;
  wire m_axi_awvalid;
  wire m_axi_wlast;
  wire m_axi_wready;
  wire m_axi_wvalid;
  wire [31:0]s_axi_awaddr;
  wire [1:0]s_axi_awburst;
  wire [3:0]s_axi_awcache;
  wire [3:0]s_axi_awlen;
  wire [0:0]s_axi_awlock;
  wire [2:0]s_axi_awprot;
  wire [3:0]s_axi_awqos;
  wire [2:0]s_axi_awsize;
  wire s_axi_awvalid;
  wire s_axi_wvalid;

  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_axi_protocol_converter_v2_1_37_a_axi3_conv \USE_WRITE.write_addr_inst 
       (.E(E),
        .SR(\USE_WRITE.write_addr_inst_n_5 ),
        .aclk(aclk),
        .aresetn(aresetn),
        .dout(\USE_WRITE.wr_cmd_length ),
        .empty(\USE_BURSTS.cmd_queue/inst/empty ),
        .empty_fwft_i_reg(empty_fwft_i_reg),
        .first_mi_word(first_mi_word),
        .length_counter_1_reg(length_counter_1_reg),
        .length_counter_1_reg_1_sp_1(\USE_WRITE.write_addr_inst_n_13 ),
        .m_axi_awaddr(m_axi_awaddr),
        .m_axi_awburst(m_axi_awburst),
        .m_axi_awcache(m_axi_awcache),
        .m_axi_awlen(m_axi_awlen),
        .m_axi_awlock(m_axi_awlock),
        .m_axi_awprot(m_axi_awprot),
        .m_axi_awqos(m_axi_awqos),
        .m_axi_awready(m_axi_awready),
        .m_axi_awsize(m_axi_awsize),
        .m_axi_awvalid(m_axi_awvalid),
        .m_axi_wready(m_axi_wready),
        .m_axi_wvalid(m_axi_wvalid),
        .rd_en(\USE_WRITE.wr_cmd_ready ),
        .s_axi_awaddr(s_axi_awaddr),
        .s_axi_awburst(s_axi_awburst),
        .s_axi_awcache(s_axi_awcache),
        .s_axi_awlen(s_axi_awlen),
        .s_axi_awlock(s_axi_awlock),
        .s_axi_awprot(s_axi_awprot),
        .s_axi_awqos(s_axi_awqos),
        .s_axi_awsize(s_axi_awsize),
        .s_axi_awvalid(s_axi_awvalid),
        .s_axi_wvalid(s_axi_wvalid));
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_axi_protocol_converter_v2_1_37_w_axi3_conv \USE_WRITE.write_data_inst 
       (.SR(\USE_WRITE.write_addr_inst_n_5 ),
        .aclk(aclk),
        .dout(\USE_WRITE.wr_cmd_length ),
        .empty(\USE_BURSTS.cmd_queue/inst/empty ),
        .first_mi_word(first_mi_word),
        .\length_counter_1_reg[1]_0 (length_counter_1_reg),
        .\length_counter_1_reg[1]_1 (\USE_WRITE.write_addr_inst_n_13 ),
        .\length_counter_1_reg[2]_0 (empty_fwft_i_reg),
        .m_axi_wlast(m_axi_wlast),
        .m_axi_wready(m_axi_wready),
        .rd_en(\USE_WRITE.wr_cmd_ready ),
        .s_axi_wvalid(s_axi_wvalid));
endmodule

(* C_AXI_ADDR_WIDTH = "32" *) (* C_AXI_ARUSER_WIDTH = "1" *) (* C_AXI_AWUSER_WIDTH = "1" *) 
(* C_AXI_BUSER_WIDTH = "1" *) (* C_AXI_DATA_WIDTH = "64" *) (* C_AXI_ID_WIDTH = "1" *) 
(* C_AXI_RUSER_WIDTH = "1" *) (* C_AXI_SUPPORTS_READ = "0" *) (* C_AXI_SUPPORTS_USER_SIGNALS = "0" *) 
(* C_AXI_SUPPORTS_WRITE = "1" *) (* C_AXI_WUSER_WIDTH = "1" *) (* C_FAMILY = "zynq" *) 
(* C_IGNORE_ID = "1" *) (* C_M_AXI_PROTOCOL = "1" *) (* C_S_AXI_PROTOCOL = "0" *) 
(* C_TRANSLATION_MODE = "0" *) (* DowngradeIPIdentifiedWarnings = "yes" *) (* P_AXI3 = "1" *) 
(* P_AXI4 = "0" *) (* P_AXILITE = "2" *) (* P_AXILITE_SIZE = "3'b011" *) 
(* P_CONVERSION = "2" *) (* P_DECERR = "2'b11" *) (* P_INCR = "2'b01" *) 
(* P_PROTECTION = "1" *) (* P_SLVERR = "2'b10" *) 
module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_axi_protocol_converter_v2_1_37_axi_protocol_converter
   (aclk,
    aresetn,
    s_axi_awid,
    s_axi_awaddr,
    s_axi_awlen,
    s_axi_awsize,
    s_axi_awburst,
    s_axi_awlock,
    s_axi_awcache,
    s_axi_awprot,
    s_axi_awregion,
    s_axi_awqos,
    s_axi_awuser,
    s_axi_awvalid,
    s_axi_awready,
    s_axi_wid,
    s_axi_wdata,
    s_axi_wstrb,
    s_axi_wlast,
    s_axi_wuser,
    s_axi_wvalid,
    s_axi_wready,
    s_axi_bid,
    s_axi_bresp,
    s_axi_buser,
    s_axi_bvalid,
    s_axi_bready,
    s_axi_arid,
    s_axi_araddr,
    s_axi_arlen,
    s_axi_arsize,
    s_axi_arburst,
    s_axi_arlock,
    s_axi_arcache,
    s_axi_arprot,
    s_axi_arregion,
    s_axi_arqos,
    s_axi_aruser,
    s_axi_arvalid,
    s_axi_arready,
    s_axi_rid,
    s_axi_rdata,
    s_axi_rresp,
    s_axi_rlast,
    s_axi_ruser,
    s_axi_rvalid,
    s_axi_rready,
    m_axi_awid,
    m_axi_awaddr,
    m_axi_awlen,
    m_axi_awsize,
    m_axi_awburst,
    m_axi_awlock,
    m_axi_awcache,
    m_axi_awprot,
    m_axi_awregion,
    m_axi_awqos,
    m_axi_awuser,
    m_axi_awvalid,
    m_axi_awready,
    m_axi_wid,
    m_axi_wdata,
    m_axi_wstrb,
    m_axi_wlast,
    m_axi_wuser,
    m_axi_wvalid,
    m_axi_wready,
    m_axi_bid,
    m_axi_bresp,
    m_axi_buser,
    m_axi_bvalid,
    m_axi_bready,
    m_axi_arid,
    m_axi_araddr,
    m_axi_arlen,
    m_axi_arsize,
    m_axi_arburst,
    m_axi_arlock,
    m_axi_arcache,
    m_axi_arprot,
    m_axi_arregion,
    m_axi_arqos,
    m_axi_aruser,
    m_axi_arvalid,
    m_axi_arready,
    m_axi_rid,
    m_axi_rdata,
    m_axi_rresp,
    m_axi_rlast,
    m_axi_ruser,
    m_axi_rvalid,
    m_axi_rready);
  input aclk;
  input aresetn;
  input [0:0]s_axi_awid;
  input [31:0]s_axi_awaddr;
  input [7:0]s_axi_awlen;
  input [2:0]s_axi_awsize;
  input [1:0]s_axi_awburst;
  input [0:0]s_axi_awlock;
  input [3:0]s_axi_awcache;
  input [2:0]s_axi_awprot;
  input [3:0]s_axi_awregion;
  input [3:0]s_axi_awqos;
  input [0:0]s_axi_awuser;
  input s_axi_awvalid;
  output s_axi_awready;
  input [0:0]s_axi_wid;
  input [63:0]s_axi_wdata;
  input [7:0]s_axi_wstrb;
  input s_axi_wlast;
  input [0:0]s_axi_wuser;
  input s_axi_wvalid;
  output s_axi_wready;
  output [0:0]s_axi_bid;
  output [1:0]s_axi_bresp;
  output [0:0]s_axi_buser;
  output s_axi_bvalid;
  input s_axi_bready;
  input [0:0]s_axi_arid;
  input [31:0]s_axi_araddr;
  input [7:0]s_axi_arlen;
  input [2:0]s_axi_arsize;
  input [1:0]s_axi_arburst;
  input [0:0]s_axi_arlock;
  input [3:0]s_axi_arcache;
  input [2:0]s_axi_arprot;
  input [3:0]s_axi_arregion;
  input [3:0]s_axi_arqos;
  input [0:0]s_axi_aruser;
  input s_axi_arvalid;
  output s_axi_arready;
  output [0:0]s_axi_rid;
  output [63:0]s_axi_rdata;
  output [1:0]s_axi_rresp;
  output s_axi_rlast;
  output [0:0]s_axi_ruser;
  output s_axi_rvalid;
  input s_axi_rready;
  output [0:0]m_axi_awid;
  output [31:0]m_axi_awaddr;
  output [3:0]m_axi_awlen;
  output [2:0]m_axi_awsize;
  output [1:0]m_axi_awburst;
  output [1:0]m_axi_awlock;
  output [3:0]m_axi_awcache;
  output [2:0]m_axi_awprot;
  output [3:0]m_axi_awregion;
  output [3:0]m_axi_awqos;
  output [0:0]m_axi_awuser;
  output m_axi_awvalid;
  input m_axi_awready;
  output [0:0]m_axi_wid;
  output [63:0]m_axi_wdata;
  output [7:0]m_axi_wstrb;
  output m_axi_wlast;
  output [0:0]m_axi_wuser;
  output m_axi_wvalid;
  input m_axi_wready;
  input [0:0]m_axi_bid;
  input [1:0]m_axi_bresp;
  input [0:0]m_axi_buser;
  input m_axi_bvalid;
  output m_axi_bready;
  output [0:0]m_axi_arid;
  output [31:0]m_axi_araddr;
  output [3:0]m_axi_arlen;
  output [2:0]m_axi_arsize;
  output [1:0]m_axi_arburst;
  output [1:0]m_axi_arlock;
  output [3:0]m_axi_arcache;
  output [2:0]m_axi_arprot;
  output [3:0]m_axi_arregion;
  output [3:0]m_axi_arqos;
  output [0:0]m_axi_aruser;
  output m_axi_arvalid;
  input m_axi_arready;
  input [0:0]m_axi_rid;
  input [63:0]m_axi_rdata;
  input [1:0]m_axi_rresp;
  input m_axi_rlast;
  input [0:0]m_axi_ruser;
  input m_axi_rvalid;
  output m_axi_rready;

  wire \<const0> ;
  wire aclk;
  wire aresetn;
  wire [31:0]m_axi_awaddr;
  wire [1:0]m_axi_awburst;
  wire [3:0]m_axi_awcache;
  wire [3:0]m_axi_awlen;
  wire [0:0]\^m_axi_awlock ;
  wire [2:0]m_axi_awprot;
  wire [3:0]m_axi_awqos;
  wire m_axi_awready;
  wire [2:0]m_axi_awsize;
  wire m_axi_awvalid;
  wire [1:0]m_axi_bresp;
  wire m_axi_bvalid;
  wire m_axi_wlast;
  wire m_axi_wready;
  wire m_axi_wvalid;
  wire [31:0]s_axi_awaddr;
  wire [1:0]s_axi_awburst;
  wire [3:0]s_axi_awcache;
  wire [7:0]s_axi_awlen;
  wire [0:0]s_axi_awlock;
  wire [2:0]s_axi_awprot;
  wire [3:0]s_axi_awqos;
  wire s_axi_awready;
  wire [2:0]s_axi_awsize;
  wire s_axi_awvalid;
  wire s_axi_bready;
  wire [63:0]s_axi_wdata;
  wire s_axi_wready;
  wire [7:0]s_axi_wstrb;
  wire s_axi_wvalid;

  assign m_axi_araddr[31] = \<const0> ;
  assign m_axi_araddr[30] = \<const0> ;
  assign m_axi_araddr[29] = \<const0> ;
  assign m_axi_araddr[28] = \<const0> ;
  assign m_axi_araddr[27] = \<const0> ;
  assign m_axi_araddr[26] = \<const0> ;
  assign m_axi_araddr[25] = \<const0> ;
  assign m_axi_araddr[24] = \<const0> ;
  assign m_axi_araddr[23] = \<const0> ;
  assign m_axi_araddr[22] = \<const0> ;
  assign m_axi_araddr[21] = \<const0> ;
  assign m_axi_araddr[20] = \<const0> ;
  assign m_axi_araddr[19] = \<const0> ;
  assign m_axi_araddr[18] = \<const0> ;
  assign m_axi_araddr[17] = \<const0> ;
  assign m_axi_araddr[16] = \<const0> ;
  assign m_axi_araddr[15] = \<const0> ;
  assign m_axi_araddr[14] = \<const0> ;
  assign m_axi_araddr[13] = \<const0> ;
  assign m_axi_araddr[12] = \<const0> ;
  assign m_axi_araddr[11] = \<const0> ;
  assign m_axi_araddr[10] = \<const0> ;
  assign m_axi_araddr[9] = \<const0> ;
  assign m_axi_araddr[8] = \<const0> ;
  assign m_axi_araddr[7] = \<const0> ;
  assign m_axi_araddr[6] = \<const0> ;
  assign m_axi_araddr[5] = \<const0> ;
  assign m_axi_araddr[4] = \<const0> ;
  assign m_axi_araddr[3] = \<const0> ;
  assign m_axi_araddr[2] = \<const0> ;
  assign m_axi_araddr[1] = \<const0> ;
  assign m_axi_araddr[0] = \<const0> ;
  assign m_axi_arburst[1] = \<const0> ;
  assign m_axi_arburst[0] = \<const0> ;
  assign m_axi_arcache[3] = \<const0> ;
  assign m_axi_arcache[2] = \<const0> ;
  assign m_axi_arcache[1] = \<const0> ;
  assign m_axi_arcache[0] = \<const0> ;
  assign m_axi_arid[0] = \<const0> ;
  assign m_axi_arlen[3] = \<const0> ;
  assign m_axi_arlen[2] = \<const0> ;
  assign m_axi_arlen[1] = \<const0> ;
  assign m_axi_arlen[0] = \<const0> ;
  assign m_axi_arlock[1] = \<const0> ;
  assign m_axi_arlock[0] = \<const0> ;
  assign m_axi_arprot[2] = \<const0> ;
  assign m_axi_arprot[1] = \<const0> ;
  assign m_axi_arprot[0] = \<const0> ;
  assign m_axi_arqos[3] = \<const0> ;
  assign m_axi_arqos[2] = \<const0> ;
  assign m_axi_arqos[1] = \<const0> ;
  assign m_axi_arqos[0] = \<const0> ;
  assign m_axi_arregion[3] = \<const0> ;
  assign m_axi_arregion[2] = \<const0> ;
  assign m_axi_arregion[1] = \<const0> ;
  assign m_axi_arregion[0] = \<const0> ;
  assign m_axi_arsize[2] = \<const0> ;
  assign m_axi_arsize[1] = \<const0> ;
  assign m_axi_arsize[0] = \<const0> ;
  assign m_axi_aruser[0] = \<const0> ;
  assign m_axi_arvalid = \<const0> ;
  assign m_axi_awid[0] = \<const0> ;
  assign m_axi_awlock[1] = \<const0> ;
  assign m_axi_awlock[0] = \^m_axi_awlock [0];
  assign m_axi_awregion[3] = \<const0> ;
  assign m_axi_awregion[2] = \<const0> ;
  assign m_axi_awregion[1] = \<const0> ;
  assign m_axi_awregion[0] = \<const0> ;
  assign m_axi_awuser[0] = \<const0> ;
  assign m_axi_bready = s_axi_bready;
  assign m_axi_rready = \<const0> ;
  assign m_axi_wdata[63:0] = s_axi_wdata;
  assign m_axi_wid[0] = \<const0> ;
  assign m_axi_wstrb[7:0] = s_axi_wstrb;
  assign m_axi_wuser[0] = \<const0> ;
  assign s_axi_arready = \<const0> ;
  assign s_axi_bid[0] = \<const0> ;
  assign s_axi_bresp[1:0] = m_axi_bresp;
  assign s_axi_buser[0] = \<const0> ;
  assign s_axi_bvalid = m_axi_bvalid;
  assign s_axi_rdata[63] = \<const0> ;
  assign s_axi_rdata[62] = \<const0> ;
  assign s_axi_rdata[61] = \<const0> ;
  assign s_axi_rdata[60] = \<const0> ;
  assign s_axi_rdata[59] = \<const0> ;
  assign s_axi_rdata[58] = \<const0> ;
  assign s_axi_rdata[57] = \<const0> ;
  assign s_axi_rdata[56] = \<const0> ;
  assign s_axi_rdata[55] = \<const0> ;
  assign s_axi_rdata[54] = \<const0> ;
  assign s_axi_rdata[53] = \<const0> ;
  assign s_axi_rdata[52] = \<const0> ;
  assign s_axi_rdata[51] = \<const0> ;
  assign s_axi_rdata[50] = \<const0> ;
  assign s_axi_rdata[49] = \<const0> ;
  assign s_axi_rdata[48] = \<const0> ;
  assign s_axi_rdata[47] = \<const0> ;
  assign s_axi_rdata[46] = \<const0> ;
  assign s_axi_rdata[45] = \<const0> ;
  assign s_axi_rdata[44] = \<const0> ;
  assign s_axi_rdata[43] = \<const0> ;
  assign s_axi_rdata[42] = \<const0> ;
  assign s_axi_rdata[41] = \<const0> ;
  assign s_axi_rdata[40] = \<const0> ;
  assign s_axi_rdata[39] = \<const0> ;
  assign s_axi_rdata[38] = \<const0> ;
  assign s_axi_rdata[37] = \<const0> ;
  assign s_axi_rdata[36] = \<const0> ;
  assign s_axi_rdata[35] = \<const0> ;
  assign s_axi_rdata[34] = \<const0> ;
  assign s_axi_rdata[33] = \<const0> ;
  assign s_axi_rdata[32] = \<const0> ;
  assign s_axi_rdata[31] = \<const0> ;
  assign s_axi_rdata[30] = \<const0> ;
  assign s_axi_rdata[29] = \<const0> ;
  assign s_axi_rdata[28] = \<const0> ;
  assign s_axi_rdata[27] = \<const0> ;
  assign s_axi_rdata[26] = \<const0> ;
  assign s_axi_rdata[25] = \<const0> ;
  assign s_axi_rdata[24] = \<const0> ;
  assign s_axi_rdata[23] = \<const0> ;
  assign s_axi_rdata[22] = \<const0> ;
  assign s_axi_rdata[21] = \<const0> ;
  assign s_axi_rdata[20] = \<const0> ;
  assign s_axi_rdata[19] = \<const0> ;
  assign s_axi_rdata[18] = \<const0> ;
  assign s_axi_rdata[17] = \<const0> ;
  assign s_axi_rdata[16] = \<const0> ;
  assign s_axi_rdata[15] = \<const0> ;
  assign s_axi_rdata[14] = \<const0> ;
  assign s_axi_rdata[13] = \<const0> ;
  assign s_axi_rdata[12] = \<const0> ;
  assign s_axi_rdata[11] = \<const0> ;
  assign s_axi_rdata[10] = \<const0> ;
  assign s_axi_rdata[9] = \<const0> ;
  assign s_axi_rdata[8] = \<const0> ;
  assign s_axi_rdata[7] = \<const0> ;
  assign s_axi_rdata[6] = \<const0> ;
  assign s_axi_rdata[5] = \<const0> ;
  assign s_axi_rdata[4] = \<const0> ;
  assign s_axi_rdata[3] = \<const0> ;
  assign s_axi_rdata[2] = \<const0> ;
  assign s_axi_rdata[1] = \<const0> ;
  assign s_axi_rdata[0] = \<const0> ;
  assign s_axi_rid[0] = \<const0> ;
  assign s_axi_rlast = \<const0> ;
  assign s_axi_rresp[1] = \<const0> ;
  assign s_axi_rresp[0] = \<const0> ;
  assign s_axi_ruser[0] = \<const0> ;
  assign s_axi_rvalid = \<const0> ;
  GND GND
       (.G(\<const0> ));
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_axi_protocol_converter_v2_1_37_axi3_conv \gen_axi4_axi3.axi3_conv_inst 
       (.E(s_axi_awready),
        .aclk(aclk),
        .aresetn(aresetn),
        .empty_fwft_i_reg(s_axi_wready),
        .m_axi_awaddr(m_axi_awaddr),
        .m_axi_awburst(m_axi_awburst),
        .m_axi_awcache(m_axi_awcache),
        .m_axi_awlen(m_axi_awlen),
        .m_axi_awlock(\^m_axi_awlock ),
        .m_axi_awprot(m_axi_awprot),
        .m_axi_awqos(m_axi_awqos),
        .m_axi_awready(m_axi_awready),
        .m_axi_awsize(m_axi_awsize),
        .m_axi_awvalid(m_axi_awvalid),
        .m_axi_wlast(m_axi_wlast),
        .m_axi_wready(m_axi_wready),
        .m_axi_wvalid(m_axi_wvalid),
        .s_axi_awaddr(s_axi_awaddr),
        .s_axi_awburst(s_axi_awburst),
        .s_axi_awcache(s_axi_awcache),
        .s_axi_awlen(s_axi_awlen[3:0]),
        .s_axi_awlock(s_axi_awlock),
        .s_axi_awprot(s_axi_awprot),
        .s_axi_awqos(s_axi_awqos),
        .s_axi_awsize(s_axi_awsize),
        .s_axi_awvalid(s_axi_awvalid),
        .s_axi_wvalid(s_axi_wvalid));
endmodule

module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_axi_protocol_converter_v2_1_37_w_axi3_conv
   (\length_counter_1_reg[1]_0 ,
    first_mi_word,
    rd_en,
    m_axi_wlast,
    SR,
    aclk,
    \length_counter_1_reg[1]_1 ,
    \length_counter_1_reg[2]_0 ,
    m_axi_wready,
    s_axi_wvalid,
    empty,
    dout);
  output [1:0]\length_counter_1_reg[1]_0 ;
  output first_mi_word;
  output rd_en;
  output m_axi_wlast;
  input [0:0]SR;
  input aclk;
  input \length_counter_1_reg[1]_1 ;
  input \length_counter_1_reg[2]_0 ;
  input m_axi_wready;
  input s_axi_wvalid;
  input empty;
  input [3:0]dout;

  wire [0:0]SR;
  wire aclk;
  wire [3:0]dout;
  wire empty;
  wire first_mi_word;
  wire first_mi_word_i_1_n_0;
  wire \length_counter_1[0]_i_1_n_0 ;
  wire \length_counter_1[2]_i_1_n_0 ;
  wire \length_counter_1[3]_i_1_n_0 ;
  wire \length_counter_1[4]_i_1_n_0 ;
  wire \length_counter_1[4]_i_2_n_0 ;
  wire \length_counter_1[5]_i_1_n_0 ;
  wire \length_counter_1[6]_i_1_n_0 ;
  wire \length_counter_1[7]_i_1_n_0 ;
  wire [7:2]length_counter_1_reg;
  wire [1:0]\length_counter_1_reg[1]_0 ;
  wire \length_counter_1_reg[1]_1 ;
  wire \length_counter_1_reg[2]_0 ;
  wire m_axi_wlast;
  wire m_axi_wlast_INST_0_i_1_n_0;
  wire m_axi_wlast_INST_0_i_2_n_0;
  wire m_axi_wlast_INST_0_i_3_n_0;
  wire m_axi_wready;
  wire rd_en;
  wire s_axi_wvalid;

  LUT6 #(
    .INIT(64'h0000CC000000CC04)) 
    fifo_gen_inst_i_2
       (.I0(length_counter_1_reg[7]),
        .I1(\length_counter_1_reg[2]_0 ),
        .I2(length_counter_1_reg[5]),
        .I3(first_mi_word),
        .I4(m_axi_wlast_INST_0_i_1_n_0),
        .I5(length_counter_1_reg[6]),
        .O(rd_en));
  LUT6 #(
    .INIT(64'h0F0FFFFF00010000)) 
    first_mi_word_i_1
       (.I0(length_counter_1_reg[7]),
        .I1(length_counter_1_reg[5]),
        .I2(m_axi_wlast_INST_0_i_1_n_0),
        .I3(length_counter_1_reg[6]),
        .I4(\length_counter_1_reg[2]_0 ),
        .I5(first_mi_word),
        .O(first_mi_word_i_1_n_0));
  FDSE #(
    .INIT(1'b0)) 
    first_mi_word_reg
       (.C(aclk),
        .CE(1'b1),
        .D(first_mi_word_i_1_n_0),
        .Q(first_mi_word),
        .S(SR));
  LUT6 #(
    .INIT(64'hF2FFFFFF07000000)) 
    \length_counter_1[0]_i_1 
       (.I0(first_mi_word),
        .I1(dout[0]),
        .I2(empty),
        .I3(s_axi_wvalid),
        .I4(m_axi_wready),
        .I5(\length_counter_1_reg[1]_0 [0]),
        .O(\length_counter_1[0]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair9" *) 
  LUT5 #(
    .INIT(32'hD8D272D2)) 
    \length_counter_1[2]_i_1 
       (.I0(\length_counter_1_reg[2]_0 ),
        .I1(m_axi_wlast_INST_0_i_3_n_0),
        .I2(length_counter_1_reg[2]),
        .I3(first_mi_word),
        .I4(dout[2]),
        .O(\length_counter_1[2]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair8" *) 
  LUT5 #(
    .INIT(32'hB8B474B4)) 
    \length_counter_1[3]_i_1 
       (.I0(\length_counter_1[4]_i_2_n_0 ),
        .I1(\length_counter_1_reg[2]_0 ),
        .I2(length_counter_1_reg[3]),
        .I3(first_mi_word),
        .I4(dout[3]),
        .O(\length_counter_1[3]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'h0A0A3A35AAAAAAAA)) 
    \length_counter_1[4]_i_1 
       (.I0(length_counter_1_reg[4]),
        .I1(dout[3]),
        .I2(first_mi_word),
        .I3(length_counter_1_reg[3]),
        .I4(\length_counter_1[4]_i_2_n_0 ),
        .I5(\length_counter_1_reg[2]_0 ),
        .O(\length_counter_1[4]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair9" *) 
  LUT4 #(
    .INIT(16'hFEAE)) 
    \length_counter_1[4]_i_2 
       (.I0(m_axi_wlast_INST_0_i_3_n_0),
        .I1(length_counter_1_reg[2]),
        .I2(first_mi_word),
        .I3(dout[2]),
        .O(\length_counter_1[4]_i_2_n_0 ));
  LUT6 #(
    .INIT(64'hF7FF0000FFF70808)) 
    \length_counter_1[5]_i_1 
       (.I0(m_axi_wready),
        .I1(s_axi_wvalid),
        .I2(empty),
        .I3(first_mi_word),
        .I4(length_counter_1_reg[5]),
        .I5(m_axi_wlast_INST_0_i_1_n_0),
        .O(\length_counter_1[5]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'h3EFF0D00)) 
    \length_counter_1[6]_i_1 
       (.I0(length_counter_1_reg[5]),
        .I1(first_mi_word),
        .I2(m_axi_wlast_INST_0_i_1_n_0),
        .I3(\length_counter_1_reg[2]_0 ),
        .I4(length_counter_1_reg[6]),
        .O(\length_counter_1[6]_i_1_n_0 ));
  LUT6 #(
    .INIT(64'h3F3EFFFF30310000)) 
    \length_counter_1[7]_i_1 
       (.I0(length_counter_1_reg[6]),
        .I1(m_axi_wlast_INST_0_i_1_n_0),
        .I2(first_mi_word),
        .I3(length_counter_1_reg[5]),
        .I4(\length_counter_1_reg[2]_0 ),
        .I5(length_counter_1_reg[7]),
        .O(\length_counter_1[7]_i_1_n_0 ));
  FDRE \length_counter_1_reg[0] 
       (.C(aclk),
        .CE(1'b1),
        .D(\length_counter_1[0]_i_1_n_0 ),
        .Q(\length_counter_1_reg[1]_0 [0]),
        .R(SR));
  FDRE \length_counter_1_reg[1] 
       (.C(aclk),
        .CE(1'b1),
        .D(\length_counter_1_reg[1]_1 ),
        .Q(\length_counter_1_reg[1]_0 [1]),
        .R(SR));
  FDRE \length_counter_1_reg[2] 
       (.C(aclk),
        .CE(1'b1),
        .D(\length_counter_1[2]_i_1_n_0 ),
        .Q(length_counter_1_reg[2]),
        .R(SR));
  FDRE \length_counter_1_reg[3] 
       (.C(aclk),
        .CE(1'b1),
        .D(\length_counter_1[3]_i_1_n_0 ),
        .Q(length_counter_1_reg[3]),
        .R(SR));
  FDRE \length_counter_1_reg[4] 
       (.C(aclk),
        .CE(1'b1),
        .D(\length_counter_1[4]_i_1_n_0 ),
        .Q(length_counter_1_reg[4]),
        .R(SR));
  FDRE \length_counter_1_reg[5] 
       (.C(aclk),
        .CE(1'b1),
        .D(\length_counter_1[5]_i_1_n_0 ),
        .Q(length_counter_1_reg[5]),
        .R(SR));
  FDRE \length_counter_1_reg[6] 
       (.C(aclk),
        .CE(1'b1),
        .D(\length_counter_1[6]_i_1_n_0 ),
        .Q(length_counter_1_reg[6]),
        .R(SR));
  FDRE \length_counter_1_reg[7] 
       (.C(aclk),
        .CE(1'b1),
        .D(\length_counter_1[7]_i_1_n_0 ),
        .Q(length_counter_1_reg[7]),
        .R(SR));
  LUT5 #(
    .INIT(32'h00F000F1)) 
    m_axi_wlast_INST_0
       (.I0(length_counter_1_reg[7]),
        .I1(length_counter_1_reg[5]),
        .I2(first_mi_word),
        .I3(m_axi_wlast_INST_0_i_1_n_0),
        .I4(length_counter_1_reg[6]),
        .O(m_axi_wlast));
  LUT6 #(
    .INIT(64'hFFFFFFFEFCFCFFFE)) 
    m_axi_wlast_INST_0_i_1
       (.I0(length_counter_1_reg[4]),
        .I1(m_axi_wlast_INST_0_i_2_n_0),
        .I2(m_axi_wlast_INST_0_i_3_n_0),
        .I3(length_counter_1_reg[2]),
        .I4(first_mi_word),
        .I5(dout[2]),
        .O(m_axi_wlast_INST_0_i_1_n_0));
  (* SOFT_HLUTNM = "soft_lutpair8" *) 
  LUT3 #(
    .INIT(8'hB8)) 
    m_axi_wlast_INST_0_i_2
       (.I0(dout[3]),
        .I1(first_mi_word),
        .I2(length_counter_1_reg[3]),
        .O(m_axi_wlast_INST_0_i_2_n_0));
  LUT5 #(
    .INIT(32'hFFFACCFA)) 
    m_axi_wlast_INST_0_i_3
       (.I0(\length_counter_1_reg[1]_0 [1]),
        .I1(dout[1]),
        .I2(\length_counter_1_reg[1]_0 [0]),
        .I3(first_mi_word),
        .I4(dout[0]),
        .O(m_axi_wlast_INST_0_i_3_n_0));
endmodule

(* CHECK_LICENSE_TYPE = "drone_block_design_axi_mem_intercon_imp_auto_pc_0,axi_protocol_converter_v2_1_37_axi_protocol_converter,{}" *) (* DowngradeIPIdentifiedWarnings = "yes" *) (* X_CORE_INFO = "axi_protocol_converter_v2_1_37_axi_protocol_converter,Vivado 2025.2" *) 
(* NotValidForBitStream *)
module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix
   (aclk,
    aresetn,
    s_axi_awaddr,
    s_axi_awlen,
    s_axi_awsize,
    s_axi_awburst,
    s_axi_awlock,
    s_axi_awcache,
    s_axi_awprot,
    s_axi_awregion,
    s_axi_awqos,
    s_axi_awvalid,
    s_axi_awready,
    s_axi_wdata,
    s_axi_wstrb,
    s_axi_wlast,
    s_axi_wvalid,
    s_axi_wready,
    s_axi_bresp,
    s_axi_bvalid,
    s_axi_bready,
    m_axi_awaddr,
    m_axi_awlen,
    m_axi_awsize,
    m_axi_awburst,
    m_axi_awlock,
    m_axi_awcache,
    m_axi_awprot,
    m_axi_awqos,
    m_axi_awvalid,
    m_axi_awready,
    m_axi_wdata,
    m_axi_wstrb,
    m_axi_wlast,
    m_axi_wvalid,
    m_axi_wready,
    m_axi_bresp,
    m_axi_bvalid,
    m_axi_bready);
  (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 CLK CLK" *) (* X_INTERFACE_MODE = "slave" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME CLK, ASSOCIATED_BUSIF S_AXI:M_AXI, ASSOCIATED_RESET aresetn, FREQ_HZ 100000000, FREQ_TOLERANCE_HZ 0, PHASE 0.0, CLK_DOMAIN drone_block_design_processing_system7_0_0_FCLK_CLK0, INSERT_VIP 0" *) input aclk;
  (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 RST RST" *) (* X_INTERFACE_MODE = "slave" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME RST, POLARITY ACTIVE_LOW, INSERT_VIP 0, TYPE INTERCONNECT" *) input aresetn;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWADDR" *) (* X_INTERFACE_MODE = "slave" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME S_AXI, DATA_WIDTH 64, PROTOCOL AXI4, FREQ_HZ 100000000, ID_WIDTH 0, ADDR_WIDTH 32, AWUSER_WIDTH 0, ARUSER_WIDTH 0, WUSER_WIDTH 0, RUSER_WIDTH 0, BUSER_WIDTH 0, READ_WRITE_MODE WRITE_ONLY, HAS_BURST 1, HAS_LOCK 1, HAS_PROT 1, HAS_CACHE 1, HAS_QOS 1, HAS_REGION 1, HAS_WSTRB 1, HAS_BRESP 1, HAS_RRESP 0, SUPPORTS_NARROW_BURST 0, NUM_READ_OUTSTANDING 8, NUM_WRITE_OUTSTANDING 8, MAX_BURST_LENGTH 32, PHASE 0.0, CLK_DOMAIN drone_block_design_processing_system7_0_0_FCLK_CLK0, NUM_READ_THREADS 1, NUM_WRITE_THREADS 1, RUSER_BITS_PER_BYTE 0, WUSER_BITS_PER_BYTE 0, INSERT_VIP 0" *) input [31:0]s_axi_awaddr;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWLEN" *) input [7:0]s_axi_awlen;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWSIZE" *) input [2:0]s_axi_awsize;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWBURST" *) input [1:0]s_axi_awburst;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWLOCK" *) input [0:0]s_axi_awlock;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWCACHE" *) input [3:0]s_axi_awcache;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWPROT" *) input [2:0]s_axi_awprot;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWREGION" *) input [3:0]s_axi_awregion;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWQOS" *) input [3:0]s_axi_awqos;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWVALID" *) input s_axi_awvalid;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWREADY" *) output s_axi_awready;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WDATA" *) input [63:0]s_axi_wdata;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WSTRB" *) input [7:0]s_axi_wstrb;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WLAST" *) input s_axi_wlast;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WVALID" *) input s_axi_wvalid;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WREADY" *) output s_axi_wready;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI BRESP" *) output [1:0]s_axi_bresp;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI BVALID" *) output s_axi_bvalid;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI BREADY" *) input s_axi_bready;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI AWADDR" *) (* X_INTERFACE_MODE = "master" *) (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME M_AXI, DATA_WIDTH 64, PROTOCOL AXI3, FREQ_HZ 100000000, ID_WIDTH 0, ADDR_WIDTH 32, AWUSER_WIDTH 0, ARUSER_WIDTH 0, WUSER_WIDTH 0, RUSER_WIDTH 0, BUSER_WIDTH 0, READ_WRITE_MODE WRITE_ONLY, HAS_BURST 0, HAS_LOCK 0, HAS_PROT 1, HAS_CACHE 1, HAS_QOS 0, HAS_REGION 0, HAS_WSTRB 1, HAS_BRESP 1, HAS_RRESP 0, SUPPORTS_NARROW_BURST 0, NUM_READ_OUTSTANDING 8, NUM_WRITE_OUTSTANDING 8, MAX_BURST_LENGTH 16, PHASE 0.0, CLK_DOMAIN drone_block_design_processing_system7_0_0_FCLK_CLK0, NUM_READ_THREADS 1, NUM_WRITE_THREADS 1, RUSER_BITS_PER_BYTE 0, WUSER_BITS_PER_BYTE 0, INSERT_VIP 0" *) output [31:0]m_axi_awaddr;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI AWLEN" *) output [3:0]m_axi_awlen;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI AWSIZE" *) output [2:0]m_axi_awsize;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI AWBURST" *) output [1:0]m_axi_awburst;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI AWLOCK" *) output [1:0]m_axi_awlock;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI AWCACHE" *) output [3:0]m_axi_awcache;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI AWPROT" *) output [2:0]m_axi_awprot;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI AWQOS" *) output [3:0]m_axi_awqos;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI AWVALID" *) output m_axi_awvalid;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI AWREADY" *) input m_axi_awready;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI WDATA" *) output [63:0]m_axi_wdata;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI WSTRB" *) output [7:0]m_axi_wstrb;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI WLAST" *) output m_axi_wlast;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI WVALID" *) output m_axi_wvalid;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI WREADY" *) input m_axi_wready;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI BRESP" *) input [1:0]m_axi_bresp;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI BVALID" *) input m_axi_bvalid;
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 M_AXI BREADY" *) output m_axi_bready;

  wire \<const0> ;
  wire aclk;
  wire aresetn;
  wire [31:0]m_axi_awaddr;
  wire [1:0]m_axi_awburst;
  wire [3:0]m_axi_awcache;
  wire [3:0]m_axi_awlen;
  wire [0:0]\^m_axi_awlock ;
  wire [2:0]m_axi_awprot;
  wire [3:0]m_axi_awqos;
  wire m_axi_awready;
  wire [2:0]m_axi_awsize;
  wire m_axi_awvalid;
  wire m_axi_bready;
  wire [1:0]m_axi_bresp;
  wire m_axi_bvalid;
  wire [63:0]m_axi_wdata;
  wire m_axi_wlast;
  wire m_axi_wready;
  wire [7:0]m_axi_wstrb;
  wire m_axi_wvalid;
  wire [31:0]s_axi_awaddr;
  wire [1:0]s_axi_awburst;
  wire [3:0]s_axi_awcache;
  wire [7:0]s_axi_awlen;
  wire [0:0]s_axi_awlock;
  wire [2:0]s_axi_awprot;
  wire [3:0]s_axi_awqos;
  wire s_axi_awready;
  wire [2:0]s_axi_awsize;
  wire s_axi_awvalid;
  wire s_axi_bready;
  wire [1:0]s_axi_bresp;
  wire s_axi_bvalid;
  wire [63:0]s_axi_wdata;
  wire s_axi_wready;
  wire [7:0]s_axi_wstrb;
  wire s_axi_wvalid;
  wire NLW_inst_m_axi_arvalid_UNCONNECTED;
  wire NLW_inst_m_axi_rready_UNCONNECTED;
  wire NLW_inst_s_axi_arready_UNCONNECTED;
  wire NLW_inst_s_axi_rlast_UNCONNECTED;
  wire NLW_inst_s_axi_rvalid_UNCONNECTED;
  wire [31:0]NLW_inst_m_axi_araddr_UNCONNECTED;
  wire [1:0]NLW_inst_m_axi_arburst_UNCONNECTED;
  wire [3:0]NLW_inst_m_axi_arcache_UNCONNECTED;
  wire [0:0]NLW_inst_m_axi_arid_UNCONNECTED;
  wire [3:0]NLW_inst_m_axi_arlen_UNCONNECTED;
  wire [1:0]NLW_inst_m_axi_arlock_UNCONNECTED;
  wire [2:0]NLW_inst_m_axi_arprot_UNCONNECTED;
  wire [3:0]NLW_inst_m_axi_arqos_UNCONNECTED;
  wire [3:0]NLW_inst_m_axi_arregion_UNCONNECTED;
  wire [2:0]NLW_inst_m_axi_arsize_UNCONNECTED;
  wire [0:0]NLW_inst_m_axi_aruser_UNCONNECTED;
  wire [0:0]NLW_inst_m_axi_awid_UNCONNECTED;
  wire [1:1]NLW_inst_m_axi_awlock_UNCONNECTED;
  wire [3:0]NLW_inst_m_axi_awregion_UNCONNECTED;
  wire [0:0]NLW_inst_m_axi_awuser_UNCONNECTED;
  wire [0:0]NLW_inst_m_axi_wid_UNCONNECTED;
  wire [0:0]NLW_inst_m_axi_wuser_UNCONNECTED;
  wire [0:0]NLW_inst_s_axi_bid_UNCONNECTED;
  wire [0:0]NLW_inst_s_axi_buser_UNCONNECTED;
  wire [63:0]NLW_inst_s_axi_rdata_UNCONNECTED;
  wire [0:0]NLW_inst_s_axi_rid_UNCONNECTED;
  wire [1:0]NLW_inst_s_axi_rresp_UNCONNECTED;
  wire [0:0]NLW_inst_s_axi_ruser_UNCONNECTED;

  assign m_axi_awlock[1] = \<const0> ;
  assign m_axi_awlock[0] = \^m_axi_awlock [0];
  GND GND
       (.G(\<const0> ));
  (* C_AXI_ADDR_WIDTH = "32" *) 
  (* C_AXI_ARUSER_WIDTH = "1" *) 
  (* C_AXI_AWUSER_WIDTH = "1" *) 
  (* C_AXI_BUSER_WIDTH = "1" *) 
  (* C_AXI_DATA_WIDTH = "64" *) 
  (* C_AXI_ID_WIDTH = "1" *) 
  (* C_AXI_RUSER_WIDTH = "1" *) 
  (* C_AXI_SUPPORTS_READ = "0" *) 
  (* C_AXI_SUPPORTS_USER_SIGNALS = "0" *) 
  (* C_AXI_SUPPORTS_WRITE = "1" *) 
  (* C_AXI_WUSER_WIDTH = "1" *) 
  (* C_FAMILY = "zynq" *) 
  (* C_IGNORE_ID = "1" *) 
  (* C_M_AXI_PROTOCOL = "1" *) 
  (* C_S_AXI_PROTOCOL = "0" *) 
  (* C_TRANSLATION_MODE = "0" *) 
  (* P_AXI3 = "1" *) 
  (* P_AXI4 = "0" *) 
  (* P_AXILITE = "2" *) 
  (* P_AXILITE_SIZE = "3'b011" *) 
  (* P_CONVERSION = "2" *) 
  (* P_DECERR = "2'b11" *) 
  (* P_INCR = "2'b01" *) 
  (* P_PROTECTION = "1" *) 
  (* P_SLVERR = "2'b10" *) 
  (* downgradeipidentifiedwarnings = "yes" *) 
  decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_axi_protocol_converter_v2_1_37_axi_protocol_converter inst
       (.aclk(aclk),
        .aresetn(aresetn),
        .m_axi_araddr(NLW_inst_m_axi_araddr_UNCONNECTED[31:0]),
        .m_axi_arburst(NLW_inst_m_axi_arburst_UNCONNECTED[1:0]),
        .m_axi_arcache(NLW_inst_m_axi_arcache_UNCONNECTED[3:0]),
        .m_axi_arid(NLW_inst_m_axi_arid_UNCONNECTED[0]),
        .m_axi_arlen(NLW_inst_m_axi_arlen_UNCONNECTED[3:0]),
        .m_axi_arlock(NLW_inst_m_axi_arlock_UNCONNECTED[1:0]),
        .m_axi_arprot(NLW_inst_m_axi_arprot_UNCONNECTED[2:0]),
        .m_axi_arqos(NLW_inst_m_axi_arqos_UNCONNECTED[3:0]),
        .m_axi_arready(1'b0),
        .m_axi_arregion(NLW_inst_m_axi_arregion_UNCONNECTED[3:0]),
        .m_axi_arsize(NLW_inst_m_axi_arsize_UNCONNECTED[2:0]),
        .m_axi_aruser(NLW_inst_m_axi_aruser_UNCONNECTED[0]),
        .m_axi_arvalid(NLW_inst_m_axi_arvalid_UNCONNECTED),
        .m_axi_awaddr(m_axi_awaddr),
        .m_axi_awburst(m_axi_awburst),
        .m_axi_awcache(m_axi_awcache),
        .m_axi_awid(NLW_inst_m_axi_awid_UNCONNECTED[0]),
        .m_axi_awlen(m_axi_awlen),
        .m_axi_awlock({NLW_inst_m_axi_awlock_UNCONNECTED[1],\^m_axi_awlock }),
        .m_axi_awprot(m_axi_awprot),
        .m_axi_awqos(m_axi_awqos),
        .m_axi_awready(m_axi_awready),
        .m_axi_awregion(NLW_inst_m_axi_awregion_UNCONNECTED[3:0]),
        .m_axi_awsize(m_axi_awsize),
        .m_axi_awuser(NLW_inst_m_axi_awuser_UNCONNECTED[0]),
        .m_axi_awvalid(m_axi_awvalid),
        .m_axi_bid(1'b0),
        .m_axi_bready(m_axi_bready),
        .m_axi_bresp(m_axi_bresp),
        .m_axi_buser(1'b0),
        .m_axi_bvalid(m_axi_bvalid),
        .m_axi_rdata({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .m_axi_rid(1'b0),
        .m_axi_rlast(1'b1),
        .m_axi_rready(NLW_inst_m_axi_rready_UNCONNECTED),
        .m_axi_rresp({1'b0,1'b0}),
        .m_axi_ruser(1'b0),
        .m_axi_rvalid(1'b0),
        .m_axi_wdata(m_axi_wdata),
        .m_axi_wid(NLW_inst_m_axi_wid_UNCONNECTED[0]),
        .m_axi_wlast(m_axi_wlast),
        .m_axi_wready(m_axi_wready),
        .m_axi_wstrb(m_axi_wstrb),
        .m_axi_wuser(NLW_inst_m_axi_wuser_UNCONNECTED[0]),
        .m_axi_wvalid(m_axi_wvalid),
        .s_axi_araddr({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .s_axi_arburst({1'b0,1'b1}),
        .s_axi_arcache({1'b0,1'b0,1'b0,1'b0}),
        .s_axi_arid(1'b0),
        .s_axi_arlen({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .s_axi_arlock(1'b0),
        .s_axi_arprot({1'b0,1'b0,1'b0}),
        .s_axi_arqos({1'b0,1'b0,1'b0,1'b0}),
        .s_axi_arready(NLW_inst_s_axi_arready_UNCONNECTED),
        .s_axi_arregion({1'b0,1'b0,1'b0,1'b0}),
        .s_axi_arsize({1'b0,1'b0,1'b0}),
        .s_axi_aruser(1'b0),
        .s_axi_arvalid(1'b0),
        .s_axi_awaddr(s_axi_awaddr),
        .s_axi_awburst(s_axi_awburst),
        .s_axi_awcache(s_axi_awcache),
        .s_axi_awid(1'b0),
        .s_axi_awlen({1'b0,1'b0,1'b0,1'b0,s_axi_awlen[3:0]}),
        .s_axi_awlock(s_axi_awlock),
        .s_axi_awprot(s_axi_awprot),
        .s_axi_awqos(s_axi_awqos),
        .s_axi_awready(s_axi_awready),
        .s_axi_awregion({1'b0,1'b0,1'b0,1'b0}),
        .s_axi_awsize(s_axi_awsize),
        .s_axi_awuser(1'b0),
        .s_axi_awvalid(s_axi_awvalid),
        .s_axi_bid(NLW_inst_s_axi_bid_UNCONNECTED[0]),
        .s_axi_bready(s_axi_bready),
        .s_axi_bresp(s_axi_bresp),
        .s_axi_buser(NLW_inst_s_axi_buser_UNCONNECTED[0]),
        .s_axi_bvalid(s_axi_bvalid),
        .s_axi_rdata(NLW_inst_s_axi_rdata_UNCONNECTED[63:0]),
        .s_axi_rid(NLW_inst_s_axi_rid_UNCONNECTED[0]),
        .s_axi_rlast(NLW_inst_s_axi_rlast_UNCONNECTED),
        .s_axi_rready(1'b0),
        .s_axi_rresp(NLW_inst_s_axi_rresp_UNCONNECTED[1:0]),
        .s_axi_ruser(NLW_inst_s_axi_ruser_UNCONNECTED[0]),
        .s_axi_rvalid(NLW_inst_s_axi_rvalid_UNCONNECTED),
        .s_axi_wdata(s_axi_wdata),
        .s_axi_wid(1'b0),
        .s_axi_wlast(1'b0),
        .s_axi_wready(s_axi_wready),
        .s_axi_wstrb(s_axi_wstrb),
        .s_axi_wuser(1'b0),
        .s_axi_wvalid(s_axi_wvalid));
endmodule

(* DEF_VAL = "1'b0" *) (* DEST_SYNC_FF = "2" *) (* INIT_SYNC_FF = "0" *) 
(* INV_DEF_VAL = "1'b1" *) (* RST_ACTIVE_HIGH = "1" *) (* VERSION = "0" *) 
(* XPM_MODULE = "TRUE" *) (* is_du_within_envelope = "true" *) (* keep_hierarchy = "soft" *) 
(* xpm_cdc = "ASYNC_RST" *) 
module decalper_eb_ot_sdeen_pot_pi_dehcac_xnilix_xpm_cdc_async_rst
   (src_arst,
    dest_clk,
    dest_arst);
  input src_arst;
  input dest_clk;
  output dest_arst;

  (* RTL_KEEP = "true" *) (* async_reg = "true" *) (* xpm_cdc = "ASYNC_RST" *) wire [1:0]arststages_ff;
  wire dest_clk;
  wire src_arst;

  assign dest_arst = arststages_ff[1];
  (* ASYNC_REG *) 
  (* KEEP = "true" *) 
  (* XPM_CDC = "ASYNC_RST" *) 
  FDPE #(
    .INIT(1'b0)) 
    \arststages_ff_reg[0] 
       (.C(dest_clk),
        .CE(1'b1),
        .D(1'b0),
        .PRE(src_arst),
        .Q(arststages_ff[0]));
  (* ASYNC_REG *) 
  (* KEEP = "true" *) 
  (* XPM_CDC = "ASYNC_RST" *) 
  FDPE #(
    .INIT(1'b0)) 
    \arststages_ff_reg[1] 
       (.C(dest_clk),
        .CE(1'b1),
        .D(arststages_ff[0]),
        .PRE(src_arst),
        .Q(arststages_ff[1]));
endmodule
`pragma protect begin_protected
`pragma protect version = 1
`pragma protect encrypt_agent = "XILINX"
`pragma protect encrypt_agent_info = "Xilinx Encryption Tool 2025.2"
`pragma protect key_keyowner="Synopsys", key_keyname="SNPS-VCS-RSA-2", key_method="rsa"
`pragma protect encoding = (enctype="BASE64", line_length=76, bytes=128)
`pragma protect key_block
UU0HctCtrDGjqiFgNj8KUV1CNrtLH1fzvWozH/S7aVj0RSc24esnSs0ybsApJYbLPSCW6MJRxlk8
TZTBIGKXHEs9iSJrHyeb7Q9LsfbX2O77j94jiFzmN8lM/LIVA6RCDBtX2LtKWWw0Ex0IvwdPy+Mg
2z4iCfTMzyceiAZWkhE=

`pragma protect key_keyowner="Aldec", key_keyname="ALDEC15_001", key_method="rsa"
`pragma protect encoding = (enctype="BASE64", line_length=76, bytes=256)
`pragma protect key_block
GF0Vw/gqBrc9IHG5aASlKQHzVjMUtBIwjnrAUquexOCvx+SSWyZN88WoE2YOio8l2Mng8jmA3ELb
iVwbk5kPsSQid3iLelRIejTGTCNP7ErmhAyw9N/gInxZrkBgF+99fwCp/qSFsRz+GkpjXlmNPLal
1m+CmI2mtQjH/zDmulZq9kFS9URMU7E3TrKSiNtdLMYc1ulwC3kFJ99geu/tuMfIrNOmA9KkJtnb
Zoy9fNs53bR+fUGBL5n7AwoO6cdU62PpktsyWXh1Gp6Ylf2HTT0CPMyzWbJQve0G4+iszllRawxG
r+FcAh4BuFpKqaFogcTloexA8MTZ9ICsGZkzkg==

`pragma protect key_keyowner="Mentor Graphics Corporation", key_keyname="MGC-VELOCE-RSA", key_method="rsa"
`pragma protect encoding = (enctype="BASE64", line_length=76, bytes=128)
`pragma protect key_block
Hzytw/FfXpsPrE5ZowzcEV+nwakl1BirWDR+Iseu9nWPYk6Otw/UyzdfMGdUJQcXxjn8eODJUMPS
SLvHyIbu8M+iaMMz4+lNG/o0csNo8MO67HX9fxa4xkVOaSOTCzBVfRk3cjnK+OAXlJEZO2/F0Im7
evCVwWE8mv0p9yv9NZA=

`pragma protect key_keyowner="Mentor Graphics Corporation", key_keyname="MGC-VERIF-SIM-RSA-2", key_method="rsa"
`pragma protect encoding = (enctype="BASE64", line_length=76, bytes=256)
`pragma protect key_block
aYTxAf85PVmpAktzX89uf9AJXAUs8FLk2gaAmaPtMQhfYN72ydFe5GcOlR9/W705GnhW+LSDUX2b
XQnSvIzmqRMwIqE2sgix0W4aZDvptNpP2y+gttAzQaOhAd12INExGFaZxKro7f/cey7YiwGKPPah
zcBWMoHI2bIhFDe04i/Jt1MdciCe1haFyhwBCett8eV6Laia/DlHOXxqH2bLukgGZp5p2EYoM0T8
WwuwxJ3X0IIphS/uP6nXSuuuMQcAplYzcG4PLCMpn2Lo3HwmwSo5w+0N1NFI5LYfb6ZrdTXjRH+j
oHZlteBZzQ+4jNx7/nPPCnuUB8IFMROek8y3aQ==

`pragma protect key_keyowner="Real Intent", key_keyname="RI-RSA-KEY-1", key_method="rsa"
`pragma protect encoding = (enctype="BASE64", line_length=76, bytes=256)
`pragma protect key_block
e6jDiYnzLTYk/3jC49X3YNnxEmaFBYGO/cl88hMTKYq1FltlAtsDFs47xPVxcrXJmXB6FiDcQKgy
Zcri+H61avSebr0yHZ1uigtfwqLvcivJwyCmMK1zZ+tk95pu+v8wQUekejQwCfm8d4EwcPtFRBCP
VuiAB7kH68VA/rKSNW/L3Ck+PVdkE6HHJnrneJm4Aial7Xm5QOsroJRJU/ObInH0MO+tgwAysCdd
6eCmjEBFQGTjmThY8W79EF9AQGGRTMTJSajCB65vB7j4uMsw7y2m2q5T1cf5FapbNOa5qVGM3ltu
WzPHL8ffpwsn/Um4FxL0m2OELCU3vijgWPxyYg==

`pragma protect key_keyowner="Xilinx", key_keyname="xilinxt_2025.1-2029.x", key_method="rsa"
`pragma protect encoding = (enctype="BASE64", line_length=76, bytes=256)
`pragma protect key_block
W4uYHM01gGeA2MU+ib2L/ExIRZJnY4G/4/BNSFnBkDMClm5bxdPZWGZhCUejE4JXBUBzvBBii0hv
o/qn9snazl844XvvPfn0rjgdMjBDDTUc14EhQ+t9LtnZFAV+z3wAIKGQaUOt5C451j/28rPyPkS0
kBiQMKRYL8V8HYzz8PJCw/2pMZh5nAGYlHVN7x7BRfHg/eGLL9Vxje7mRSIq9oPfHNxp9KvTPnEz
BAbFFeUiH6gtQHgv3loUdp74IXW+8+uJHlh0BbE4crWkB23UetPNvBTz30q+iGUe+Uy9cDako55V
AVXIMgciLrWVPF+qY5b7zySQkB4Xsfj+udkVyA==

`pragma protect key_keyowner="Metrics Technologies Inc.", key_keyname="DSim", key_method="rsa"
`pragma protect encoding = (enctype="BASE64", line_length=76, bytes=256)
`pragma protect key_block
R0MJeGCQpSjYsGBWKKr56ZJi8ovYpLtniBxpCnrQicvQybY+fnPA8Daj6MXdCf3qwLF8yF5WCJ8s
qgsZvXSLz7hwsKVEId08i3cpwMDSnKdPTNXjuKS2h7UKOlcr6QZ5j31qcO2XbyCffpn/pAXTmv3a
wywj0bLNK61+JY8v+VTzUKzR370hK34Ryuts+hg1InhuHxLuVnu52lVOpk/PYUaA+w7ORS7AIzBm
Ic2Gs+gCO56TT/kHzEdPXDOhyRk/LG0ir7xXNq7VYILxVh4t9QTZ+TIjutFAhElz9ceEjJ95QYy+
i58LiAOmyF9ID0yxSSYM4KQAF2bqt9kvgdWRhg==

`pragma protect key_keyowner="Atrenta", key_keyname="ATR-SG-RSA-1", key_method="rsa"
`pragma protect encoding = (enctype="BASE64", line_length=76, bytes=384)
`pragma protect key_block
piBTg4FhL4gV7WxO2j/dIDXpMS0DVV+BCPbz6qHH74TfGEKWiiBMU6gK+ZbplwJNS8NHNyEzAlya
r4wgVpBFLdWysNz1JTSjKKJCO9JEQN5/H5jfiaYLOSRwE+N3Opc54BvT85yu1V+zTS+2aJj4AQ/f
gjyVCtr2A8YVv2zEjqFuQcYlcSxHTEk5eig4u36hHgzGJsmifFlP0OtE2NeoOMzFbBJe4LR9f1Ac
XQfLq8HilNwnOz4EYZGL9iJymjQ63NwSYfWcRjHVPPJXQFZSrWlI6V5kkz1/IDnPuelueoAKOk5K
OAAeaRjYDKgXhfse4B1Cy+u9f08zryJez9v+yfA14jVDkQQJp6a0qHJYuemefEFrmwJxSLUqG+Xq
QDK6/emEA9ZXoln0PNQyFzaEVDeFDZBn8LZi5SGL6f+TpO0acfI2jxa5+vCQHX/boxpyVjtxPh0W
Xjk7+E7CKFDmE6T/ZNnn7MRpaG1g4A2TEvSqCSRRnPprcg/+bRR6T6Sy

`pragma protect key_keyowner="Cadence Design Systems.", key_keyname="CDS_RSA_KEY_VER_1", key_method="rsa"
`pragma protect encoding = (enctype="BASE64", line_length=76, bytes=256)
`pragma protect key_block
GlYhuN+XgK/dKipYGy0F51EWCsMzdTtEw7DUl9GCeVeyU6B0qQxd4o+WGLqPzleHUcbSjTY0Zsbn
PYVk3cx1yet4akcLytYAGFXC4n/Xi+1UqMz5TGn6+YQTvRIQ3rDpVCwwETOtxY9exyURa9vrZwN6
wg8aS7eaMRDPPrD9XOy8sQT0WrdKizBToFy2xoVRXceycyYYY7TdZikow1sCVE5Dsq8WQ5SRprGB
6XOvNlQnaIlUCVafx8nFv91VsM31btEViBrUpTqFHJAuoebt0ZL+JlrQ5nOk7XQnw6AQ+0ZlOKba
q3Ttg2CqLMLHVI+1yNiz+OEKhmPV1D5J7vlPQQ==

`pragma protect key_keyowner="Synplicity", key_keyname="SYNP15_1", key_method="rsa"
`pragma protect encoding = (enctype="BASE64", line_length=76, bytes=256)
`pragma protect key_block
2gbN0jz/o58BxZjM7+eT+qN7Q3qHE0g1JsI7dvdgaVydBYqQVWbzuiZYLMAHv8yrsn9b32oHcBSE
0o5Cui6GiD7neKU4AljBAlKAaN9vmM7TfUunNvBpRwv61T0jxsnbQPWfLrtpbTXbXa9k+COT+cqb
xPXfz1KFKZR+jUVQfqg3k9yE8k42Qekbv3kD1KU/qey8yzrOiZWk3YSqYVf+xtUpOvJY52CMhroS
XNjVVkBPUu8Qp/8HAzxqzWi+9FMbOuRKapPdzyPMn/9u5V3oDa03Jlbl/wNvQRAMkkI4MR0Z6Fef
acPXE4lO4yrbdCI+/JWNiFnMhbPxxOqB2cgi5g==

`pragma protect key_keyowner="Mentor Graphics Corporation", key_keyname="MGC-PREC-RSA", key_method="rsa"
`pragma protect encoding = (enctype="BASE64", line_length=76, bytes=256)
`pragma protect key_block
ijvB9ebv8UTsfEBOdwLX29OhkfU+M38mGG3GBCgYR1J/bZmxD6jFCxoFCEm1aKFgD1oURupMHfs1
c3MOeOmJ+miekD3bzrkO2GpRCnMbhKovUm5w9Qm7OnK1B25OU6+Xq1Ykk4tIi1xMOMYX8YKOrSrC
twPgnJ2VHr4FFKQ+p5YO7BYb6KtJrf3+2JKYjVPpp3gkR5SZklV/ugbHgXnKTC8NtjSnys5yM8fs
hXOpMWgzLJxxPm595q7fFP3rHvMyw7H7unYraHK+0uc9zTFZ4LHWuOQvc3TRUEmRmJmaag8nwld1
2cnhyhbuZqsuwb5+2W6amIYGSDb8gPS45qwzBg==

`pragma protect data_method = "AES128-CBC"
`pragma protect encoding = (enctype = "BASE64", line_length = 76, bytes = 71856)
`pragma protect data_block
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
`pragma protect end_protected
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
