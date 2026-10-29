// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2025 Advanced Micro Devices, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2025.2 (win64) Build 6299465 Fri Nov 14 19:35:11 GMT 2025
// Date        : Sun Jun 21 18:08:49 2026
// Host        : Michel-PC running 64-bit major release  (build 9200)
// Command     : write_verilog -force -mode funcsim -rename_top drone_block_design_axi_mem_intercon_imp_auto_pc_0 -prefix
//               drone_block_design_axi_mem_intercon_imp_auto_pc_0_ drone_block_design_axi_mem_intercon_imp_auto_pc_0_sim_netlist.v
// Design      : drone_block_design_axi_mem_intercon_imp_auto_pc_0
// Purpose     : This verilog netlist is a functional simulation representation of the design and should not be modified
//               or synthesized. This netlist cannot be used for SDF annotated simulation.
// Device      : xc7z020clg400-1
// --------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

module drone_block_design_axi_mem_intercon_imp_auto_pc_0_axi_data_fifo_v2_1_36_axic_fifo
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
  drone_block_design_axi_mem_intercon_imp_auto_pc_0_axi_data_fifo_v2_1_36_fifo_gen inst
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

module drone_block_design_axi_mem_intercon_imp_auto_pc_0_axi_data_fifo_v2_1_36_fifo_gen
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
  drone_block_design_axi_mem_intercon_imp_auto_pc_0_fifo_generator_v13_2_14 fifo_gen_inst
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

module drone_block_design_axi_mem_intercon_imp_auto_pc_0_axi_protocol_converter_v2_1_37_a_axi3_conv
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
  drone_block_design_axi_mem_intercon_imp_auto_pc_0_axi_data_fifo_v2_1_36_axic_fifo \USE_BURSTS.cmd_queue 
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

module drone_block_design_axi_mem_intercon_imp_auto_pc_0_axi_protocol_converter_v2_1_37_axi3_conv
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

  drone_block_design_axi_mem_intercon_imp_auto_pc_0_axi_protocol_converter_v2_1_37_a_axi3_conv \USE_WRITE.write_addr_inst 
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
  drone_block_design_axi_mem_intercon_imp_auto_pc_0_axi_protocol_converter_v2_1_37_w_axi3_conv \USE_WRITE.write_data_inst 
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
module drone_block_design_axi_mem_intercon_imp_auto_pc_0_axi_protocol_converter_v2_1_37_axi_protocol_converter
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
  drone_block_design_axi_mem_intercon_imp_auto_pc_0_axi_protocol_converter_v2_1_37_axi3_conv \gen_axi4_axi3.axi3_conv_inst 
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

module drone_block_design_axi_mem_intercon_imp_auto_pc_0_axi_protocol_converter_v2_1_37_w_axi3_conv
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
module drone_block_design_axi_mem_intercon_imp_auto_pc_0
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
  drone_block_design_axi_mem_intercon_imp_auto_pc_0_axi_protocol_converter_v2_1_37_axi_protocol_converter inst
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
module drone_block_design_axi_mem_intercon_imp_auto_pc_0_xpm_cdc_async_rst
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
`pragma protect encoding = (enctype = "BASE64", line_length = 76, bytes = 72080)
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
