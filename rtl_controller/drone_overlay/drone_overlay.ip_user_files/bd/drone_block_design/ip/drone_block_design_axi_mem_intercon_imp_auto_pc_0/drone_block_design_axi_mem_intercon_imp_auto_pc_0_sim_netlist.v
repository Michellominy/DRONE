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
