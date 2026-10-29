vlib modelsim_lib/work
vlib modelsim_lib/msim

vlib modelsim_lib/msim/xilinx_vip
vlib modelsim_lib/msim/xpm
vlib modelsim_lib/msim/axi_infrastructure_v1_1_0
vlib modelsim_lib/msim/axi_vip_v1_1_22
vlib modelsim_lib/msim/processing_system7_vip_v1_0_24
vlib modelsim_lib/msim/xil_defaultlib
vlib modelsim_lib/msim/generic_baseblocks_v2_1_2
vlib modelsim_lib/msim/axi_register_slice_v2_1_36
vlib modelsim_lib/msim/fifo_generator_v13_2_14
vlib modelsim_lib/msim/axi_data_fifo_v2_1_36
vlib modelsim_lib/msim/axi_crossbar_v2_1_38
vlib modelsim_lib/msim/proc_sys_reset_v5_0_17
vlib modelsim_lib/msim/xbip_utils_v3_0_15
vlib modelsim_lib/msim/axi_utils_v2_0_11
vlib modelsim_lib/msim/xbip_pipe_v3_0_11
vlib modelsim_lib/msim/xbip_dsp48_wrapper_v3_0_7
vlib modelsim_lib/msim/mult_gen_v12_0_24
vlib modelsim_lib/msim/floating_point_v7_1_21
vlib modelsim_lib/msim/xlconstant_v1_1_10
vlib modelsim_lib/msim/axi_protocol_converter_v2_1_37
vlib modelsim_lib/msim/axi_clock_converter_v2_1_35
vlib modelsim_lib/msim/blk_mem_gen_v8_4_12
vlib modelsim_lib/msim/axi_dwidth_converter_v2_1_37

vmap xilinx_vip modelsim_lib/msim/xilinx_vip
vmap xpm modelsim_lib/msim/xpm
vmap axi_infrastructure_v1_1_0 modelsim_lib/msim/axi_infrastructure_v1_1_0
vmap axi_vip_v1_1_22 modelsim_lib/msim/axi_vip_v1_1_22
vmap processing_system7_vip_v1_0_24 modelsim_lib/msim/processing_system7_vip_v1_0_24
vmap xil_defaultlib modelsim_lib/msim/xil_defaultlib
vmap generic_baseblocks_v2_1_2 modelsim_lib/msim/generic_baseblocks_v2_1_2
vmap axi_register_slice_v2_1_36 modelsim_lib/msim/axi_register_slice_v2_1_36
vmap fifo_generator_v13_2_14 modelsim_lib/msim/fifo_generator_v13_2_14
vmap axi_data_fifo_v2_1_36 modelsim_lib/msim/axi_data_fifo_v2_1_36
vmap axi_crossbar_v2_1_38 modelsim_lib/msim/axi_crossbar_v2_1_38
vmap proc_sys_reset_v5_0_17 modelsim_lib/msim/proc_sys_reset_v5_0_17
vmap xbip_utils_v3_0_15 modelsim_lib/msim/xbip_utils_v3_0_15
vmap axi_utils_v2_0_11 modelsim_lib/msim/axi_utils_v2_0_11
vmap xbip_pipe_v3_0_11 modelsim_lib/msim/xbip_pipe_v3_0_11
vmap xbip_dsp48_wrapper_v3_0_7 modelsim_lib/msim/xbip_dsp48_wrapper_v3_0_7
vmap mult_gen_v12_0_24 modelsim_lib/msim/mult_gen_v12_0_24
vmap floating_point_v7_1_21 modelsim_lib/msim/floating_point_v7_1_21
vmap xlconstant_v1_1_10 modelsim_lib/msim/xlconstant_v1_1_10
vmap axi_protocol_converter_v2_1_37 modelsim_lib/msim/axi_protocol_converter_v2_1_37
vmap axi_clock_converter_v2_1_35 modelsim_lib/msim/axi_clock_converter_v2_1_35
vmap blk_mem_gen_v8_4_12 modelsim_lib/msim/blk_mem_gen_v8_4_12
vmap axi_dwidth_converter_v2_1_37 modelsim_lib/msim/axi_dwidth_converter_v2_1_37

vlog -work xilinx_vip  -incr -mfcu  -sv -L axi_vip_v1_1_22 -L processing_system7_vip_v1_0_24 -L xilinx_vip "+incdir+C:/AMDDesignTools/2025.2/Vivado/data/xilinx_vip/include" \
"C:/AMDDesignTools/2025.2/Vivado/data/xilinx_vip/hdl/axi4stream_vip_axi4streampc.sv" \
"C:/AMDDesignTools/2025.2/Vivado/data/xilinx_vip/hdl/axi_vip_axi4pc.sv" \
"C:/AMDDesignTools/2025.2/Vivado/data/xilinx_vip/hdl/xil_common_vip_pkg.sv" \
"C:/AMDDesignTools/2025.2/Vivado/data/xilinx_vip/hdl/axi4stream_vip_pkg.sv" \
"C:/AMDDesignTools/2025.2/Vivado/data/xilinx_vip/hdl/axi_vip_pkg.sv" \
"C:/AMDDesignTools/2025.2/Vivado/data/xilinx_vip/hdl/axi4stream_vip_if.sv" \
"C:/AMDDesignTools/2025.2/Vivado/data/xilinx_vip/hdl/axi_vip_if.sv" \
"C:/AMDDesignTools/2025.2/Vivado/data/xilinx_vip/hdl/clk_vip_if.sv" \
"C:/AMDDesignTools/2025.2/Vivado/data/xilinx_vip/hdl/rst_vip_if.sv" \

vlog -work xpm  -incr -mfcu  -sv -L axi_vip_v1_1_22 -L processing_system7_vip_v1_0_24 -L xilinx_vip "+incdir+../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/ec67/hdl" "+incdir+../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/9a25/hdl" "+incdir+../../../../../../../../../../../AMDDesignTools/2025.2/Vivado/data/rsb/busdef" "+incdir+C:/AMDDesignTools/2025.2/Vivado/data/xilinx_vip/include" \
"C:/AMDDesignTools/2025.2/Vivado/data/ip/xpm/xpm_cdc/hdl/xpm_cdc.sv" \
"C:/AMDDesignTools/2025.2/Vivado/data/ip/xpm/xpm_memory/hdl/xpm_memory.sv" \

vcom -work xpm  -93  \
"C:/AMDDesignTools/2025.2/Vivado/data/ip/xpm/xpm_VCOMP.vhd" \

vlog -work axi_infrastructure_v1_1_0  -incr -mfcu  "+incdir+../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/ec67/hdl" "+incdir+../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/9a25/hdl" "+incdir+../../../../../../../../../../../AMDDesignTools/2025.2/Vivado/data/rsb/busdef" "+incdir+C:/AMDDesignTools/2025.2/Vivado/data/xilinx_vip/include" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/ec67/hdl/axi_infrastructure_v1_1_vl_rfs.v" \

vlog -work axi_vip_v1_1_22  -incr -mfcu  -sv -L axi_vip_v1_1_22 -L processing_system7_vip_v1_0_24 -L xilinx_vip "+incdir+../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/ec67/hdl" "+incdir+../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/9a25/hdl" "+incdir+../../../../../../../../../../../AMDDesignTools/2025.2/Vivado/data/rsb/busdef" "+incdir+C:/AMDDesignTools/2025.2/Vivado/data/xilinx_vip/include" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/b16a/hdl/axi_vip_v1_1_vl_rfs.sv" \

vlog -work processing_system7_vip_v1_0_24  -incr -mfcu  -sv -L axi_vip_v1_1_22 -L processing_system7_vip_v1_0_24 -L xilinx_vip "+incdir+../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/ec67/hdl" "+incdir+../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/9a25/hdl" "+incdir+../../../../../../../../../../../AMDDesignTools/2025.2/Vivado/data/rsb/busdef" "+incdir+C:/AMDDesignTools/2025.2/Vivado/data/xilinx_vip/include" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/9a25/hdl/processing_system7_vip_v1_0_vl_rfs.sv" \

vlog -work xil_defaultlib  -incr -mfcu  "+incdir+../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/ec67/hdl" "+incdir+../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/9a25/hdl" "+incdir+../../../../../../../../../../../AMDDesignTools/2025.2/Vivado/data/rsb/busdef" "+incdir+C:/AMDDesignTools/2025.2/Vivado/data/xilinx_vip/include" \
"../../../bd/drone_block_design/ip/drone_block_design_processing_system7_0_0/sim/drone_block_design_processing_system7_0_0.v" \

vlog -work generic_baseblocks_v2_1_2  -incr -mfcu  "+incdir+../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/ec67/hdl" "+incdir+../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/9a25/hdl" "+incdir+../../../../../../../../../../../AMDDesignTools/2025.2/Vivado/data/rsb/busdef" "+incdir+C:/AMDDesignTools/2025.2/Vivado/data/xilinx_vip/include" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/0c28/hdl/generic_baseblocks_v2_1_vl_rfs.v" \

vlog -work axi_register_slice_v2_1_36  -incr -mfcu  "+incdir+../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/ec67/hdl" "+incdir+../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/9a25/hdl" "+incdir+../../../../../../../../../../../AMDDesignTools/2025.2/Vivado/data/rsb/busdef" "+incdir+C:/AMDDesignTools/2025.2/Vivado/data/xilinx_vip/include" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/bc4b/hdl/axi_register_slice_v2_1_vl_rfs.v" \

vlog -work fifo_generator_v13_2_14  -incr -mfcu  "+incdir+../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/ec67/hdl" "+incdir+../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/9a25/hdl" "+incdir+../../../../../../../../../../../AMDDesignTools/2025.2/Vivado/data/rsb/busdef" "+incdir+C:/AMDDesignTools/2025.2/Vivado/data/xilinx_vip/include" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/d654/simulation/fifo_generator_vlog_beh.v" \

vcom -work fifo_generator_v13_2_14  -93  \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/d654/hdl/fifo_generator_v13_2_rfs.vhd" \

vlog -work fifo_generator_v13_2_14  -incr -mfcu  "+incdir+../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/ec67/hdl" "+incdir+../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/9a25/hdl" "+incdir+../../../../../../../../../../../AMDDesignTools/2025.2/Vivado/data/rsb/busdef" "+incdir+C:/AMDDesignTools/2025.2/Vivado/data/xilinx_vip/include" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/d654/hdl/fifo_generator_v13_2_rfs.v" \

vlog -work axi_data_fifo_v2_1_36  -incr -mfcu  "+incdir+../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/ec67/hdl" "+incdir+../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/9a25/hdl" "+incdir+../../../../../../../../../../../AMDDesignTools/2025.2/Vivado/data/rsb/busdef" "+incdir+C:/AMDDesignTools/2025.2/Vivado/data/xilinx_vip/include" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/fb46/hdl/axi_data_fifo_v2_1_vl_rfs.v" \

vlog -work axi_crossbar_v2_1_38  -incr -mfcu  "+incdir+../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/ec67/hdl" "+incdir+../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/9a25/hdl" "+incdir+../../../../../../../../../../../AMDDesignTools/2025.2/Vivado/data/rsb/busdef" "+incdir+C:/AMDDesignTools/2025.2/Vivado/data/xilinx_vip/include" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/f084/hdl/axi_crossbar_v2_1_vl_rfs.v" \

vlog -work xil_defaultlib  -incr -mfcu  "+incdir+../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/ec67/hdl" "+incdir+../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/9a25/hdl" "+incdir+../../../../../../../../../../../AMDDesignTools/2025.2/Vivado/data/rsb/busdef" "+incdir+C:/AMDDesignTools/2025.2/Vivado/data/xilinx_vip/include" \
"../../../bd/drone_block_design/ip/drone_block_design_axi_mem_intercon_imp_xbar_0/sim/drone_block_design_axi_mem_intercon_imp_xbar_0.v" \

vcom -work proc_sys_reset_v5_0_17  -93  \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/9438/hdl/proc_sys_reset_v5_0_vh_rfs.vhd" \

vcom -work xil_defaultlib  -93  \
"../../../bd/drone_block_design/ip/drone_block_design_rst_ps7_0_100M_2/sim/drone_block_design_rst_ps7_0_100M_2.vhd" \
"../../../bd/drone_block_design/ipshared/6837/src/i2c_master.vhd" \
"../../../bd/drone_block_design/ipshared/6837/src/imu_axis.vhd" \
"../../../bd/drone_block_design/ip/drone_block_design_mpu6050_axis_ip_0_0/sim/drone_block_design_mpu6050_axis_ip_0_0.vhd" \

vcom -work xbip_utils_v3_0_15  -93  \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ip/drone_block_design_sensor_fusion_0_1/hdl/ip/sensor_fusion_faddfsub_32ns_32ns_32_5_full_dsp_1_ip/hdl/xbip_utils_v3_0_vh_rfs.vhd" \

vcom -work axi_utils_v2_0_11  -93  \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ip/drone_block_design_sensor_fusion_0_1/hdl/ip/sensor_fusion_faddfsub_32ns_32ns_32_5_full_dsp_1_ip/hdl/axi_utils_v2_0_vh_rfs.vhd" \

vcom -work xbip_pipe_v3_0_11  -93  \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ip/drone_block_design_sensor_fusion_0_1/hdl/ip/sensor_fusion_faddfsub_32ns_32ns_32_5_full_dsp_1_ip/hdl/xbip_pipe_v3_0_vh_rfs.vhd" \

vcom -work xbip_dsp48_wrapper_v3_0_7  -93  \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ip/drone_block_design_sensor_fusion_0_1/hdl/ip/sensor_fusion_faddfsub_32ns_32ns_32_5_full_dsp_1_ip/hdl/xbip_dsp48_wrapper_v3_0_vh_rfs.vhd" \

vcom -work mult_gen_v12_0_24  -93  \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ip/drone_block_design_sensor_fusion_0_1/hdl/ip/sensor_fusion_faddfsub_32ns_32ns_32_5_full_dsp_1_ip/hdl/mult_gen_v12_0_vh_rfs.vhd" \

vcom -work floating_point_v7_1_21  -93  \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ip/drone_block_design_sensor_fusion_0_1/hdl/ip/sensor_fusion_faddfsub_32ns_32ns_32_5_full_dsp_1_ip/hdl/floating_point_v7_1_vh_rfs.vhd" \

vcom -work xil_defaultlib  -93  \
"../../../bd/drone_block_design/ip/drone_block_design_sensor_fusion_0_1/hdl/ip/sensor_fusion_faddfsub_32ns_32ns_32_5_full_dsp_1_ip/sim/sensor_fusion_faddfsub_32ns_32ns_32_5_full_dsp_1_ip.vhd" \
"../../../bd/drone_block_design/ip/drone_block_design_sensor_fusion_0_1/hdl/ip/sensor_fusion_fcmp_32ns_32ns_1_2_no_dsp_1_ip/sim/sensor_fusion_fcmp_32ns_32ns_1_2_no_dsp_1_ip.vhd" \
"../../../bd/drone_block_design/ip/drone_block_design_sensor_fusion_0_1/hdl/ip/sensor_fusion_fdiv_32ns_32ns_32_16_no_dsp_1_ip/sim/sensor_fusion_fdiv_32ns_32ns_32_16_no_dsp_1_ip.vhd" \
"../../../bd/drone_block_design/ip/drone_block_design_sensor_fusion_0_1/hdl/ip/sensor_fusion_fmul_32ns_32ns_32_4_max_dsp_1_ip/sim/sensor_fusion_fmul_32ns_32ns_32_4_max_dsp_1_ip.vhd" \
"../../../bd/drone_block_design/ip/drone_block_design_sensor_fusion_0_1/hdl/ip/sensor_fusion_fpext_32ns_64_2_no_dsp_1_ip/sim/sensor_fusion_fpext_32ns_64_2_no_dsp_1_ip.vhd" \
"../../../bd/drone_block_design/ip/drone_block_design_sensor_fusion_0_1/hdl/ip/sensor_fusion_frsqrt_32ns_32ns_32_11_full_dsp_1_ip/sim/sensor_fusion_frsqrt_32ns_32ns_32_11_full_dsp_1_ip.vhd" \
"../../../bd/drone_block_design/ip/drone_block_design_sensor_fusion_0_1/hdl/ip/sensor_fusion_fsub_32ns_32ns_32_5_full_dsp_1_ip/sim/sensor_fusion_fsub_32ns_32ns_32_5_full_dsp_1_ip.vhd" \
"../../../bd/drone_block_design/ip/drone_block_design_sensor_fusion_0_1/hdl/ip/sensor_fusion_sitofp_32s_32_6_no_dsp_1_ip/sim/sensor_fusion_sitofp_32s_32_6_no_dsp_1_ip.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_atan2_cordic_float_s.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_atan2_generic_float_Pipeline_1.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_atan2_generic_float_s.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_bitselect_1ns_40ns_6ns_1_1_1.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_bitselect_1ns_55ns_6ns_1_1_1.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_ctlz_30_30_1_1.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_ctlz_32_32_1_1.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_ctlz_40_40_1_1.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_ctlz_55_55_1_1.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_faddfsub_32ns_32ns_32_5_full_dsp_1.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_fcmp_32ns_32ns_1_2_no_dsp_1.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_fdiv_32ns_32ns_32_16_no_dsp_1.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_flow_control_loop_pipe_sequential_init.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_fmul_32ns_32ns_32_4_max_dsp_1.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_fpext_32ns_64_2_no_dsp_1.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_frsqrt_32ns_32ns_32_11_full_dsp_1.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_fsub_32ns_32ns_32_5_full_dsp_1.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_generic_asin_float_Pipeline_1.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_generic_asin_float_Pipeline_1_cordic_apfixed_circ_table_arctan_128_ROM_AUTO_1R.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_generic_asin_float_s.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_generic_sincos_float_s.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_generic_sincos_float_s_second_order_float_cos_K0_ROM_1P_LUTRAM_1R.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_generic_sincos_float_s_second_order_float_cos_K1_ROM_1P_LUTRAM_1R.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_generic_sincos_float_s_second_order_float_cos_K2_ROM_1P_LUTRAM_1R.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_generic_sincos_float_s_second_order_float_sin_K0_ROM_1P_LUTRAM_1R.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_generic_sincos_float_s_second_order_float_sin_K1_ROM_1P_LUTRAM_1R.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_generic_sincos_float_s_second_order_float_sin_K2_ROM_1P_LUTRAM_1R.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_mul_15ns_13s_28_1_1.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_mul_15ns_14ns_29_1_1.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_mul_15ns_15ns_30_1_1.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_mul_15ns_15s_30_1_1.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_mul_22ns_21s_43_1_1.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_mul_22ns_22ns_44_1_1.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_mul_23s_22ns_45_1_1.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_mul_28ns_29ns_57_2_1.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_mul_30s_29ns_58_2_1.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_mul_80s_24ns_80_5_1.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_regslice_both.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_scaled_fixed2ieee_29_1_Pipeline_1.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_scaled_fixed2ieee_29_1_Pipeline_2.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_scaled_fixed2ieee_29_1_s.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_sin_or_cos_float_Pipeline_1.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_sin_or_cos_float_Pipeline_2.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_sin_or_cos_float_s.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_sin_or_cos_float_s_ref_4oPi_table_100_ROM_1P_LUTRAM_1R.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_sin_or_cos_float_s_second_order_float_sin_cos_K0_ROM_1P_LUTRAM_1R.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_sin_or_cos_float_s_second_order_float_sin_cos_K1_ROM_1P_LUTRAM_1R.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_sin_or_cos_float_s_second_order_float_sin_cos_K2_ROM_1P_LUTRAM_1R.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_sitofp_32s_32_6_no_dsp_1.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_sparsemux_9_3_32_1_1.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_sparsemux_11_4_55_1_1.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_sparsemux_17_3_1_1_1.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion_sparsemux_33_4_1_1_1.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/874d/hdl/vhdl/sensor_fusion.vhd" \
"../../../bd/drone_block_design/ip/drone_block_design_sensor_fusion_0_1/sim/drone_block_design_sensor_fusion_0_1.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/958f/hdl/vhdl/pid_controller_mul_20s_32s_48_2_1.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/958f/hdl/vhdl/pid_controller_mul_21s_32s_48_2_1.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/958f/hdl/vhdl/pid_controller_mul_32s_32s_48_2_1.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/958f/hdl/vhdl/pid_controller_mul_33s_24ns_48_2_1.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/958f/hdl/vhdl/pid_controller_regslice_both.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/958f/hdl/vhdl/pid_controller.vhd" \
"../../../bd/drone_block_design/ip/drone_block_design_pid_controller_0_1/sim/drone_block_design_pid_controller_0_1.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/7483/hdl/vhdl/mixer_module_regslice_both.vhd" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/7483/hdl/vhdl/mixer_module.vhd" \
"../../../bd/drone_block_design/ip/drone_block_design_mixer_module_0_0/sim/drone_block_design_mixer_module_0_0.vhd" \
"../../../bd/drone_block_design/ip/drone_block_design_util_ds_buf_0_0/util_ds_buf.vhd" \
"../../../bd/drone_block_design/ip/drone_block_design_util_ds_buf_0_0/sim/drone_block_design_util_ds_buf_0_0.vhd" \
"../../../bd/drone_block_design/ip/drone_block_design_util_ds_buf_1_0/sim/drone_block_design_util_ds_buf_1_0.vhd" \

vlog -work xlconstant_v1_1_10  -incr -mfcu  "+incdir+../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/ec67/hdl" "+incdir+../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/9a25/hdl" "+incdir+../../../../../../../../../../../AMDDesignTools/2025.2/Vivado/data/rsb/busdef" "+incdir+C:/AMDDesignTools/2025.2/Vivado/data/xilinx_vip/include" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/a165/hdl/xlconstant_v1_1_vl_rfs.v" \

vlog -work xil_defaultlib  -incr -mfcu  "+incdir+../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/ec67/hdl" "+incdir+../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/9a25/hdl" "+incdir+../../../../../../../../../../../AMDDesignTools/2025.2/Vivado/data/rsb/busdef" "+incdir+C:/AMDDesignTools/2025.2/Vivado/data/xilinx_vip/include" \
"../../../bd/drone_block_design/ip/drone_block_design_xlconstant_0_0/sim/drone_block_design_xlconstant_0_0.v" \
"../../../bd/drone_block_design/ip/drone_block_design_xlconstant_1_0/sim/drone_block_design_xlconstant_1_0.v" \

vcom -work xil_defaultlib  -93  \
"../../../bd/drone_block_design/ipshared/a5c2/hdl/i2c_master.vhd" \
"../../../bd/drone_block_design/ipshared/a5c2/hdl/pca9685_controller.vhd" \
"../../../bd/drone_block_design/ip/drone_block_design_pca9685_controller_0_1/sim/drone_block_design_pca9685_controller_0_1.vhd" \
"../../../bd/drone_block_design/ip/drone_block_design_util_ds_buf_2_0_1/sim/drone_block_design_util_ds_buf_2_0.vhd" \
"../../../bd/drone_block_design/ip/drone_block_design_util_ds_buf_3_0_1/sim/drone_block_design_util_ds_buf_3_0.vhd" \

vlog -work xil_defaultlib  -incr -mfcu  "+incdir+../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/ec67/hdl" "+incdir+../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/9a25/hdl" "+incdir+../../../../../../../../../../../AMDDesignTools/2025.2/Vivado/data/rsb/busdef" "+incdir+C:/AMDDesignTools/2025.2/Vivado/data/xilinx_vip/include" \
"../../../bd/drone_block_design/ip/drone_block_design_xlconstant_2_0_1/sim/drone_block_design_xlconstant_2_0.v" \
"../../../bd/drone_block_design/ip/drone_block_design_xlconstant_3_0_1/sim/drone_block_design_xlconstant_3_0.v" \

vcom -work xil_defaultlib  -93  \
"../../../bd/drone_block_design/ipshared/8ffd/hdl/control_manager.vhd" \
"../../../bd/drone_block_design/ip/drone_block_design_control_manager_0_3/sim/drone_block_design_control_manager_0_3.vhd" \

vlog -work axi_protocol_converter_v2_1_37  -incr -mfcu  "+incdir+../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/ec67/hdl" "+incdir+../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/9a25/hdl" "+incdir+../../../../../../../../../../../AMDDesignTools/2025.2/Vivado/data/rsb/busdef" "+incdir+C:/AMDDesignTools/2025.2/Vivado/data/xilinx_vip/include" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/d98a/hdl/axi_protocol_converter_v2_1_vl_rfs.v" \

vlog -work xil_defaultlib  -incr -mfcu  "+incdir+../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/ec67/hdl" "+incdir+../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/9a25/hdl" "+incdir+../../../../../../../../../../../AMDDesignTools/2025.2/Vivado/data/rsb/busdef" "+incdir+C:/AMDDesignTools/2025.2/Vivado/data/xilinx_vip/include" \
"../../../bd/drone_block_design/ip/drone_block_design_axi_mem_intercon_imp_auto_pc_2/sim/drone_block_design_axi_mem_intercon_imp_auto_pc_2.v" \

vlog -work axi_clock_converter_v2_1_35  -incr -mfcu  "+incdir+../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/ec67/hdl" "+incdir+../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/9a25/hdl" "+incdir+../../../../../../../../../../../AMDDesignTools/2025.2/Vivado/data/rsb/busdef" "+incdir+C:/AMDDesignTools/2025.2/Vivado/data/xilinx_vip/include" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/e059/hdl/axi_clock_converter_v2_1_vl_rfs.v" \

vlog -work blk_mem_gen_v8_4_12  -incr -mfcu  "+incdir+../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/ec67/hdl" "+incdir+../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/9a25/hdl" "+incdir+../../../../../../../../../../../AMDDesignTools/2025.2/Vivado/data/rsb/busdef" "+incdir+C:/AMDDesignTools/2025.2/Vivado/data/xilinx_vip/include" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/42f3/simulation/blk_mem_gen_v8_4.v" \

vlog -work axi_dwidth_converter_v2_1_37  -incr -mfcu  "+incdir+../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/ec67/hdl" "+incdir+../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/9a25/hdl" "+incdir+../../../../../../../../../../../AMDDesignTools/2025.2/Vivado/data/rsb/busdef" "+incdir+C:/AMDDesignTools/2025.2/Vivado/data/xilinx_vip/include" \
"../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/21f8/hdl/axi_dwidth_converter_v2_1_vl_rfs.v" \

vlog -work xil_defaultlib  -incr -mfcu  "+incdir+../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/ec67/hdl" "+incdir+../../../../drone_overlay.gen/sources_1/bd/drone_block_design/ipshared/9a25/hdl" "+incdir+../../../../../../../../../../../AMDDesignTools/2025.2/Vivado/data/rsb/busdef" "+incdir+C:/AMDDesignTools/2025.2/Vivado/data/xilinx_vip/include" \
"../../../bd/drone_block_design/ip/drone_block_design_axi_mem_intercon_imp_auto_us_0/sim/drone_block_design_axi_mem_intercon_imp_auto_us_0.v" \
"../../../bd/drone_block_design/ip/drone_block_design_axi_mem_intercon_imp_auto_pc_0/sim/drone_block_design_axi_mem_intercon_imp_auto_pc_0.v" \
"../../../bd/drone_block_design/ip/drone_block_design_axi_mem_intercon_imp_auto_ds_0/sim/drone_block_design_axi_mem_intercon_imp_auto_ds_0.v" \
"../../../bd/drone_block_design/ip/drone_block_design_axi_mem_intercon_imp_auto_pc_1/sim/drone_block_design_axi_mem_intercon_imp_auto_pc_1.v" \

vcom -work xil_defaultlib  -93  \
"../../../bd/drone_block_design/sim/drone_block_design.vhd" \

vlog -work xil_defaultlib \
"glbl.v"

