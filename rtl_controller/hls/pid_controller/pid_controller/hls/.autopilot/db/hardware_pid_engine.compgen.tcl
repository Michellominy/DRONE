# This script segment is generated automatically by AutoPilot

set name hardware_pid_engine_mul_32s_32s_48_2_1
if {${::AESL::PGuard_rtl_comp_handler}} {
	::AP::rtl_comp_handler $name BINDTYPE {op} TYPE {mul} IMPL {auto} LATENCY 1 ALLOW_PRAGMA 1
}


set name hardware_pid_engine_sdiv_48ns_32s_32_52_seq_1
if {${::AESL::PGuard_rtl_comp_handler}} {
	::AP::rtl_comp_handler $name BINDTYPE {op} TYPE {sdiv} IMPL {auto_seq} LATENCY 51 ALLOW_PRAGMA 1
}


# clear list
if {${::AESL::PGuard_autoexp_gen}} {
    cg_default_interface_gen_dc_begin
    cg_default_interface_gen_bundle_begin
    AESL_LIB_XILADAPTER::native_axis_begin
}

set axilite_register_dict [dict create]
set port_CTRL_BUS {
set_roll { 
	dir I
	width 32
	depth 1
	mode ap_none
	offset 16
	offset_end 23
}
set_pitch { 
	dir I
	width 32
	depth 1
	mode ap_none
	offset 24
	offset_end 31
}
set_yaw { 
	dir I
	width 32
	depth 1
	mode ap_none
	offset 32
	offset_end 39
}
gyro_x { 
	dir I
	width 32
	depth 1
	mode ap_none
	offset 40
	offset_end 47
}
gyro_y { 
	dir I
	width 32
	depth 1
	mode ap_none
	offset 48
	offset_end 55
}
gyro_z { 
	dir I
	width 32
	depth 1
	mode ap_none
	offset 56
	offset_end 63
}
kp { 
	dir I
	width 32
	depth 1
	mode ap_none
	offset 64
	offset_end 71
}
ki { 
	dir I
	width 32
	depth 1
	mode ap_none
	offset 72
	offset_end 79
}
kd { 
	dir I
	width 32
	depth 1
	mode ap_none
	offset 80
	offset_end 87
}
dt { 
	dir I
	width 32
	depth 1
	mode ap_none
	offset 88
	offset_end 95
}
throttle { 
	dir I
	width 32
	depth 1
	mode ap_none
	offset 96
	offset_end 103
}
out_roll { 
	dir O
	width 32
	depth 1
	mode ap_vld
	offset 104
	offset_end 111
}
out_pitch { 
	dir O
	width 32
	depth 1
	mode ap_vld
	offset 120
	offset_end 127
}
out_yaw { 
	dir O
	width 32
	depth 1
	mode ap_vld
	offset 136
	offset_end 143
}
ap_start { }
ap_done { }
ap_ready { }
ap_idle { }
interrupt {
}
}
dict set axilite_register_dict CTRL_BUS $port_CTRL_BUS


# Native S_AXILite:
if {${::AESL::PGuard_simmodel_gen}} {
	if {[info proc ::AESL_LIB_XILADAPTER::s_axilite_gen] == "::AESL_LIB_XILADAPTER::s_axilite_gen"} {
		eval "::AESL_LIB_XILADAPTER::s_axilite_gen { \
			id 12 \
			corename hardware_pid_engine_CTRL_BUS_axilite \
			name hardware_pid_engine_CTRL_BUS_s_axi \
			ports {$port_CTRL_BUS} \
			op interface \
			interrupt_clear_mode TOW \
			interrupt_trigger_type default \
			is_flushable 0 \
			is_datawidth64 0 \
			is_addrwidth64 1 \
			enable_mem_auto_widen 1 \
		} "
	} else {
		puts "@W \[IMPL-110\] Cannot find AXI Lite interface model in the library. Ignored generation of AXI Lite  interface for 'CTRL_BUS'"
	}
}

if {${::AESL::PGuard_rtl_comp_handler}} {
	::AP::rtl_comp_handler hardware_pid_engine_CTRL_BUS_s_axi BINDTYPE interface TYPE interface_s_axilite
}


# Adapter definition:
set PortName ap_clk
set DataWd 1 
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc cg_default_interface_gen_clock] == "cg_default_interface_gen_clock"} {
eval "cg_default_interface_gen_clock { \
    id -1 \
    name ${PortName} \
    reset_level 0 \
    sync_rst true \
    corename apif_ap_clk \
    data_wd ${DataWd} \
    op interface \
}"
} else {
puts "@W \[IMPL-113\] Cannot find bus interface model in the library. Ignored generation of bus interface for '${PortName}'"
}
}


# Adapter definition:
set PortName ap_rst_n
set DataWd 1 
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc cg_default_interface_gen_reset] == "cg_default_interface_gen_reset"} {
eval "cg_default_interface_gen_reset { \
    id -2 \
    name ${PortName} \
    reset_level 0 \
    sync_rst true \
    corename apif_ap_rst_n \
    data_wd ${DataWd} \
    op interface \
}"
} else {
puts "@W \[IMPL-114\] Cannot find bus interface model in the library. Ignored generation of bus interface for '${PortName}'"
}
}



# merge
if {${::AESL::PGuard_autoexp_gen}} {
    cg_default_interface_gen_dc_end
    cg_default_interface_gen_bundle_end
    AESL_LIB_XILADAPTER::native_axis_end
}


