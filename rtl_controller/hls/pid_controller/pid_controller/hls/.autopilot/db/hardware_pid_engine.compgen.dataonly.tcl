# This script segment is generated automatically by AutoPilot

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


