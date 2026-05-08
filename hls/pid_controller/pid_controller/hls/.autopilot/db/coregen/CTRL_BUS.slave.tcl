dict set slaves CTRL_BUS {ports {set_roll {type i_ap_none width 32} set_pitch {type i_ap_none width 32} set_yaw {type i_ap_none width 32} gyro_x {type i_ap_none width 32} gyro_y {type i_ap_none width 32} gyro_z {type i_ap_none width 32} kp {type i_ap_none width 32} ki {type i_ap_none width 32} kd {type i_ap_none width 32} dt {type i_ap_none width 32} throttle {type i_ap_none width 32} out_roll {type o_ap_vld width 32} out_pitch {type o_ap_vld width 32} out_yaw {type o_ap_vld width 32} ap_start {type ap_ctrl width 1} ap_done {type ap_ctrl width 1} ap_ready {type ap_ctrl width 1} ap_idle {type ap_ctrl width 1}} mems {} has_ctrl 1}
set datawidth 32
set addrwidth 64
set intr_clr_mode TOW
