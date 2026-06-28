set moduleName hardware_pid_engine
set isTopModule 1
set isCombinational 0
set isDatapathOnly 0
set isPipelined 0
set isPipelined_legacy 0
set pipeline_type none
set FunctionProtocol ap_ctrl_hs
set restart_counter_num 0
set isOneStateSeq 0
set ProfileFlag 0
set StallSigGenFlag 0
set isEnableWaveformDebug 1
set hasInterrupt 0
set DLRegFirstOffset 0
set DLRegItemOffset 0
set svuvm_can_support 1
set cdfgNum 2
set C_modelName {hardware_pid_engine}
set C_modelType { void 0 }
set ap_memory_interface_dict [dict create]
set C_modelArgList {
	{ set_roll int 32 regular {axi_slave 0}  }
	{ set_pitch int 32 regular {axi_slave 0}  }
	{ set_yaw int 32 regular {axi_slave 0}  }
	{ gyro_x int 32 regular {axi_slave 0}  }
	{ gyro_y int 32 regular {axi_slave 0}  }
	{ gyro_z int 32 regular {axi_slave 0}  }
	{ kp int 32 regular {axi_slave 0}  }
	{ ki int 32 regular {axi_slave 0}  }
	{ kd int 32 regular {axi_slave 0}  }
	{ dt int 32 regular {axi_slave 0}  }
	{ throttle int 32 unused {axi_slave 0}  }
	{ out_roll int 32 regular {axi_slave 1}  }
	{ out_pitch int 32 regular {axi_slave 1}  }
	{ out_yaw int 32 regular {axi_slave 1}  }
}
set hasAXIMCache 0
set l_AXIML2Cache [list]
set AXIMCacheInstDict [dict create]
set C_modelArgMapList {[ 
	{ "Name" : "set_roll", "interface" : "axi_slave", "bundle":"CTRL_BUS","type":"ap_none","bitwidth" : 32, "direction" : "READONLY", "offset" : {"in":16}, "offset_end" : {"in":23}} , 
 	{ "Name" : "set_pitch", "interface" : "axi_slave", "bundle":"CTRL_BUS","type":"ap_none","bitwidth" : 32, "direction" : "READONLY", "offset" : {"in":24}, "offset_end" : {"in":31}} , 
 	{ "Name" : "set_yaw", "interface" : "axi_slave", "bundle":"CTRL_BUS","type":"ap_none","bitwidth" : 32, "direction" : "READONLY", "offset" : {"in":32}, "offset_end" : {"in":39}} , 
 	{ "Name" : "gyro_x", "interface" : "axi_slave", "bundle":"CTRL_BUS","type":"ap_none","bitwidth" : 32, "direction" : "READONLY", "offset" : {"in":40}, "offset_end" : {"in":47}} , 
 	{ "Name" : "gyro_y", "interface" : "axi_slave", "bundle":"CTRL_BUS","type":"ap_none","bitwidth" : 32, "direction" : "READONLY", "offset" : {"in":48}, "offset_end" : {"in":55}} , 
 	{ "Name" : "gyro_z", "interface" : "axi_slave", "bundle":"CTRL_BUS","type":"ap_none","bitwidth" : 32, "direction" : "READONLY", "offset" : {"in":56}, "offset_end" : {"in":63}} , 
 	{ "Name" : "kp", "interface" : "axi_slave", "bundle":"CTRL_BUS","type":"ap_none","bitwidth" : 32, "direction" : "READONLY", "offset" : {"in":64}, "offset_end" : {"in":71}} , 
 	{ "Name" : "ki", "interface" : "axi_slave", "bundle":"CTRL_BUS","type":"ap_none","bitwidth" : 32, "direction" : "READONLY", "offset" : {"in":72}, "offset_end" : {"in":79}} , 
 	{ "Name" : "kd", "interface" : "axi_slave", "bundle":"CTRL_BUS","type":"ap_none","bitwidth" : 32, "direction" : "READONLY", "offset" : {"in":80}, "offset_end" : {"in":87}} , 
 	{ "Name" : "dt", "interface" : "axi_slave", "bundle":"CTRL_BUS","type":"ap_none","bitwidth" : 32, "direction" : "READONLY", "offset" : {"in":88}, "offset_end" : {"in":95}} , 
 	{ "Name" : "throttle", "interface" : "axi_slave", "bundle":"CTRL_BUS","type":"ap_none","bitwidth" : 32, "direction" : "READONLY", "offset" : {"in":96}, "offset_end" : {"in":103}} , 
 	{ "Name" : "out_roll", "interface" : "axi_slave", "bundle":"CTRL_BUS","type":"ap_vld","bitwidth" : 32, "direction" : "WRITEONLY", "offset" : {"out":104}, "offset_end" : {"out":111}} , 
 	{ "Name" : "out_pitch", "interface" : "axi_slave", "bundle":"CTRL_BUS","type":"ap_vld","bitwidth" : 32, "direction" : "WRITEONLY", "offset" : {"out":120}, "offset_end" : {"out":127}} , 
 	{ "Name" : "out_yaw", "interface" : "axi_slave", "bundle":"CTRL_BUS","type":"ap_vld","bitwidth" : 32, "direction" : "WRITEONLY", "offset" : {"out":136}, "offset_end" : {"out":143}} ]}
# RTL Port declarations: 
set portNum 20
set portList { 
	{ ap_clk sc_in sc_logic 1 clock -1 } 
	{ ap_rst_n sc_in sc_logic 1 reset -1 active_low_sync } 
	{ s_axi_CTRL_BUS_AWVALID sc_in sc_logic 1 signal -1 } 
	{ s_axi_CTRL_BUS_AWREADY sc_out sc_logic 1 signal -1 } 
	{ s_axi_CTRL_BUS_AWADDR sc_in sc_lv 8 signal -1 } 
	{ s_axi_CTRL_BUS_WVALID sc_in sc_logic 1 signal -1 } 
	{ s_axi_CTRL_BUS_WREADY sc_out sc_logic 1 signal -1 } 
	{ s_axi_CTRL_BUS_WDATA sc_in sc_lv 32 signal -1 } 
	{ s_axi_CTRL_BUS_WSTRB sc_in sc_lv 4 signal -1 } 
	{ s_axi_CTRL_BUS_ARVALID sc_in sc_logic 1 signal -1 } 
	{ s_axi_CTRL_BUS_ARREADY sc_out sc_logic 1 signal -1 } 
	{ s_axi_CTRL_BUS_ARADDR sc_in sc_lv 8 signal -1 } 
	{ s_axi_CTRL_BUS_RVALID sc_out sc_logic 1 signal -1 } 
	{ s_axi_CTRL_BUS_RREADY sc_in sc_logic 1 signal -1 } 
	{ s_axi_CTRL_BUS_RDATA sc_out sc_lv 32 signal -1 } 
	{ s_axi_CTRL_BUS_RRESP sc_out sc_lv 2 signal -1 } 
	{ s_axi_CTRL_BUS_BVALID sc_out sc_logic 1 signal -1 } 
	{ s_axi_CTRL_BUS_BREADY sc_in sc_logic 1 signal -1 } 
	{ s_axi_CTRL_BUS_BRESP sc_out sc_lv 2 signal -1 } 
	{ interrupt sc_out sc_logic 1 signal -1 } 
}
set NewPortList {[ 
	{ "name": "s_axi_CTRL_BUS_AWADDR", "direction": "in", "datatype": "sc_lv", "bitwidth":8, "type": "signal", "bundle":{"name": "CTRL_BUS", "role": "AWADDR" },"address":[{"name":"hardware_pid_engine","role":"start","value":"0","valid_bit":"0"},{"name":"hardware_pid_engine","role":"continue","value":"0","valid_bit":"4"},{"name":"hardware_pid_engine","role":"auto_start","value":"0","valid_bit":"7"},{"name":"set_roll","role":"data","value":"16"},{"name":"set_pitch","role":"data","value":"24"},{"name":"set_yaw","role":"data","value":"32"},{"name":"gyro_x","role":"data","value":"40"},{"name":"gyro_y","role":"data","value":"48"},{"name":"gyro_z","role":"data","value":"56"},{"name":"kp","role":"data","value":"64"},{"name":"ki","role":"data","value":"72"},{"name":"kd","role":"data","value":"80"},{"name":"dt","role":"data","value":"88"},{"name":"throttle","role":"data","value":"96"}] },
	{ "name": "s_axi_CTRL_BUS_AWVALID", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "CTRL_BUS", "role": "AWVALID" } },
	{ "name": "s_axi_CTRL_BUS_AWREADY", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "CTRL_BUS", "role": "AWREADY" } },
	{ "name": "s_axi_CTRL_BUS_WVALID", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "CTRL_BUS", "role": "WVALID" } },
	{ "name": "s_axi_CTRL_BUS_WREADY", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "CTRL_BUS", "role": "WREADY" } },
	{ "name": "s_axi_CTRL_BUS_WDATA", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "CTRL_BUS", "role": "WDATA" } },
	{ "name": "s_axi_CTRL_BUS_WSTRB", "direction": "in", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "CTRL_BUS", "role": "WSTRB" } },
	{ "name": "s_axi_CTRL_BUS_ARADDR", "direction": "in", "datatype": "sc_lv", "bitwidth":8, "type": "signal", "bundle":{"name": "CTRL_BUS", "role": "ARADDR" },"address":[{"name":"hardware_pid_engine","role":"start","value":"0","valid_bit":"0"},{"name":"hardware_pid_engine","role":"done","value":"0","valid_bit":"1"},{"name":"hardware_pid_engine","role":"idle","value":"0","valid_bit":"2"},{"name":"hardware_pid_engine","role":"ready","value":"0","valid_bit":"3"},{"name":"hardware_pid_engine","role":"auto_start","value":"0","valid_bit":"7"},{"name":"out_roll","role":"data","value":"104"}, {"name":"out_roll","role":"valid","value":"108","valid_bit":"0"},{"name":"out_pitch","role":"data","value":"120"}, {"name":"out_pitch","role":"valid","value":"124","valid_bit":"0"},{"name":"out_yaw","role":"data","value":"136"}, {"name":"out_yaw","role":"valid","value":"140","valid_bit":"0"}] },
	{ "name": "s_axi_CTRL_BUS_ARVALID", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "CTRL_BUS", "role": "ARVALID" } },
	{ "name": "s_axi_CTRL_BUS_ARREADY", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "CTRL_BUS", "role": "ARREADY" } },
	{ "name": "s_axi_CTRL_BUS_RVALID", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "CTRL_BUS", "role": "RVALID" } },
	{ "name": "s_axi_CTRL_BUS_RREADY", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "CTRL_BUS", "role": "RREADY" } },
	{ "name": "s_axi_CTRL_BUS_RDATA", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "CTRL_BUS", "role": "RDATA" } },
	{ "name": "s_axi_CTRL_BUS_RRESP", "direction": "out", "datatype": "sc_lv", "bitwidth":2, "type": "signal", "bundle":{"name": "CTRL_BUS", "role": "RRESP" } },
	{ "name": "s_axi_CTRL_BUS_BVALID", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "CTRL_BUS", "role": "BVALID" } },
	{ "name": "s_axi_CTRL_BUS_BREADY", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "CTRL_BUS", "role": "BREADY" } },
	{ "name": "s_axi_CTRL_BUS_BRESP", "direction": "out", "datatype": "sc_lv", "bitwidth":2, "type": "signal", "bundle":{"name": "CTRL_BUS", "role": "BRESP" } },
	{ "name": "interrupt", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "CTRL_BUS", "role": "interrupt" } }, 
 	{ "name": "ap_clk", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "clock", "bundle":{"name": "ap_clk", "role": "default" }} , 
 	{ "name": "ap_rst_n", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "reset", "bundle":{"name": "ap_rst_n", "role": "default" }}  ]}

set ArgLastReadFirstWriteLatency {
	hardware_pid_engine {
		set_roll {Type I LastRead 0 FirstWrite -1}
		set_pitch {Type I LastRead 0 FirstWrite -1}
		set_yaw {Type I LastRead 0 FirstWrite -1}
		gyro_x {Type I LastRead 0 FirstWrite -1}
		gyro_y {Type I LastRead 0 FirstWrite -1}
		gyro_z {Type I LastRead 0 FirstWrite -1}
		kp {Type I LastRead 0 FirstWrite -1}
		ki {Type I LastRead 0 FirstWrite -1}
		kd {Type I LastRead 0 FirstWrite -1}
		dt {Type I LastRead 0 FirstWrite -1}
		throttle {Type I LastRead -1 FirstWrite -1}
		out_roll {Type O LastRead -1 FirstWrite 60}
		out_pitch {Type O LastRead -1 FirstWrite 60}
		out_yaw {Type O LastRead -1 FirstWrite 60}
		roll_st_integ {Type IO LastRead -1 FirstWrite -1}
		roll_st_last_err {Type IO LastRead -1 FirstWrite -1}
		pitch_st_integ {Type IO LastRead -1 FirstWrite -1}
		pitch_st_last_err {Type IO LastRead -1 FirstWrite -1}
		yaw_st_integ {Type IO LastRead -1 FirstWrite -1}
		yaw_st_last_err {Type IO LastRead -1 FirstWrite -1}}}

set hasDtUnsupportedChannel 0

set PerformanceInfo {[
	{"Name" : "Latency", "Min" : "8", "Max" : "60"}
	, {"Name" : "Interval", "Min" : "9", "Max" : "61"}
]}

set PipelineEnableSignalInfo {[
]}

set Spec2ImplPortList { 
}

set maxi_interface_dict [dict create]

# RTL port scheduling information:
set fifoSchedulingInfoList { 
}

# RTL bus port read request latency information:
set busReadReqLatencyList { 
}

# RTL bus port write response latency information:
set busWriteResLatencyList { 
}

# RTL array port load latency information:
set memoryLoadLatencyList { 
}
