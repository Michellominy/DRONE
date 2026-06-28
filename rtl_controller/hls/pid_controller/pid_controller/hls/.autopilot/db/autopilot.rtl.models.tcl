set SynModuleInfo {
  {SRCNAME hardware_pid_engine MODELNAME hardware_pid_engine RTLNAME hardware_pid_engine IS_TOP 1
    SUBMODULES {
      {MODELNAME hardware_pid_engine_mul_32s_32s_48_2_1 RTLNAME hardware_pid_engine_mul_32s_32s_48_2_1 BINDTYPE op TYPE mul IMPL auto LATENCY 1 ALLOW_PRAGMA 1}
      {MODELNAME hardware_pid_engine_sdiv_48ns_32s_32_52_seq_1 RTLNAME hardware_pid_engine_sdiv_48ns_32s_32_52_seq_1 BINDTYPE op TYPE sdiv IMPL auto_seq LATENCY 51 ALLOW_PRAGMA 1}
      {MODELNAME hardware_pid_engine_CTRL_BUS_s_axi RTLNAME hardware_pid_engine_CTRL_BUS_s_axi BINDTYPE interface TYPE interface_s_axilite}
    }
  }
}
