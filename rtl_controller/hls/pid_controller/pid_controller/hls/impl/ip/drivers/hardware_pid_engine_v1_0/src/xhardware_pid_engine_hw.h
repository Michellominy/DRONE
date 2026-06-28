// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2025.2 (64-bit)
// Tool Version Limit: 2025.11
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2025 Advanced Micro Devices, Inc. All Rights Reserved.
// 
// ==============================================================
// CTRL_BUS
// 0x00 : Control signals
//        bit 0  - ap_start (Read/Write/COH)
//        bit 1  - ap_done (Read/COR)
//        bit 2  - ap_idle (Read)
//        bit 3  - ap_ready (Read/COR)
//        bit 7  - auto_restart (Read/Write)
//        bit 9  - interrupt (Read)
//        others - reserved
// 0x04 : Global Interrupt Enable Register
//        bit 0  - Global Interrupt Enable (Read/Write)
//        others - reserved
// 0x08 : IP Interrupt Enable Register (Read/Write)
//        bit 0 - enable ap_done interrupt (Read/Write)
//        bit 1 - enable ap_ready interrupt (Read/Write)
//        others - reserved
// 0x0c : IP Interrupt Status Register (Read/TOW)
//        bit 0 - ap_done (Read/TOW)
//        bit 1 - ap_ready (Read/TOW)
//        others - reserved
// 0x10 : Data signal of set_roll
//        bit 31~0 - set_roll[31:0] (Read/Write)
// 0x14 : reserved
// 0x18 : Data signal of set_pitch
//        bit 31~0 - set_pitch[31:0] (Read/Write)
// 0x1c : reserved
// 0x20 : Data signal of set_yaw
//        bit 31~0 - set_yaw[31:0] (Read/Write)
// 0x24 : reserved
// 0x28 : Data signal of gyro_x
//        bit 31~0 - gyro_x[31:0] (Read/Write)
// 0x2c : reserved
// 0x30 : Data signal of gyro_y
//        bit 31~0 - gyro_y[31:0] (Read/Write)
// 0x34 : reserved
// 0x38 : Data signal of gyro_z
//        bit 31~0 - gyro_z[31:0] (Read/Write)
// 0x3c : reserved
// 0x40 : Data signal of kp
//        bit 31~0 - kp[31:0] (Read/Write)
// 0x44 : reserved
// 0x48 : Data signal of ki
//        bit 31~0 - ki[31:0] (Read/Write)
// 0x4c : reserved
// 0x50 : Data signal of kd
//        bit 31~0 - kd[31:0] (Read/Write)
// 0x54 : reserved
// 0x58 : Data signal of dt
//        bit 31~0 - dt[31:0] (Read/Write)
// 0x5c : reserved
// 0x60 : Data signal of throttle
//        bit 31~0 - throttle[31:0] (Read/Write)
// 0x64 : reserved
// 0x68 : Data signal of out_roll
//        bit 31~0 - out_roll[31:0] (Read)
// 0x6c : Control signal of out_roll
//        bit 0  - out_roll_ap_vld (Read/COR)
//        others - reserved
// 0x78 : Data signal of out_pitch
//        bit 31~0 - out_pitch[31:0] (Read)
// 0x7c : Control signal of out_pitch
//        bit 0  - out_pitch_ap_vld (Read/COR)
//        others - reserved
// 0x88 : Data signal of out_yaw
//        bit 31~0 - out_yaw[31:0] (Read)
// 0x8c : Control signal of out_yaw
//        bit 0  - out_yaw_ap_vld (Read/COR)
//        others - reserved
// (SC = Self Clear, COR = Clear on Read, TOW = Toggle on Write, COH = Clear on Handshake)

#define XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_AP_CTRL        0x00
#define XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_GIE            0x04
#define XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_IER            0x08
#define XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_ISR            0x0c
#define XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_SET_ROLL_DATA  0x10
#define XHARDWARE_PID_ENGINE_CTRL_BUS_BITS_SET_ROLL_DATA  32
#define XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_SET_PITCH_DATA 0x18
#define XHARDWARE_PID_ENGINE_CTRL_BUS_BITS_SET_PITCH_DATA 32
#define XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_SET_YAW_DATA   0x20
#define XHARDWARE_PID_ENGINE_CTRL_BUS_BITS_SET_YAW_DATA   32
#define XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_GYRO_X_DATA    0x28
#define XHARDWARE_PID_ENGINE_CTRL_BUS_BITS_GYRO_X_DATA    32
#define XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_GYRO_Y_DATA    0x30
#define XHARDWARE_PID_ENGINE_CTRL_BUS_BITS_GYRO_Y_DATA    32
#define XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_GYRO_Z_DATA    0x38
#define XHARDWARE_PID_ENGINE_CTRL_BUS_BITS_GYRO_Z_DATA    32
#define XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_KP_DATA        0x40
#define XHARDWARE_PID_ENGINE_CTRL_BUS_BITS_KP_DATA        32
#define XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_KI_DATA        0x48
#define XHARDWARE_PID_ENGINE_CTRL_BUS_BITS_KI_DATA        32
#define XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_KD_DATA        0x50
#define XHARDWARE_PID_ENGINE_CTRL_BUS_BITS_KD_DATA        32
#define XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_DT_DATA        0x58
#define XHARDWARE_PID_ENGINE_CTRL_BUS_BITS_DT_DATA        32
#define XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_THROTTLE_DATA  0x60
#define XHARDWARE_PID_ENGINE_CTRL_BUS_BITS_THROTTLE_DATA  32
#define XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_OUT_ROLL_DATA  0x68
#define XHARDWARE_PID_ENGINE_CTRL_BUS_BITS_OUT_ROLL_DATA  32
#define XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_OUT_ROLL_CTRL  0x6c
#define XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_OUT_PITCH_DATA 0x78
#define XHARDWARE_PID_ENGINE_CTRL_BUS_BITS_OUT_PITCH_DATA 32
#define XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_OUT_PITCH_CTRL 0x7c
#define XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_OUT_YAW_DATA   0x88
#define XHARDWARE_PID_ENGINE_CTRL_BUS_BITS_OUT_YAW_DATA   32
#define XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_OUT_YAW_CTRL   0x8c

