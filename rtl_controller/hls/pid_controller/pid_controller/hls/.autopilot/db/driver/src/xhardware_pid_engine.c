// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2025.2 (64-bit)
// Tool Version Limit: 2025.11
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2025 Advanced Micro Devices, Inc. All Rights Reserved.
// 
// ==============================================================
/***************************** Include Files *********************************/
#include "xhardware_pid_engine.h"

/************************** Function Implementation *************************/
#ifndef __linux__
int XHardware_pid_engine_CfgInitialize(XHardware_pid_engine *InstancePtr, XHardware_pid_engine_Config *ConfigPtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(ConfigPtr != NULL);

    InstancePtr->Ctrl_bus_BaseAddress = ConfigPtr->Ctrl_bus_BaseAddress;
    InstancePtr->IsReady = XIL_COMPONENT_IS_READY;

    return XST_SUCCESS;
}
#endif

void XHardware_pid_engine_Start(XHardware_pid_engine *InstancePtr) {
    u32 Data;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XHardware_pid_engine_ReadReg(InstancePtr->Ctrl_bus_BaseAddress, XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_AP_CTRL) & 0x80;
    XHardware_pid_engine_WriteReg(InstancePtr->Ctrl_bus_BaseAddress, XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_AP_CTRL, Data | 0x01);
}

u32 XHardware_pid_engine_IsDone(XHardware_pid_engine *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XHardware_pid_engine_ReadReg(InstancePtr->Ctrl_bus_BaseAddress, XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_AP_CTRL);
    return (Data >> 1) & 0x1;
}

u32 XHardware_pid_engine_IsIdle(XHardware_pid_engine *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XHardware_pid_engine_ReadReg(InstancePtr->Ctrl_bus_BaseAddress, XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_AP_CTRL);
    return (Data >> 2) & 0x1;
}

u32 XHardware_pid_engine_IsReady(XHardware_pid_engine *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XHardware_pid_engine_ReadReg(InstancePtr->Ctrl_bus_BaseAddress, XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_AP_CTRL);
    // check ap_start to see if the pcore is ready for next input
    return !(Data & 0x1);
}

void XHardware_pid_engine_EnableAutoRestart(XHardware_pid_engine *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XHardware_pid_engine_WriteReg(InstancePtr->Ctrl_bus_BaseAddress, XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_AP_CTRL, 0x80);
}

void XHardware_pid_engine_DisableAutoRestart(XHardware_pid_engine *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XHardware_pid_engine_WriteReg(InstancePtr->Ctrl_bus_BaseAddress, XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_AP_CTRL, 0);
}

void XHardware_pid_engine_Set_set_roll(XHardware_pid_engine *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XHardware_pid_engine_WriteReg(InstancePtr->Ctrl_bus_BaseAddress, XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_SET_ROLL_DATA, Data);
}

u32 XHardware_pid_engine_Get_set_roll(XHardware_pid_engine *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XHardware_pid_engine_ReadReg(InstancePtr->Ctrl_bus_BaseAddress, XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_SET_ROLL_DATA);
    return Data;
}

void XHardware_pid_engine_Set_set_pitch(XHardware_pid_engine *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XHardware_pid_engine_WriteReg(InstancePtr->Ctrl_bus_BaseAddress, XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_SET_PITCH_DATA, Data);
}

u32 XHardware_pid_engine_Get_set_pitch(XHardware_pid_engine *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XHardware_pid_engine_ReadReg(InstancePtr->Ctrl_bus_BaseAddress, XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_SET_PITCH_DATA);
    return Data;
}

void XHardware_pid_engine_Set_set_yaw(XHardware_pid_engine *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XHardware_pid_engine_WriteReg(InstancePtr->Ctrl_bus_BaseAddress, XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_SET_YAW_DATA, Data);
}

u32 XHardware_pid_engine_Get_set_yaw(XHardware_pid_engine *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XHardware_pid_engine_ReadReg(InstancePtr->Ctrl_bus_BaseAddress, XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_SET_YAW_DATA);
    return Data;
}

void XHardware_pid_engine_Set_gyro_x(XHardware_pid_engine *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XHardware_pid_engine_WriteReg(InstancePtr->Ctrl_bus_BaseAddress, XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_GYRO_X_DATA, Data);
}

u32 XHardware_pid_engine_Get_gyro_x(XHardware_pid_engine *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XHardware_pid_engine_ReadReg(InstancePtr->Ctrl_bus_BaseAddress, XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_GYRO_X_DATA);
    return Data;
}

void XHardware_pid_engine_Set_gyro_y(XHardware_pid_engine *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XHardware_pid_engine_WriteReg(InstancePtr->Ctrl_bus_BaseAddress, XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_GYRO_Y_DATA, Data);
}

u32 XHardware_pid_engine_Get_gyro_y(XHardware_pid_engine *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XHardware_pid_engine_ReadReg(InstancePtr->Ctrl_bus_BaseAddress, XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_GYRO_Y_DATA);
    return Data;
}

void XHardware_pid_engine_Set_gyro_z(XHardware_pid_engine *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XHardware_pid_engine_WriteReg(InstancePtr->Ctrl_bus_BaseAddress, XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_GYRO_Z_DATA, Data);
}

u32 XHardware_pid_engine_Get_gyro_z(XHardware_pid_engine *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XHardware_pid_engine_ReadReg(InstancePtr->Ctrl_bus_BaseAddress, XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_GYRO_Z_DATA);
    return Data;
}

void XHardware_pid_engine_Set_kp(XHardware_pid_engine *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XHardware_pid_engine_WriteReg(InstancePtr->Ctrl_bus_BaseAddress, XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_KP_DATA, Data);
}

u32 XHardware_pid_engine_Get_kp(XHardware_pid_engine *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XHardware_pid_engine_ReadReg(InstancePtr->Ctrl_bus_BaseAddress, XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_KP_DATA);
    return Data;
}

void XHardware_pid_engine_Set_ki(XHardware_pid_engine *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XHardware_pid_engine_WriteReg(InstancePtr->Ctrl_bus_BaseAddress, XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_KI_DATA, Data);
}

u32 XHardware_pid_engine_Get_ki(XHardware_pid_engine *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XHardware_pid_engine_ReadReg(InstancePtr->Ctrl_bus_BaseAddress, XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_KI_DATA);
    return Data;
}

void XHardware_pid_engine_Set_kd(XHardware_pid_engine *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XHardware_pid_engine_WriteReg(InstancePtr->Ctrl_bus_BaseAddress, XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_KD_DATA, Data);
}

u32 XHardware_pid_engine_Get_kd(XHardware_pid_engine *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XHardware_pid_engine_ReadReg(InstancePtr->Ctrl_bus_BaseAddress, XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_KD_DATA);
    return Data;
}

void XHardware_pid_engine_Set_dt(XHardware_pid_engine *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XHardware_pid_engine_WriteReg(InstancePtr->Ctrl_bus_BaseAddress, XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_DT_DATA, Data);
}

u32 XHardware_pid_engine_Get_dt(XHardware_pid_engine *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XHardware_pid_engine_ReadReg(InstancePtr->Ctrl_bus_BaseAddress, XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_DT_DATA);
    return Data;
}

void XHardware_pid_engine_Set_throttle(XHardware_pid_engine *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XHardware_pid_engine_WriteReg(InstancePtr->Ctrl_bus_BaseAddress, XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_THROTTLE_DATA, Data);
}

u32 XHardware_pid_engine_Get_throttle(XHardware_pid_engine *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XHardware_pid_engine_ReadReg(InstancePtr->Ctrl_bus_BaseAddress, XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_THROTTLE_DATA);
    return Data;
}

u32 XHardware_pid_engine_Get_out_roll(XHardware_pid_engine *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XHardware_pid_engine_ReadReg(InstancePtr->Ctrl_bus_BaseAddress, XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_OUT_ROLL_DATA);
    return Data;
}

u32 XHardware_pid_engine_Get_out_roll_vld(XHardware_pid_engine *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XHardware_pid_engine_ReadReg(InstancePtr->Ctrl_bus_BaseAddress, XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_OUT_ROLL_CTRL);
    return Data & 0x1;
}

u32 XHardware_pid_engine_Get_out_pitch(XHardware_pid_engine *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XHardware_pid_engine_ReadReg(InstancePtr->Ctrl_bus_BaseAddress, XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_OUT_PITCH_DATA);
    return Data;
}

u32 XHardware_pid_engine_Get_out_pitch_vld(XHardware_pid_engine *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XHardware_pid_engine_ReadReg(InstancePtr->Ctrl_bus_BaseAddress, XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_OUT_PITCH_CTRL);
    return Data & 0x1;
}

u32 XHardware_pid_engine_Get_out_yaw(XHardware_pid_engine *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XHardware_pid_engine_ReadReg(InstancePtr->Ctrl_bus_BaseAddress, XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_OUT_YAW_DATA);
    return Data;
}

u32 XHardware_pid_engine_Get_out_yaw_vld(XHardware_pid_engine *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XHardware_pid_engine_ReadReg(InstancePtr->Ctrl_bus_BaseAddress, XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_OUT_YAW_CTRL);
    return Data & 0x1;
}

void XHardware_pid_engine_InterruptGlobalEnable(XHardware_pid_engine *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XHardware_pid_engine_WriteReg(InstancePtr->Ctrl_bus_BaseAddress, XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_GIE, 1);
}

void XHardware_pid_engine_InterruptGlobalDisable(XHardware_pid_engine *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XHardware_pid_engine_WriteReg(InstancePtr->Ctrl_bus_BaseAddress, XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_GIE, 0);
}

void XHardware_pid_engine_InterruptEnable(XHardware_pid_engine *InstancePtr, u32 Mask) {
    u32 Register;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Register =  XHardware_pid_engine_ReadReg(InstancePtr->Ctrl_bus_BaseAddress, XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_IER);
    XHardware_pid_engine_WriteReg(InstancePtr->Ctrl_bus_BaseAddress, XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_IER, Register | Mask);
}

void XHardware_pid_engine_InterruptDisable(XHardware_pid_engine *InstancePtr, u32 Mask) {
    u32 Register;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Register =  XHardware_pid_engine_ReadReg(InstancePtr->Ctrl_bus_BaseAddress, XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_IER);
    XHardware_pid_engine_WriteReg(InstancePtr->Ctrl_bus_BaseAddress, XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_IER, Register & (~Mask));
}

void XHardware_pid_engine_InterruptClear(XHardware_pid_engine *InstancePtr, u32 Mask) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XHardware_pid_engine_WriteReg(InstancePtr->Ctrl_bus_BaseAddress, XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_ISR, Mask);
}

u32 XHardware_pid_engine_InterruptGetEnabled(XHardware_pid_engine *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XHardware_pid_engine_ReadReg(InstancePtr->Ctrl_bus_BaseAddress, XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_IER);
}

u32 XHardware_pid_engine_InterruptGetStatus(XHardware_pid_engine *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XHardware_pid_engine_ReadReg(InstancePtr->Ctrl_bus_BaseAddress, XHARDWARE_PID_ENGINE_CTRL_BUS_ADDR_ISR);
}

