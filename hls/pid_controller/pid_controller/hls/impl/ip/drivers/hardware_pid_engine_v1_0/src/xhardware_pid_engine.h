// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2025.2 (64-bit)
// Tool Version Limit: 2025.11
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2025 Advanced Micro Devices, Inc. All Rights Reserved.
// 
// ==============================================================
#ifndef XHARDWARE_PID_ENGINE_H
#define XHARDWARE_PID_ENGINE_H

#ifdef __cplusplus
extern "C" {
#endif

/***************************** Include Files *********************************/
#ifndef __linux__
#include "xil_types.h"
#include "xil_assert.h"
#include "xstatus.h"
#include "xil_io.h"
#else
#include <stdint.h>
#include <assert.h>
#include <dirent.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stddef.h>
#endif
#include "xhardware_pid_engine_hw.h"

/**************************** Type Definitions ******************************/
#ifdef __linux__
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
#else
typedef struct {
#ifdef SDT
    char *Name;
#else
    u16 DeviceId;
#endif
    u64 Ctrl_bus_BaseAddress;
} XHardware_pid_engine_Config;
#endif

typedef struct {
    u64 Ctrl_bus_BaseAddress;
    u32 IsReady;
} XHardware_pid_engine;

typedef u32 word_type;

/***************** Macros (Inline Functions) Definitions *********************/
#ifndef __linux__
#define XHardware_pid_engine_WriteReg(BaseAddress, RegOffset, Data) \
    Xil_Out32((BaseAddress) + (RegOffset), (u32)(Data))
#define XHardware_pid_engine_ReadReg(BaseAddress, RegOffset) \
    Xil_In32((BaseAddress) + (RegOffset))
#else
#define XHardware_pid_engine_WriteReg(BaseAddress, RegOffset, Data) \
    *(volatile u32*)((BaseAddress) + (RegOffset)) = (u32)(Data)
#define XHardware_pid_engine_ReadReg(BaseAddress, RegOffset) \
    *(volatile u32*)((BaseAddress) + (RegOffset))

#define Xil_AssertVoid(expr)    assert(expr)
#define Xil_AssertNonvoid(expr) assert(expr)

#define XST_SUCCESS             0
#define XST_DEVICE_NOT_FOUND    2
#define XST_OPEN_DEVICE_FAILED  3
#define XIL_COMPONENT_IS_READY  1
#endif

/************************** Function Prototypes *****************************/
#ifndef __linux__
#ifdef SDT
int XHardware_pid_engine_Initialize(XHardware_pid_engine *InstancePtr, UINTPTR BaseAddress);
XHardware_pid_engine_Config* XHardware_pid_engine_LookupConfig(UINTPTR BaseAddress);
#else
int XHardware_pid_engine_Initialize(XHardware_pid_engine *InstancePtr, u16 DeviceId);
XHardware_pid_engine_Config* XHardware_pid_engine_LookupConfig(u16 DeviceId);
#endif
int XHardware_pid_engine_CfgInitialize(XHardware_pid_engine *InstancePtr, XHardware_pid_engine_Config *ConfigPtr);
#else
int XHardware_pid_engine_Initialize(XHardware_pid_engine *InstancePtr, const char* InstanceName);
int XHardware_pid_engine_Release(XHardware_pid_engine *InstancePtr);
#endif

void XHardware_pid_engine_Start(XHardware_pid_engine *InstancePtr);
u32 XHardware_pid_engine_IsDone(XHardware_pid_engine *InstancePtr);
u32 XHardware_pid_engine_IsIdle(XHardware_pid_engine *InstancePtr);
u32 XHardware_pid_engine_IsReady(XHardware_pid_engine *InstancePtr);
void XHardware_pid_engine_EnableAutoRestart(XHardware_pid_engine *InstancePtr);
void XHardware_pid_engine_DisableAutoRestart(XHardware_pid_engine *InstancePtr);

void XHardware_pid_engine_Set_set_roll(XHardware_pid_engine *InstancePtr, u32 Data);
u32 XHardware_pid_engine_Get_set_roll(XHardware_pid_engine *InstancePtr);
void XHardware_pid_engine_Set_set_pitch(XHardware_pid_engine *InstancePtr, u32 Data);
u32 XHardware_pid_engine_Get_set_pitch(XHardware_pid_engine *InstancePtr);
void XHardware_pid_engine_Set_set_yaw(XHardware_pid_engine *InstancePtr, u32 Data);
u32 XHardware_pid_engine_Get_set_yaw(XHardware_pid_engine *InstancePtr);
void XHardware_pid_engine_Set_gyro_x(XHardware_pid_engine *InstancePtr, u32 Data);
u32 XHardware_pid_engine_Get_gyro_x(XHardware_pid_engine *InstancePtr);
void XHardware_pid_engine_Set_gyro_y(XHardware_pid_engine *InstancePtr, u32 Data);
u32 XHardware_pid_engine_Get_gyro_y(XHardware_pid_engine *InstancePtr);
void XHardware_pid_engine_Set_gyro_z(XHardware_pid_engine *InstancePtr, u32 Data);
u32 XHardware_pid_engine_Get_gyro_z(XHardware_pid_engine *InstancePtr);
void XHardware_pid_engine_Set_kp(XHardware_pid_engine *InstancePtr, u32 Data);
u32 XHardware_pid_engine_Get_kp(XHardware_pid_engine *InstancePtr);
void XHardware_pid_engine_Set_ki(XHardware_pid_engine *InstancePtr, u32 Data);
u32 XHardware_pid_engine_Get_ki(XHardware_pid_engine *InstancePtr);
void XHardware_pid_engine_Set_kd(XHardware_pid_engine *InstancePtr, u32 Data);
u32 XHardware_pid_engine_Get_kd(XHardware_pid_engine *InstancePtr);
void XHardware_pid_engine_Set_dt(XHardware_pid_engine *InstancePtr, u32 Data);
u32 XHardware_pid_engine_Get_dt(XHardware_pid_engine *InstancePtr);
void XHardware_pid_engine_Set_throttle(XHardware_pid_engine *InstancePtr, u32 Data);
u32 XHardware_pid_engine_Get_throttle(XHardware_pid_engine *InstancePtr);
u32 XHardware_pid_engine_Get_out_roll(XHardware_pid_engine *InstancePtr);
u32 XHardware_pid_engine_Get_out_roll_vld(XHardware_pid_engine *InstancePtr);
u32 XHardware_pid_engine_Get_out_pitch(XHardware_pid_engine *InstancePtr);
u32 XHardware_pid_engine_Get_out_pitch_vld(XHardware_pid_engine *InstancePtr);
u32 XHardware_pid_engine_Get_out_yaw(XHardware_pid_engine *InstancePtr);
u32 XHardware_pid_engine_Get_out_yaw_vld(XHardware_pid_engine *InstancePtr);

void XHardware_pid_engine_InterruptGlobalEnable(XHardware_pid_engine *InstancePtr);
void XHardware_pid_engine_InterruptGlobalDisable(XHardware_pid_engine *InstancePtr);
void XHardware_pid_engine_InterruptEnable(XHardware_pid_engine *InstancePtr, u32 Mask);
void XHardware_pid_engine_InterruptDisable(XHardware_pid_engine *InstancePtr, u32 Mask);
void XHardware_pid_engine_InterruptClear(XHardware_pid_engine *InstancePtr, u32 Mask);
u32 XHardware_pid_engine_InterruptGetEnabled(XHardware_pid_engine *InstancePtr);
u32 XHardware_pid_engine_InterruptGetStatus(XHardware_pid_engine *InstancePtr);

#ifdef __cplusplus
}
#endif

#endif
