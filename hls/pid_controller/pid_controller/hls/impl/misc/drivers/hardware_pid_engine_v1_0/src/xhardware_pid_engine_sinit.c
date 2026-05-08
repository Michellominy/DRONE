// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2025.2 (64-bit)
// Tool Version Limit: 2025.11
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2025 Advanced Micro Devices, Inc. All Rights Reserved.
// 
// ==============================================================
#ifndef __linux__

#include "xstatus.h"
#ifdef SDT
#include "xparameters.h"
#endif
#include "xhardware_pid_engine.h"

extern XHardware_pid_engine_Config XHardware_pid_engine_ConfigTable[];

#ifdef SDT
XHardware_pid_engine_Config *XHardware_pid_engine_LookupConfig(UINTPTR BaseAddress) {
	XHardware_pid_engine_Config *ConfigPtr = NULL;

	int Index;

	for (Index = (u32)0x0; XHardware_pid_engine_ConfigTable[Index].Name != NULL; Index++) {
		if (!BaseAddress || XHardware_pid_engine_ConfigTable[Index].Ctrl_bus_BaseAddress == BaseAddress) {
			ConfigPtr = &XHardware_pid_engine_ConfigTable[Index];
			break;
		}
	}

	return ConfigPtr;
}

int XHardware_pid_engine_Initialize(XHardware_pid_engine *InstancePtr, UINTPTR BaseAddress) {
	XHardware_pid_engine_Config *ConfigPtr;

	Xil_AssertNonvoid(InstancePtr != NULL);

	ConfigPtr = XHardware_pid_engine_LookupConfig(BaseAddress);
	if (ConfigPtr == NULL) {
		InstancePtr->IsReady = 0;
		return (XST_DEVICE_NOT_FOUND);
	}

	return XHardware_pid_engine_CfgInitialize(InstancePtr, ConfigPtr);
}
#else
XHardware_pid_engine_Config *XHardware_pid_engine_LookupConfig(u16 DeviceId) {
	XHardware_pid_engine_Config *ConfigPtr = NULL;

	int Index;

	for (Index = 0; Index < XPAR_XHARDWARE_PID_ENGINE_NUM_INSTANCES; Index++) {
		if (XHardware_pid_engine_ConfigTable[Index].DeviceId == DeviceId) {
			ConfigPtr = &XHardware_pid_engine_ConfigTable[Index];
			break;
		}
	}

	return ConfigPtr;
}

int XHardware_pid_engine_Initialize(XHardware_pid_engine *InstancePtr, u16 DeviceId) {
	XHardware_pid_engine_Config *ConfigPtr;

	Xil_AssertNonvoid(InstancePtr != NULL);

	ConfigPtr = XHardware_pid_engine_LookupConfig(DeviceId);
	if (ConfigPtr == NULL) {
		InstancePtr->IsReady = 0;
		return (XST_DEVICE_NOT_FOUND);
	}

	return XHardware_pid_engine_CfgInitialize(InstancePtr, ConfigPtr);
}
#endif

#endif

