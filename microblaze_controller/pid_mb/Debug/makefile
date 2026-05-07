# Copyright (C) 2022 Xilinx, Inc
# SPDX-License-Identifier: BSD-3-Clause

################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables
ELFSIZE += \
pid_mb.elf.size \

# All Target
all: pid_mb.elf secondary-outputs

# Tool invocations
pid_mb.elf: $(OBJS) ../src/lscript.ld $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MicroBlaze gcc linker'
	mb-gcc -Wl,-gc-sections \
	       -Wl,-T -Wl,../src/lscript.ld \
	       -L../../../rpi/bsp_iop_rpi/iop_rpi_mb/lib \
	       -mlittle-endian -mcpu=v11.0 -mxl-soft-mul \
	       -Wl,--no-relax \
	       -o "pid_mb.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

pid_mb.elf.size: pid_mb.elf
	@echo 'Invoking: MicroBlaze Print Size'
	mb-size pid_mb.elf | tee "pid_mb.elf.size"
	@echo 'Finished building: $@'
	@echo ' '

pid_mb.bin: pid_mb.elf
	@echo 'Invoking: MicroBlaze Bin Gen'
	mb-objcopy -O binary pid_mb.elf pid_mb.bin
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXECUTABLES) $(OBJS) $(S_UPPER_DEPS) $(C_DEPS) $(ELFSIZE) pid_mb.elf pid_mb.bin
	-@echo ' '

secondary-outputs: $(ELFSIZE) pid_mb.bin

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

