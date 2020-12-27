#******************************************************************************
#
# Makefile - Rules for building the f302r8 example.
#
# Copyright (c) 2013-2014 Texas Instruments Incorporated.  All rights reserved.
# Software License Agreement
# 
# Texas Instruments (TI) is supplying this software for use solely and
# exclusively on TI's microcontroller products. The software is owned by
# TI and/or its suppliers, and is protected under applicable copyright
# laws. You may not combine this software with "viral" open-source
# software in order to form a larger program.
# 
# THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
# NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
# NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
# CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
# DAMAGES, FOR ANY REASON WHATSOEVER.
# 
# This is part of revision 2.1.0.12573 of the EK-TM4C1294XL Firmware Package.
#
#******************************************************************************

#
# The base directory for TivaWare.
#
ROOT=.

#
# Include the common make definitions.
#
include ${ROOT}/makedefs

#
# Where to find header files that do not live in the source directory.
#
IPATH = ./inc
IPATH += ./driverlib/STM32F30x_StdPeriph_Driver/inc
IPATH += ./driverlib/CMSIS/Include
IPATH += ./driverlib/CMSIS/Device/ST/STM32F30x/Include
IPATH += ./src/draw
IPATH += ./src/draw/tslib

IPATH += ./rt-thread/include
IPATH += ./rt-thread/components/finsh

VPATH=./src
VPATH += ./driverlib/STM32F30x_StdPeriph_Driver/src
VPATH += ./driverlib/CMSIS/Device/ST/STM32F30x/Source/Templates/gcc_ride7
VPATH += ./src/draw
VPATH += ./src/draw/tslib

VPATH += ./rt-thread/src
VPATH += ./rt-thread/components/finsh
VPATH += ./rt-thread/libcpu/arm/cortex-m4

#
# The default rule, which causes the f302r8 example to be built.
#
all: ${COMPILER}
all: ${COMPILER}/f302r8.axf

#
# The rule to clean out all the build products.
#
clean:
	@rm -rf ${COMPILER} ${wildcard *~}

install:
	jlink.exe burn.txt
#
# The rule to create the target directory.
#
${COMPILER}:
	@mkdir -p ${COMPILER}

#
# Rules for building the f302r8 example.
#
${COMPILER}/f302r8.axf: ${COMPILER}/startup_stm32f30x.o
${COMPILER}/f302r8.axf: ${COMPILER}/system_stm32f30x.o
${COMPILER}/f302r8.axf: ${COMPILER}/main.o
${COMPILER}/f302r8.axf: ${COMPILER}/stm32f30x_it.o
${COMPILER}/f302r8.axf: ${COMPILER}/mymisc.o
${COMPILER}/f302r8.axf: ${COMPILER}/ads7843.o
${COMPILER}/f302r8.axf: ${COMPILER}/font_8x8.o
${COMPILER}/f302r8.axf: ${COMPILER}/ili9325.o
${COMPILER}/f302r8.axf: ${COMPILER}/tslib.o
${COMPILER}/f302r8.axf: ${COMPILER}/testutils.o
${COMPILER}/f302r8.axf: ${COMPILER}/ts_calibrate.o
${COMPILER}/f302r8.axf: ${COMPILER}/stm32f30x_adc.o
${COMPILER}/f302r8.axf: ${COMPILER}/stm32f30x_can.o
${COMPILER}/f302r8.axf: ${COMPILER}/stm32f30x_comp.o
${COMPILER}/f302r8.axf: ${COMPILER}/stm32f30x_crc.o
${COMPILER}/f302r8.axf: ${COMPILER}/stm32f30x_dac.o
${COMPILER}/f302r8.axf: ${COMPILER}/stm32f30x_dbgmcu.o
${COMPILER}/f302r8.axf: ${COMPILER}/stm32f30x_dma.o
${COMPILER}/f302r8.axf: ${COMPILER}/stm32f30x_exti.o
${COMPILER}/f302r8.axf: ${COMPILER}/stm32f30x_flash.o
${COMPILER}/f302r8.axf: ${COMPILER}/stm32f30x_fmc.o
${COMPILER}/f302r8.axf: ${COMPILER}/stm32f30x_gpio.o
${COMPILER}/f302r8.axf: ${COMPILER}/stm32f30x_hrtim.o
${COMPILER}/f302r8.axf: ${COMPILER}/stm32f30x_i2c.o
${COMPILER}/f302r8.axf: ${COMPILER}/stm32f30x_iwdg.o
${COMPILER}/f302r8.axf: ${COMPILER}/stm32f30x_misc.o
${COMPILER}/f302r8.axf: ${COMPILER}/stm32f30x_opamp.o
${COMPILER}/f302r8.axf: ${COMPILER}/stm32f30x_pwr.o
${COMPILER}/f302r8.axf: ${COMPILER}/stm32f30x_rcc.o
${COMPILER}/f302r8.axf: ${COMPILER}/stm32f30x_rtc.o
${COMPILER}/f302r8.axf: ${COMPILER}/stm32f30x_spi.o
${COMPILER}/f302r8.axf: ${COMPILER}/stm32f30x_syscfg.o
${COMPILER}/f302r8.axf: ${COMPILER}/stm32f30x_tim.o
${COMPILER}/f302r8.axf: ${COMPILER}/stm32f30x_usart.o
${COMPILER}/f302r8.axf: ${COMPILER}/stm32f30x_wwdg.o
#rtthread
${COMPILER}/f302r8.axf: ${COMPILER}/clock.o
${COMPILER}/f302r8.axf: ${COMPILER}/components.o
${COMPILER}/f302r8.axf: ${COMPILER}/context_gcc.o
${COMPILER}/f302r8.axf: ${COMPILER}/cpuport.o
${COMPILER}/f302r8.axf: ${COMPILER}/idle.o
${COMPILER}/f302r8.axf: ${COMPILER}/ipc.o
${COMPILER}/f302r8.axf: ${COMPILER}/irq.o
${COMPILER}/f302r8.axf: ${COMPILER}/kservice.o
${COMPILER}/f302r8.axf: ${COMPILER}/mem.o
${COMPILER}/f302r8.axf: ${COMPILER}/memheap.o
${COMPILER}/f302r8.axf: ${COMPILER}/mempool.o
${COMPILER}/f302r8.axf: ${COMPILER}/object.o
${COMPILER}/f302r8.axf: ${COMPILER}/scheduler.o
${COMPILER}/f302r8.axf: ${COMPILER}/thread.o
${COMPILER}/f302r8.axf: ${COMPILER}/timer.o
${COMPILER}/f302r8.axf: ${COMPILER}/cmd.o
${COMPILER}/f302r8.axf: ${COMPILER}/msh.o
${COMPILER}/f302r8.axf: ${COMPILER}/shell.o

${COMPILER}/f302r8.axf: f302r8.ld
SCATTERgcc_f302r8=f302r8.ld
ENTRY_f302r8=Reset_Handler
CFLAGSgcc=-DUSE_STDPERIPH_DRIVER -DSTM32F302x8
CFLAGSgcc+=-DDEBUG -D__FPU_PRESENT -D__VFP_FP__
#
# Include the automatically generated dependency files.
#
ifneq (${MAKECMDGOALS},clean)
-include ${wildcard ${COMPILER}/*.d} __dummy__
endif
