#******************************************************************************
#
# Makefile - Rules for building the key example.
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
IPATH += ./driverlib/STM32F0xx_StdPeriph_Driver/inc
IPATH += ./driverlib/STM32_USB_Device_Library/Class/hid/inc
IPATH += ./driverlib/STM32_USB_Device_Library/Core/inc
IPATH += ./driverlib/STM32_USB_Device_Driver/inc
IPATH += ./driverlib/CMSIS/Include
IPATH += ./driverlib/CMSIS/Device/ST/STM32F0xx/Include

VPATH=./src
VPATH += ./driverlib/STM32_USB_Device_Driver/src
VPATH += ./driverlib/STM32_USB_Device_Library/Class/hid/src
VPATH += ./driverlib/STM32_USB_Device_Library/Core/src
VPATH += ./driverlib/STM32F0xx_StdPeriph_Driver/src
#
# The default rule, which causes the key example to be built.
#
all: ${COMPILER}
all: ${COMPILER}/key.axf

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
# Rules for building the key example.
#
${COMPILER}/key.axf: ${COMPILER}/startup_${COMPILER}_042f4p6.o
${COMPILER}/key.axf: ${COMPILER}/system_stm32f0xx.o
${COMPILER}/key.axf: ${COMPILER}/key.o
${COMPILER}/key.axf: ${COMPILER}/syscalls.o
${COMPILER}/key.axf: ${COMPILER}/mymisc.o
${COMPILER}/key.axf: ${COMPILER}/stm32f0xx_adc.o
${COMPILER}/key.axf: ${COMPILER}/stm32f0xx_can.o
${COMPILER}/key.axf: ${COMPILER}/stm32f0xx_cec.o
${COMPILER}/key.axf: ${COMPILER}/stm32f0xx_comp.o
${COMPILER}/key.axf: ${COMPILER}/stm32f0xx_crc.o
${COMPILER}/key.axf: ${COMPILER}/stm32f0xx_crs.o
${COMPILER}/key.axf: ${COMPILER}/stm32f0xx_dac.o
${COMPILER}/key.axf: ${COMPILER}/stm32f0xx_dbgmcu.o
${COMPILER}/key.axf: ${COMPILER}/stm32f0xx_dma.o
${COMPILER}/key.axf: ${COMPILER}/stm32f0xx_exti.o
${COMPILER}/key.axf: ${COMPILER}/stm32f0xx_flash.o
${COMPILER}/key.axf: ${COMPILER}/stm32f0xx_gpio.o
${COMPILER}/key.axf: ${COMPILER}/stm32f0xx_i2c.o
${COMPILER}/key.axf: ${COMPILER}/stm32f0xx_iwdg.o
${COMPILER}/key.axf: ${COMPILER}/stm32f0xx_misc.o
${COMPILER}/key.axf: ${COMPILER}/stm32f0xx_pwr.o
${COMPILER}/key.axf: ${COMPILER}/stm32f0xx_rcc.o
${COMPILER}/key.axf: ${COMPILER}/stm32f0xx_rtc.o
${COMPILER}/key.axf: ${COMPILER}/stm32f0xx_spi.o
${COMPILER}/key.axf: ${COMPILER}/stm32f0xx_syscfg.o
${COMPILER}/key.axf: ${COMPILER}/stm32f0xx_tim.o
${COMPILER}/key.axf: ${COMPILER}/stm32f0xx_usart.o
${COMPILER}/key.axf: ${COMPILER}/stm32f0xx_wwdg.o
${COMPILER}/key.axf: ${COMPILER}/usb_core.o
${COMPILER}/key.axf: ${COMPILER}/usb_dcd.o
${COMPILER}/key.axf: ${COMPILER}/usb_dcd_int.o
${COMPILER}/key.axf: ${COMPILER}/usbd_custom_hid_core.o
${COMPILER}/key.axf: ${COMPILER}/usbd_core.o
${COMPILER}/key.axf: ${COMPILER}/usbd_ioreq.o
${COMPILER}/key.axf: ${COMPILER}/usbd_req.o
${COMPILER}/key.axf: ${COMPILER}/usb_bsp.o
${COMPILER}/key.axf: ${COMPILER}/usbd_desc.o
${COMPILER}/key.axf: ${COMPILER}/usbd_pwr.o
${COMPILER}/key.axf: ${COMPILER}/usbd_usr.o
${COMPILER}/key.axf: key.ld
SCATTERgcc_key=key.ld
ENTRY_key=Reset_Handler
#CFLAGSgcc=-DUSE_STDPERIPH_DRIVER -DSTM32F10X_HD -DMASTER
CFLAGSgcc=-DUSE_STDPERIPH_DRIVER -DSTM32F042
CFLAGSgcc+=-DDEBUG
#
# Include the automatically generated dependency files.
#
ifneq (${MAKECMDGOALS},clean)
-include ${wildcard ${COMPILER}/*.d} __dummy__
endif
