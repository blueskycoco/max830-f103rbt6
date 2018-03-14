#******************************************************************************
#
# Makefile - Rules for building the gd103 example.
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
IPATH=./inc
IPATH+=./usb/inc
VPATH=./usb/src
#
# The default rule, which causes the gd103 example to be built.
#
all: ${COMPILER}
all: ${COMPILER}/gd103.axf

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
# Rules for building the gd103 example.
#
${COMPILER}/gd103.axf: ${COMPILER}/startup_${COMPILER}_103c8t6.o
${COMPILER}/gd103.axf: ${COMPILER}/system_stm32f10x.o
${COMPILER}/gd103.axf: ${COMPILER}/gd103.o
${COMPILER}/gd103.axf: ${COMPILER}/syscalls.o
${COMPILER}/gd103.axf: ${COMPILER}/mymisc.o
${COMPILER}/gd103.axf: ${COMPILER}/stm32f1xx_hal_pcd.o
${COMPILER}/gd103.axf: ${COMPILER}/stm32f1xx_hal_pcd_ex.o
${COMPILER}/gd103.axf: ${COMPILER}/usb.o
${COMPILER}/gd103.axf: ${COMPILER}/usbd_cdc.o
${COMPILER}/gd103.axf: ${COMPILER}/usbd_cdc_interface.o
${COMPILER}/gd103.axf: ${COMPILER}/usbd_conf.o
${COMPILER}/gd103.axf: ${COMPILER}/usbd_core.o
${COMPILER}/gd103.axf: ${COMPILER}/usbd_ctlreq.o
${COMPILER}/gd103.axf: ${COMPILER}/usbd_desc.o
${COMPILER}/gd103.axf: ${COMPILER}/usbd_ioreq.o
${COMPILER}/gd103.axf: ${ROOT}/driverlib/${COMPILER}/libdriver.a
${COMPILER}/gd103.axf: gd103.ld
SCATTERgcc_gd103=gd103.ld
ENTRY_gd103=Reset_Handler
#CFLAGSgcc=-DUSE_STDPERIPH_DRIVER -DSTM32F10X_MD -DMASTER
CFLAGSgcc=-DUSE_STDPERIPH_DRIVER -DSTM32F10X_MD -DSTM32F103x6
CFLAGSgcc+=-DDEBUG
#
# Include the automatically generated dependency files.
#
ifneq (${MAKECMDGOALS},clean)
-include ${wildcard ${COMPILER}/*.d} __dummy__
endif
