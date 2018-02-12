#******************************************************************************
#
# Makefile - Rules for building the infrar example.
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

#
# The default rule, which causes the infrar example to be built.
#
all: ${COMPILER}
all: ${COMPILER}/infrar.axf

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
# Rules for building the infrar example.
#
${COMPILER}/infrar.axf: ${COMPILER}/startup_${COMPILER}_103c8t6.o
${COMPILER}/infrar.axf: ${COMPILER}/system_stm32f10x.o
${COMPILER}/infrar.axf: ${COMPILER}/infrar.o
${COMPILER}/infrar.axf: ${COMPILER}/syscalls.o
${COMPILER}/infrar.axf: ${COMPILER}/mymisc.o
${COMPILER}/infrar.axf: ${COMPILER}/can.o
${COMPILER}/infrar.axf: ${ROOT}/driverlib/${COMPILER}/libdriver.a
${COMPILER}/infrar.axf: infrar.ld
SCATTERgcc_infrar=infrar.ld
ENTRY_infrar=Reset_Handler
CFLAGSgcc=-DUSE_STDPERIPH_DRIVER -DSTM32F10X_HD -DMASTER
#CFLAGSgcc=-DUSE_STDPERIPH_DRIVER -DSTM32F10X_HD

#
# Include the automatically generated dependency files.
#
ifneq (${MAKECMDGOALS},clean)
-include ${wildcard ${COMPILER}/*.d} __dummy__
endif
