//*****************************************************************************
//
// startup_gcc.c - Startup code for use with GNU tools.
//
// Copyright (c) 2013-2014 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.0.12573 of the EK-TM4C1294XL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include "system_stm32f10x.h"

//*****************************************************************************
//
// Forward declaration of the default fault handlers.
//
//*****************************************************************************
void Reset_Handler(void);
static void NmiSR(void);
static void FaultISR(void);
static void IntDefaultHandler(void);
static void SVC_Handler(void);
static void DebugMon_Handler(void);
//*****************************************************************************
//
// External declaration for the interrupt handler used by the application.
//
//*****************************************************************************
extern void EXTI9_5_IRQHandler(void);

//*****************************************************************************
//
// The entry point for the application.
//
//*****************************************************************************
extern int main(void);

//*****************************************************************************
//
// Reserve space for the system stack.
//
//*****************************************************************************
static uint32_t pui32Stack[1024];
#define BootRAM ((void *)(0xF108F85F))
//*****************************************************************************
//
// The vector table.  Note that the proper constructs must be placed on this to
// ensure that it ends up at physical address 0x0000.0000.
//
//*****************************************************************************
__attribute__ ((section(".isr_vector")))
void (* const g_pfnVectors[])(void) =
{
	(void (*)(void))((uint32_t)pui32Stack + sizeof(pui32Stack)),
	// The initial stack pointer
		  Reset_Handler//              ; Reset Handler
		, NmiSR//                ; NMI Handler
		, FaultISR//          ; Hard Fault Handler
		, 0//          ; MPU Fault Handler
		, 0//           ; Bus Fault Handler
		, 0//         ; Usage Fault Handler
		, 0                   //       ; Reserved
		, 0                   //       ; Reserved
		, 0                   //       ; Reserved
		, 0                   //       ; Reserved
		, SVC_Handler         //       ; SVCall Handler
		, DebugMon_Handler    //       ; Debug Monitor Handler
		, 0                   //       ; Reserved
		, IntDefaultHandler      //       ; PendSV Handler
		, IntDefaultHandler     //       ; SysTick Handler
		, IntDefaultHandler
		, IntDefaultHandler    //        ; Window Watchdog
		, IntDefaultHandler     //        ; PVD through EXTI Line detect
		, IntDefaultHandler  //        ; Tamper
		, IntDefaultHandler     //        ; RTC
		, IntDefaultHandler   //        ; Flash
		, IntDefaultHandler     //        ; RCC
		, IntDefaultHandler   //        ; EXTI Line 0
		, IntDefaultHandler   //        ; EXTI Line 1
		, IntDefaultHandler   //        ; EXTI Line 2
		, IntDefaultHandler   //        ; EXTI Line 3
		, IntDefaultHandler   //        ; EXTI Line 4
		, IntDefaultHandler //  ; DMA1 Channel 1
		, IntDefaultHandler //  ; DMA1 Channel 2
		, IntDefaultHandler //  ; DMA1 Channel 3
		, IntDefaultHandler //  ; DMA1 Channel 4
		, IntDefaultHandler //  ; DMA1 Channel 5
		, IntDefaultHandler //  ; DMA1 Channel 6
		, IntDefaultHandler //  ; DMA1 Channel 7
		, IntDefaultHandler        //  ; ADC1 & ADC2
		, IntDefaultHandler //  ; USB High Priority or CAN1 TX
		, IntDefaultHandler //; USB Low  Priority or CAN1 RX0
		, IntDefaultHandler        //; CAN1 RX1
		, IntDefaultHandler        //; CAN1 SCE
		, IntDefaultHandler         //; EXTI Line 9..5
		, IntDefaultHandler        //; TIM1 Break
		, IntDefaultHandler         //; TIM1 Update
		, IntDefaultHandler    //; TIM1 Trigger and Commutation
		, IntDefaultHandler         //; TIM1 Capture Compare
		, IntDefaultHandler            //; TIM2
		, IntDefaultHandler            //; TIM3
		, IntDefaultHandler            //; TIM4
		, IntDefaultHandler         //; I2C1 Event
		, IntDefaultHandler         //; I2C1 Error
		, IntDefaultHandler         //; I2C2 Event
		, IntDefaultHandler         //; I2C2 Error
		, IntDefaultHandler            //; SPI1
		, IntDefaultHandler            //; SPI2
		, IntDefaultHandler          //; USART1
		, IntDefaultHandler          //; USART2
		, IntDefaultHandler          //; USART3
		, IntDefaultHandler       //; EXTI Line 15..10
		, IntDefaultHandler        //; RTC Alarm through EXTI Line
		, IntDefaultHandler       //; USB Wakeup from suspend
		, IntDefaultHandler        //; TIM8 Break
		, IntDefaultHandler         //; TIM8 Update
		, IntDefaultHandler    //; TIM8 Trigger and Commutation
		, IntDefaultHandler         //; TIM8 Capture Compare
		, IntDefaultHandler            //; ADC3
		, IntDefaultHandler            //; FSMC
		, IntDefaultHandler            //; SDIO
		, IntDefaultHandler            //; TIM5
		, IntDefaultHandler            //; SPI3
		, IntDefaultHandler           //; UART4
		, IntDefaultHandler           //; UART5
		, IntDefaultHandler            //; TIM6
		, IntDefaultHandler            //; TIM7
		, IntDefaultHandler   //; DMA2 Channel1
		, IntDefaultHandler   //; DMA2 Channel2
		, IntDefaultHandler   //; DMA2 Channel3
		, IntDefaultHandler //; DMA2 Channel4 & Channel5
		, BootRAM          /* @0x108. This is for boot in RAM mode for 
							STM32F10x Medium Density devices. */
};

//*****************************************************************************
//
// The following are constructs created by the linker, indicating where the
// the "data" and "bss" segments reside in memory.  The initializers for the
// for the "data" segment resides immediately following the "text" segment.
//
//*****************************************************************************
extern uint32_t _sidata;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;

//*****************************************************************************
//
// This is the code that gets called when the processor first starts execution
// following a reset event.  Only the absolutely necessary set is performed,
// after which the application supplied entry() routine is called.  Any fancy
// actions (such as making decisions based on the reset cause register, and
// resetting the bits in that register) are left solely in the hands of the
// application.
//
//*****************************************************************************
	void
Reset_Handler(void)
{

	uint32_t *pui32Src, *pui32Dest;

	//
	// Copy the data segment initializers from flash to SRAM.
	//
	pui32Src = &_sidata;
	for(pui32Dest = &_sdata; pui32Dest < &_edata; )
	{
		*pui32Dest++ = *pui32Src++;
	}

	//
	// Zero fill the bss segment.
	//
	for(pui32Dest = &_sbss; pui32Dest < &_ebss; )
	{
		*pui32Dest++ = 0;
	}

	//
	// Call the application's entry point.
	//
	SystemInit();
	main();
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives a NMI.  This
// simply enters an infinite loop, preserving the system state for examination
// by a debugger.
//
//*****************************************************************************
	static void
NmiSR(void)
{
	//
	// Enter an infinite loop.
	//
	while(1)
	{
	}
}
	static void
SVC_Handler(void)
{
	//
	// Enter an infinite loop.
	//
	while(1)
	{
	}
}
	static void
DebugMon_Handler(void)
{
	//
	// Enter an infinite loop.
	//
	while(1)
	{
	}
}
//*****************************************************************************
//
// This is the code that gets called when the processor receives a fault
// interrupt.  This simply enters an infinite loop, preserving the system state
// for examination by a debugger.
//
//*****************************************************************************
	static void
FaultISR(void)
{
	//
	// Enter an infinite loop.
	//
	while(1)
	{
	}
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives an unexpected
// interrupt.  This simply enters an infinite loop, preserving the system state
// for examination by a debugger.
//
//*****************************************************************************
	static void
IntDefaultHandler(void)
{
	//
	// Go into an infinite loop.
	//
	while(1)
	{
	}
}
