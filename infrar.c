#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stm32f0xx.h>
#include <string.h>
#include <stdlib.h>
#include "mymisc.h"
static void SYSCLKConfig_STOP(void)
{  
	/* After wake-up from STOP reconfigure the system clock */
	/* Enable HSE */
	RCC_HSEConfig(RCC_HSE_ON);

	/* Wait till HSE is ready */
	while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET)
	{}

	/* Enable PLL */
	RCC_PLLCmd(ENABLE);

	/* Wait till PLL is ready */
	while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
	{}

	/* Select PLL as system clock source */
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

	/* Wait till PLL is used as system clock source */
	while (RCC_GetSYSCLKSource() != 0x08)
	{}
}
void EXTI0_1_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line1) != RESET)
	{ 
		/* Clear the TAMPER Button EXTI line pending bit */
		EXTI_ClearITPendingBit(EXTI_Line1);
		printf("user btn\r\n");
	}
}
static void EXTI0_Config(void)
{
EXTI_InitTypeDef   EXTI_InitStructure;
GPIO_InitTypeDef   GPIO_InitStructure;
NVIC_InitTypeDef   NVIC_InitStructure;
	/* Enable GPIOA clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	/* Configure PA0 pin as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	/* Connect EXTI0 Line to PA0 pin */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource1);

	/* Configure EXTI0 line */
	EXTI_InitStructure.EXTI_Line = EXTI_Line1;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI0 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
int main(void)
{	
	EXTI0_Config();
	led_init();
	delay_init(48);
	Debug_uart_Init();
	while(1) {
		//unsigned char *ptr = (unsigned char *)malloc(512);
		//if (ptr == NULL)
		if (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1) == RESET)
			printf("button is pressed\r\n");
		else
			printf("button is released\r\n");
		//else
		//{
		//	printf("malloc 1024 ok\r\n");
		//	free(ptr);
		//}
		delay_ms(500);
		led(1);
		delay_ms(500);
		led(0);
		PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);		
		SYSCLKConfig_STOP();
	}

}
