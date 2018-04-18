#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stm32f0xx.h>
#include <string.h>
#include <stdlib.h>
#include "mymisc.h"
volatile uint8_t uart_rx_ind = 0;
uint32_t cnt=0;
uint8_t rx_buf[64] = {0};
void USART1_IRQHandler(void)
{
	//int8_t ch;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		if(USART_GetFlagStatus(USART1, USART_FLAG_PE) == RESET)
		{
			//while((ch = GetChar()) != -1)
			rx_buf[cnt++] = GetChar();
		}
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
	if(USART_GetITStatus(USART1, USART_IT_RTO) != RESET)
	{
		uart_rx_ind = 1;
		USART_ClearITPendingBit(USART1, USART_IT_RTO);
	}
}
int main(void)
{	
	int i;
//	EXTI1_Config();
	led_init();
	delay_init(48);
	Uart_Init();
	printf("float system on\r\n");
	led(0);
	while(1) {
		__WFI();
		printf("stm32 wakeup\r\n");
		if (uart_rx_ind) {
			printf("got cmd[%d]: ",cnt);
			for (i=0; i<cnt; i++)
				printf("%02x ", rx_buf[i]);
			printf("\r\n");
			uart_rx_ind = 0;			
			cnt=0;
		}
	}
}
