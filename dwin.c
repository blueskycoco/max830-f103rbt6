#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stm32f10x.h>
#include "dwin.h"
#define PRE_0 		0x5a 
#define PRE_1 		0xa5
#define TYPE_REG 	0x80
#define TYPE_VAL 	0x82
#define OFS_LEN		0x02
#define OFS_TYPE	0x03
void init_dwin(void)
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure);

	USART_Cmd(USART1, ENABLE);
}

void send_cmd(uint8_t *cmd, uint32_t len)
{
	int i = 0;
	for (i = 0; i < len; i++) {
		USART_SendData(USART1, cmd[i]);
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); 
	}
}

void display(uint8_t id, uint8_t *data, uint32_t len) 
{
	uint8_t type = TYPE_REG;
	uint8_t *cmd = NULL;
	uint32_t extra_len = 0;
	switch (id)
	{
		case 0:
			type = TYPE_VAL;
			break;
		default:
			break;
	}
	
	cmd = (uint8_t *)malloc((len + extra_len) * sizeof(uint8_t));
	cmd[0] = PRE_0;
	cmd[1] = PRE_1;
	cmd[OFS_LEN] = len + extra_len - 3;
	cmd[OFS_TYPE] = type;
	send_cmd(cmd, len+extra_len);
}
