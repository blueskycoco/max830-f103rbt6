#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stm32f0xx.h>
#include <string.h>
#include <stdlib.h>
#include "mymisc.h"
volatile uint8_t uart_rx_ind = 0;
uint8_t cnt=0;
uint8_t rx_buf[64] = {0};
uint8_t door_status=0;
uint8_t lock_status=0;
void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		if(USART_GetFlagStatus(USART1, USART_FLAG_PE) == RESET)
		{
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
void cmd_dump(uint8_t *data)
{
	uint8_t resp[8] = {0};
	uint16_t crc;
	uint8_t i;
	printf("<======================================\r\n");
	switch (data[0])
	{
		case 0x00:
			printf("Light %d %d, %s\r\n",
					data[1],data[2],(data[3]==1)?"on":"off");
			set7219(data[1],data[2],data[3]);
			break;
		case 0x01:
			printf("All led %s\r\n", (data[1]==1)?"on":"off");
			ctl_7219(data[1]);
			break;
		case 0x02:
			printf("Horse lantern\r\n");
			horse();
			break;
		case 0x03:
			printf("blink %d %d, %d sec\r\n",
					data[1],data[2],data[3]);
			blink(data[1],data[2],data[3]);
			break;
		case 0x04:
			printf("ask door status\r\n");
			resp[0] = MSG_HEAD0;resp[1] = MSG_HEAD1;
			resp[2] = 4; resp[3] = 0x84;
			resp[4] = door_status;
			crc = CRC_check(resp, 5);
			resp[5] = (crc >> 8) & 0xff;
			resp[6] = crc & 0xff;
			for (i=0; i<7; i++)
				PutChar(resp[i]);
			break;
		case 0x05:
			printf("ask lock status\r\n");
			resp[0] = MSG_HEAD0;resp[1] = MSG_HEAD1;
			resp[2] = 4; resp[3] = 0x85;
			resp[4] = lock_status;
			crc = CRC_check(resp, 5);
			resp[5] = (crc >> 8) & 0xff;
			resp[6] = crc & 0xff;
			for (i=0; i<7; i++)
				PutChar(resp[i]);
			break;
		case 0x06:
			printf("%s door\r\n", (data[1]==1)?"lock":"unlock");
			lock_door(data[1]);
			break;
		default:
			printf("unknown cmd\r\n");
			break;
	}
	printf("======================================>\r\n");
}
/* cmd format
 * 0x6c 0xaa len xx xx ... crc0 crc1
 */
void handle_cmd(uint8_t *cmd, uint8_t len)
{
	uint8_t packet_len;
	if (cmd[0] != MSG_HEAD0 || cmd[1] != MSG_HEAD1) {
		printf("invaild cmd\r\n");
		return;
	}

	packet_len = cmd[2];
	uint16_t crc = CRC_check(cmd, packet_len+1);
	if (crc != ((cmd[packet_len+1]<<8)|cmd[packet_len+2])) {
		printf("crc failed %04x %04x\r\n", crc,
				((cmd[packet_len+1]<<8)|cmd[packet_len+2]));
		return;
	}
	cmd_dump(cmd+3);
}

int main(void)
{	
	int i;
	EXTI6_Config();
	led_init();
	delay_init(48);
	Uart_Init();
	Init_MAX7219();
	printf("float system on\r\n");
	led(0);
	while(1) {
		led(0);
		__WFI();
		if (uart_rx_ind) {
			printf("got cmd[%d]: ",cnt);
			for (i=0; i<cnt; i++)
				printf("%02x ", rx_buf[i]);
			printf("\r\n");
			handle_cmd(rx_buf, cnt);
			uart_rx_ind = 0;			
			cnt=0;
		}
	}
}
