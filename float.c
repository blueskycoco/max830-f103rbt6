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
uint8_t door_status=1;
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
#ifdef DEBUG
	printf("<======================================\r\n");
#endif
	switch (data[0])
	{
		case 0x00:
#ifdef DEBUG
			printf("Light %d %d, %s\r\n",
					data[1],data[2],(data[3]==1)?"on":"off");
#endif
			set7219(data[1],data[2],data[3]);
			break;
		case 0x01:
#ifdef DEBUG
			printf("All led %s\r\n", (data[1]==1)?"on":"off");
#endif
			ctl_7219(data[1]);
			break;
		case 0x02:
#ifdef DEBUG
			printf("Horse lantern\r\n");
#endif
			horse();
			break;
		case 0x03:
#ifdef DEBUG
			printf("blink %d %d, %d sec\r\n",
					data[1],data[2],data[3]);
#endif
			blink(data[1],data[2],data[3]);
			break;
		case 0x04:
#ifdef DEBUG
			printf("ask door status\r\n");
#endif
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
#ifdef DEBUG
			printf("ask lock status\r\n");
#endif
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
#ifdef DEBUG
			printf("%s door\r\n", (data[1]==1)?"lock":"unlock");
#endif
			lock_door(data[1]);
			break;
		default:
#ifdef DEBUG
			printf("unknown cmd\r\n");
#endif
			break;
	}
#ifdef DEBUG
	printf("======================================>\r\n");
#endif
}
/* cmd format
 * 0x6c 0xaa len xx xx ... crc0 crc1
 */
void handle_cmd(uint8_t *cmd, uint8_t len)
{
	uint8_t packet_len;
	if (cmd[0] != MSG_HEAD0 || cmd[1] != MSG_HEAD1) {
#ifdef DEBUG
		printf("invaild cmd\r\n");
#endif
		return;
	}

	packet_len = cmd[2];
	uint16_t crc = CRC_check(cmd, packet_len+1);
	if (crc != ((cmd[packet_len+1]<<8)|cmd[packet_len+2])) {
#ifdef DEBUG
		printf("crc failed %04x %04x\r\n", crc,
				((cmd[packet_len+1]<<8)|cmd[packet_len+2]));
#endif
		return;
	}
	cmd_dump(cmd+3);
}
uint8_t off[] = {0x6c,0xaa,0x04,0x01,0x00,0x80,0xd5};
uint8_t on[] = {0x6c,0xaa,0x04,0x01,0x01,0x40,0x14};
uint8_t lock[] = {0x6c,0xaa,0x04,0x06,0x00,0xb0,0xd7};
uint8_t unlock[] = {0x6c,0xaa,0x04,0x06,0x01,0x70,0x16};
uint8_t horse1[] = {0x6c,0xaa,0x03,0x02,0x65,0xbc};
int main(void)
{	
	EXTI6_Config();
	led_init();
	delay_init(36);
	Uart_Init();
	Init_MAX7219();
	uart_ctl(0);
#ifdef DEBUG
	printf("float system on\r\n");
#endif
	uart_ctl(1);
	led(0);
	lock_init();
	ctl_7219(0);
#if 0
while(1) {
	led(0);
	delay_ms(1000);
	led(1);
	delay_ms(1000);
}
	handle_cmd(on, sizeof(on));
	delay_ms(1000);
		led(0);
	handle_cmd(off, sizeof(off));
	delay_ms(1000);
		led(1);
	handle_cmd(unlock, sizeof(unlock));
	delay_ms(1000);
		led(0);
	handle_cmd(lock, sizeof(lock));
	delay_ms(1000);
		led(1);
	handle_cmd(horse1, sizeof(horse1));
	delay_ms(1000);
		led(0);
	//return 0;
#endif
	while(1) {
		led(0);
		__WFI();
		if (uart_rx_ind) {
#ifdef DEBUG
			int i;
			printf("got cmd[%d]: ",cnt);
			for (i=0; i<cnt; i++)
				printf("%02x ", rx_buf[i]);
			printf("\r\n");
#endif
			led(1);
			uart_ctl(0);
			handle_cmd(rx_buf, cnt);
			uart_ctl(1);
			uart_rx_ind = 0;			
			cnt=0;
		}
	}
}
