#ifndef _MY_MISC_H
#define _MY_MISC_H
void delay_ms(unsigned short nms);
void delay_us(unsigned long Nus);
void delay_init(unsigned char SYSCLK);
void led_init();
void led(int on);
void Debug_uart_Init();
void DBG_PutChar(char ptr);
#endif
