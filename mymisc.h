#ifndef _MY_MISC_H
#define _MY_MISC_H
void delay_ms(unsigned short nms);
void delay_us(unsigned long Nus);
void delay_init(unsigned char SYSCLK);
void led_init();
void led(int on);
void Uart_Init();
void PutChar(char ptr);
void EXTI1_Config(void);
int8_t GetChar(void);
#endif
