#ifndef _MY_MISC_H
#define _MY_MISC_H
void delay_ms(unsigned short nms);
void delay_us(unsigned long Nus);
void delay_init(unsigned char SYSCLK);
void led_init();
void led(int on);
void Uart_Init();
void PutChar(char ptr);
void EXTI6_Config(void);
int8_t GetChar(void);
void Init_MAX7219();
void set7219(uint8_t x, uint8_t y, uint8_t on);
unsigned int CRC_check(unsigned char *Data,unsigned short Data_length);
#endif
