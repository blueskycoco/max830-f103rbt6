#ifndef _MY_MISC_H
#define _MY_MISC_H
#define MSG_HEAD0		0x6c
#define MSG_HEAD1		0xaa
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
void ctl_7219(uint8_t on);
void blink(uint8_t x, uint8_t y, uint8_t sec);
void horse(void);
unsigned int CRC_check(unsigned char *Data,unsigned short Data_length);
void lock_door(uint8_t on);
#endif
