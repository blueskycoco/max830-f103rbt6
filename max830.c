#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stm32f10x.h>
#include "max830.h"
#include "app_types.h"
#include "lcd_driver.h"
SPI_InitTypeDef   SPI_InitStructure;
USART_InitTypeDef USART_InitStructure;
static unsigned char  fac_us=0;
static unsigned short fac_ms=0;
unsigned long max18430_xtal = 3686400;
unsigned char buf[1024];
void delay_ms(unsigned short nms);
void delay_us(unsigned long Nus);

void DBG_PutChar(char ptr)
{   
	USART_SendData(USART1, ptr);
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); 
}
void Debug_uart_Init()
{
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

void led_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);

	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2|GPIO_Pin_3;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
}
void delay_init(unsigned char SYSCLK)
{
	SysTick->VAL=0X00000000;
	SysTick->CTRL&=0xfffffffb;
	fac_us=SYSCLK/8;      
	fac_ms=(unsigned short)fac_us*1000;
	SysTick->CTRL&=0XFFFFFFFE;
	SysTick->VAL=0X00000000;

}            
void delay_ms(unsigned short nms)
{    
	SysTick->LOAD=(unsigned long)nms*fac_ms;
	SysTick->CTRL|=0x01;
	while(!(SysTick->CTRL&(1<<16)));
	SysTick->CTRL&=0XFFFFFFFE;
	SysTick->VAL=0X00000000; 
}   
void delay_us(unsigned long Nus)
{ 
	SysTick->LOAD=Nus*fac_us;
	SysTick->CTRL|=0x01;
	while(!(SysTick->CTRL&(1<<16)));
	SysTick->CTRL=0X00000000;
	SysTick->VAL=0X00000000;
} 

int main(void)
{	
	uint16_t  x1 = 0, y1 = 0, x2 = 0, y2 = 0;
	uint16  color[8] = {WHITE,BLACK,BLUE,BLACK,RED,BLACK,GREEN,BLACK};
	char    str[6] = {0};
	led_init();
	delay_init(72);
	Debug_uart_Init();
	TFT_Init();
	GUI_PutString(FONT_16, 10, 20, "Welcome! 保单", WHITE, BLACK);
	GUI_PutString(FONT_14, 10, 60, "Welcome! 模", WHITE, BLACK);
	GUI_PutString(FONT_24, 10, 90, "Welcome! 波源", WHITE, BLACK);
	GUI_PutString(FONT_35, 10, 120, "Welcome! 频率", WHITE, BLACK);
	GUI_PutString(FONT_64, 10, 180, "23.5", WHITE, BLACK);

	x1 = 10;
	y1 = 300;

	x2 = 100;
	y2 = 400;
	GUI_HLine(x1, y1, x2, WHITE);
	GUI_VLine(x1, y1, y2, WHITE);
	GUI_Line(x1, y1, x2, y2, WHITE);
	x1 = 1;
	while(1)
	{
		GPIO_SetBits(GPIOE,GPIO_Pin_2);
		delay_ms(1000);
		GPIO_ResetBits(GPIOE,GPIO_Pin_2);
		delay_ms(1000);
		//printf("we are here , hello\r\n");
		x1++;
		sprintf(str, "%d", x1);
		GUI_PutString(FONT_24, 10, 420, str, WHITE, BLACK);

		GUI_DispColor(200, 260, 380, 440, color[x1 % 8]);
	}

}
