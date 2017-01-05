#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stm32f10x.h>
#include "max830.h"
SPI_InitTypeDef   SPI_InitStructure;
USART_InitTypeDef USART_InitStructure;
static unsigned char  fac_us=0;
static unsigned short fac_ms=0;
void delay_ms(unsigned short nms);
void delay_us(unsigned long Nus);

void DBG_PutChar(char ptr)
{   
	USART_SendData(USART2, ptr);
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET); 
}
void Debug_uart_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART2, &USART_InitStructure);

	USART_Cmd(USART2, ENABLE);
}

void Spi_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI2, &SPI_InitStructure);
	//SPI_RxFIFOThresholdConfig(SPI1, SPI_RxFIFOThreshold_HF);
    //SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);
    /* Enable NSS output for master mode */
   // SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_ERR, ENABLE);
    //SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_TXE, ENABLE);
 	SPI_Cmd(SPI2, ENABLE);
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_SetBits(GPIOB,GPIO_Pin_12);
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_SetBits(GPIOC,GPIO_Pin_8);
	GPIO_SetBits(GPIOC,GPIO_Pin_6);
	delay_ms(100);
	GPIO_ResetBits(GPIOC,GPIO_Pin_6);
	delay_ms(100);
	GPIO_SetBits(GPIOC,GPIO_Pin_6);

}
void led_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);

	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
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

int spi_wirte(uint8_t addr, uint8_t data)
{
	GPIO_ResetBits(GPIOB, GPIO_Pin_12);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET){}
	SPI_I2S_SendData(SPI2, addr);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET){}
	SPI_I2S_SendData(SPI2, data);
	delay_us(15);
	GPIO_SetBits(GPIOB, GPIO_Pin_12);
	return 0;
}
int spi_read(uint8_t addr, uint8_t *data)
{
	GPIO_ResetBits(GPIOB, GPIO_Pin_12);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET){}
	SPI_I2S_SendData(SPI2, addr);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET){}
	SPI_I2S_SendData(SPI2, 0x00);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET){}
	*data = SPI_I2S_ReceiveData(SPI2);
	delay_us(15);
	GPIO_SetBits(GPIOB, GPIO_Pin_12);
	return 1;
}
int max14830_detect()
{
	uint8_t val = 0;
	int ret;
	printf("spi write 0x1f 0xce\r\n");
	//GPIO_ResetBits(GPIOB, GPIO_Pin_12);

	ret = spi_wirte(MAX310X_GLOBALCMD_REG,
			   MAX310X_EXTREG_ENBL);
	if (ret)
		return ret;
	spi_read(MAX310X_REVID_EXTREG, &val);
	spi_wirte(MAX310X_GLOBALCMD_REG, MAX310X_EXTREG_DSBL);
	//GPIO_SetBits(GPIOB, GPIO_Pin_12);
	printf("spi read 0x05 %x\r\n",val);
	if (((val & MAX310x_REV_MASK) != MAX14830_REV_ID)) {
		printf("max14830 ID 0x%02x does not match\r\n", val);
		return -1;
	}
	else
		printf("max14830 found\r\n");

	return 0;
}

int main(void)
{	
	led_init();
	delay_init(72);
	Debug_uart_Init();
	//printf("in main\r\n");	
	Spi_Init();
	max14830_detect();
	while(1)
	{
		GPIO_SetBits(GPIOA,GPIO_Pin_5);
		//printf("lcd on\r\n");
		delay_ms(1000);
		GPIO_ResetBits(GPIOA,GPIO_Pin_5);
		//printf("lcd off\r\n");
		delay_ms(1000);
	}
	
}
