#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stm32f0xx.h>
#include <string.h>
#include "mymisc.h"
#define TIMES 200
extern uint8_t door_status;
extern uint8_t lock_status;
extern volatile uint8_t uart_rx_ind;
USART_InitTypeDef USART_InitStructure;
static unsigned char  fac_us=0;
static unsigned short fac_ms=0;
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
void PutChar(char ptr)
{   
	USART_SendData(USART1, ptr);
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); 
}
int8_t GetChar(void)
{
	int8_t ch = -1;
	if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET)
		ch = USART_ReceiveData(USART1) & 0xff;
	return ch;
}
void Uart_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = 
		USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	USART_SetReceiverTimeOut(USART1, 115200/4);
	USART_ReceiverTimeOutCmd(USART1, ENABLE);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART1, USART_IT_RTO, ENABLE);
	USART_Cmd(USART1, ENABLE);
}
void led_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOF, &GPIO_InitStructure);
}
void led(int on)
{
	if (on)
	{
		GPIO_ResetBits(GPIOB,GPIO_Pin_1);
	}
	else
	{
		GPIO_SetBits(GPIOB,GPIO_Pin_1);
	}
}
void uart_ctl(int flag)
{
	if (flag)
	{
		GPIO_ResetBits(GPIOF,GPIO_Pin_0);
		GPIO_ResetBits(GPIOF,GPIO_Pin_1);
	}
	else
	{
		GPIO_SetBits(GPIOF,GPIO_Pin_1);
		GPIO_SetBits(GPIOF,GPIO_Pin_0);
	}
}
void EXTI4_15_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line6) != RESET)
	{
		door_status = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6);
		EXTI_ClearITPendingBit(EXTI_Line6);
	}
}
void EXTI6_Config(void)
{
	EXTI_InitTypeDef   EXTI_InitStructure;
	GPIO_InitTypeDef   GPIO_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	/* Enable GPIOA clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	/* Configure PA6 pin as input floating */
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	/* Connect EXTI6 Line to PA6 pin */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource6);

	/* Configure EXTI6 line */
	EXTI_InitStructure.EXTI_Line = EXTI_Line6;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI6 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_EnableIRQ(EXTI4_15_IRQn);
}
void spi_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;
	SPI_I2S_DeInit(SPI1);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_0);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_0);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA,GPIO_Pin_4);

	SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_Init(SPI1, &(SPI_InitStructure));
	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_ERR, ENABLE);
	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, ENABLE);
	SPI_Cmd(SPI1, ENABLE);
}
static int check_status(uint8_t bit)
{
	int i=0;
	while(SPI_I2S_GetITStatus(SPI1,bit)!=SET)
	{
		i++;
		if(i==100){
			printf("send timeout\r\n");
			return 0;
		}
		delay_us(1);
	}
/*	i=0;
	while(SPI_I2S_GetITStatus(SPI1,SPI_SR_BSY)==SET)
	{
		i++;
		if(i==100){
			printf("send 2 timeout\r\n");
			return 0;
		}
		delay_us(1);
	}*/
	return 1;
}
uint8_t spi_send(uint8_t *data,int len)
{
	uint8_t result = 1;
	int i;
	GPIO_ResetBits(GPIOA,GPIO_Pin_4);
	for(i=0;i<len;i=i+1)
	{
		SPI_SendData8(SPI1, data[i]);
		check_status(SPI_I2S_IT_TXE);
	}
	//delay_us(80);
	GPIO_SetBits(GPIOA,GPIO_Pin_4);
	return result;
}
uint8_t cur[4][8] = {{0},{0},{0},{0}};
/* x from 0 - 15
 * y from 0 - 15
 * on 0/1
 * map 1 2
 *     3 4
 */
void set7219(uint8_t x, uint8_t y, uint8_t on)
{
	uint8_t i,index=0;
	uint8_t cmd[8] = {0};
	uint8_t tmp_x = x%8;
	uint8_t tmp_y = y%8;

	/* get segment for data
	 * get line for address
	 */
	if (x < 8 && x >= 0 && y < 16 && y >= 8)
		index = 0;
	else if(x < 16 && x >= 8 && y < 16 && y >= 8)
		index = 1;
	else if(x < 8 && x >= 0 && y < 8 && y >= 0)
		index = 2;
	else if(x < 16 && x >= 8 && y < 8 && y >= 0)
		index = 3;
	if (on) {
		cur[index][tmp_x] |= (1<<tmp_y);
	} else {
		cur[index][tmp_x] &= ~(1<<tmp_y);
	}
	cmd[0] = cmd[2] = cmd[4] = cmd[6] = tmp_x+1;
	for (i=1; i<8; i=i+2)
		cmd[i] = cur[(i-1)/2][tmp_x];
#ifdef DEBUG
	uint8_t j;
	printf("cmd %02x %02x %02x %02x %02x %02x %02x %02x\r\ncur:\r\n",
			cmd[0],cmd[1],cmd[2],cmd[3],cmd[4],cmd[5],cmd[6],cmd[7]);
	for (i=0; i<4; i++) {
		for (j=0; j<8; j++)
			printf("%02x ", cur[i][j]);
		printf("\r\n");
	}
#endif
	spi_send(cmd,8);
}
void ctl_7219(uint8_t on)
{
	int i;
	uint8_t cmd[8] = {0};
	if (on) {
		memset(cur[0], 0xff, 8);
		memset(cur[1], 0xff, 8);
		memset(cur[2], 0xff, 8);
		memset(cur[3], 0xff, 8);
	} else {
		memset(cur[0], 0x00, 8);
		memset(cur[1], 0x00, 8);
		memset(cur[2], 0x00, 8);
		memset(cur[3], 0x00, 8);
	}
	for (i=0; i<8; i++) {
		cmd[0] = i+1;
		cmd[2] = i+1;
		cmd[4] = i+1;
		cmd[6] = i+1;
		if (on) {
			cmd[1] = 0xff;
			cmd[3] = 0xff;
			cmd[5] = 0xff;
			cmd[7] = 0xff;
		} else {
			cmd[1] = 0x00;
			cmd[3] = 0x00;
			cmd[5] = 0x00;
			cmd[7] = 0x00;
		}
		spi_send(cmd,8);
	}
}
void horse(void)
{
	uint8_t i,j;
	ctl_7219(0);
	for (i=0; i<16; i++) {
		for (j=0; j<16; j++) {
		set7219(i,j,1);	
		if (j>0) {			
			set7219(i,j-1,0);
			} else if (i>0) {
			set7219(i-1,j,0);
			}
			delay_ms(100);
		}
		set7219(i,j-1,0);
	}
}
void blink(uint8_t x, uint8_t y, uint8_t sec)
{
	int i,j;
	if ( x >= 16 || y >= 16)
		return;

	for (i=0; i<sec; i++) {
		for(j=0; j<5; j++) {
			set7219(x,y,1);
			delay_ms(100);
			set7219(x,y,0);
			delay_ms(100);
		}
	}
}
void Init_MAX7219()
{
	uint8_t cmd0[] = {0x09,0x00,0x09,0x00,0x09,0x00,0x09,0x00};
	uint8_t cmd1[] = {0x0a,0x03,0x0a,0x03,0x0a,0x03,0x0a,0x03};
	uint8_t cmd2[] = {0x0b,0x07,0x0b,0x07,0x0b,0x07,0x0b,0x07};
	uint8_t cmd3[] = {0x0c,0x01,0x0c,0x01,0x0c,0x01,0x0c,0x01};
	uint8_t cmd4[] = {0x0f,0x00,0x0f,0x00,0x0f,0x00,0x0f,0x00};
	spi_init();
	spi_send(cmd0,8);
	spi_send(cmd1,8);
	spi_send(cmd2,8);
	spi_send(cmd3,8);
	spi_send(cmd4,8);	
}
unsigned int CRC_check(unsigned char *Data,unsigned short Data_length)
{
	unsigned int mid=0;
	unsigned char times=0;
	unsigned short Data_index=0;
	unsigned int CRC1=0xFFFF;
	while(Data_length)
	{
		CRC1=Data[Data_index]^CRC1;
		for(times=0;times<8;times++)
		{
			mid=CRC1;
			CRC1=CRC1>>1;
			if(mid & 0x0001)
			{
				CRC1=CRC1^0xA001;
			}
		}
		Data_index++;
		Data_length--;
	}
	return CRC1;
}

void lock_door(uint8_t on)
{
	uint8_t Step_f[8]={0x08,0x0c,0x04,0x06,0x02,0x03,0x01,0x09};
	uint8_t Step_r[8]={0x09,0x01,0x03,0x02,0x06,0x04,0x0c,0x08};
	uint8_t *Step;
	uint8_t i,j;
	if (lock_status && on)
		return;
	if (!lock_status && !on)
		return;
	lock_status = on;
	if (on) {
		Step = Step_f;
	} else {
		Step = Step_r;
	}
	for (j=0; j<TIMES; j++) {
		for (i=0; i<8; i++) {
			GPIO_Write(GPIOA, Step[i]);
			delay_ms(2);
		}
		delay_ms(3);
	}
}

void lock_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	door_status = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6);
	lock_door(0);
}
