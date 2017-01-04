#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stm32f10x.h>
#include "max830.h"
SPI_InitTypeDef   SPI_InitStructure;
#define SPIy                   SPI1
#define SPIy_CLK               RCC_APB2Periph_SPI1
#define SPIy_GPIO              GPIOA
#define SPIy_GPIO_CLK          RCC_APB2Periph_GPIOA  
#define SPIy_PIN_SCK           GPIO_Pin_5
#define SPIy_PIN_MISO          GPIO_Pin_6
#define SPIy_PIN_MOSI          GPIO_Pin_7
#define SPIy_PIN_CS			   GPIO_Pin_4
void RCC_Configuration(void)
{
	//RCC_PCLK2Config(RCC_HCLK_Div2); 
	RCC_APB2PeriphClockCmd(SPIy_GPIO_CLK | SPIy_CLK, ENABLE);
}

void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = SPIy_PIN_SCK | SPIy_PIN_MOSI;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(SPIy_GPIO, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = SPIy_PIN_MISO;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(SPIy_GPIO, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin   = SPIy_PIN_CS;
	GPIO_Init(SPIy_GPIO, &GPIO_InitStructure);
}

void led_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);

	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

int spi_wirte(uint8_t addr, uint8_t data)
{
	GPIO_ResetBits( SPIy_GPIO, SPIy_PIN_CS );
	while (SPI_I2S_GetFlagStatus(SPIy, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPIy, addr);
	while (SPI_I2S_GetFlagStatus(SPIy, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPIy, data);
	GPIO_SetBits( SPIy_GPIO, SPIy_PIN_CS );
	return 0;
}
int spi_read(uint8_t addr, uint8_t *data)
{
	GPIO_ResetBits( SPIy_GPIO, SPIy_PIN_CS );
	while (SPI_I2S_GetFlagStatus(SPIy, SPI_I2S_FLAG_RXNE) == RESET);
	*data = SPI_I2S_ReceiveData(SPIy);
	GPIO_SetBits( SPIy_GPIO, SPIy_PIN_CS );
	return 1;
}
int max14830_detect()
{
	uint8_t val = 0;
	int ret;

	ret = spi_wirte(MAX310X_GLOBALCMD_REG,
			   MAX310X_EXTREG_ENBL);
	if (ret)
		return ret;
	
	spi_read(MAX310X_REVID_EXTREG, &val);
	spi_wirte(MAX310X_GLOBALCMD_REG, MAX310X_EXTREG_DSBL);
	if (((val & MAX310x_REV_MASK) != MAX14830_REV_ID)) {
		printf("max14830 ID 0x%02x does not match\n", val);
		return -1;
	}

	return 0;
}

int main(void)
{
	RCC_Configuration();
	GPIO_Configuration();

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPIy, &SPI_InitStructure);
 	SPI_Cmd(SPIy, ENABLE);
	max14830_detect();
	while(1)
	{

	}
}
