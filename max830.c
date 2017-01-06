#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stm32f10x.h>
#include "max830.h"
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
	GPIO_SetBits(GPIOB,GPIO_Pin_12);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI2, &SPI_InitStructure);
	
 	SPI_Cmd(SPI2, ENABLE);
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	
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

void max14830_int_init()
{
	EXTI_InitTypeDef   EXTI_InitStructure;
	GPIO_InitTypeDef   GPIO_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	/* Enable GPIOG clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	/* Configure PG.08 pin as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Enable AFIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	/* Connect EXTI8 Line to PG.08 pin */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource5);

	/* Configure EXTI8 line */
	EXTI_InitStructure.EXTI_Line = EXTI_Line5;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI9_5 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);

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

int spi_write(uint8_t addr, uint8_t data)
{
	GPIO_ResetBits(GPIOB, GPIO_Pin_12);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET){}
	SPI_I2S_SendData(SPI2, 0x80|addr);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET){}
	SPI_I2S_ReceiveData(SPI2);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET){}
	SPI_I2S_SendData(SPI2, data);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET){}
	SPI_I2S_ReceiveData(SPI2);
	GPIO_SetBits(GPIOB, GPIO_Pin_12);
	return 0;
}
int spi_read(uint8_t addr, uint8_t *data)
{
	GPIO_ResetBits(GPIOB, GPIO_Pin_12);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET){}
	SPI_I2S_SendData(SPI2, addr);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET){}
	SPI_I2S_ReceiveData(SPI2);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET){}
	SPI_I2S_SendData(SPI2, 0x00);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET){}
	*data = SPI_I2S_ReceiveData(SPI2);
	GPIO_SetBits(GPIOB, GPIO_Pin_12);
	return 1;
}
int max14830_detect()
{
	uint8_t val = 0;
	int ret;

	ret = spi_write(MAX310X_GLOBALCMD_REG,
			   MAX310X_EXTREG_ENBL);
	if (ret)
		return ret;
	spi_read(MAX310X_REVID_EXTREG, &val);
	spi_write(MAX310X_GLOBALCMD_REG, MAX310X_EXTREG_DSBL);

	if (((val & MAX310x_REV_MASK) != MAX14830_REV_ID)) {
		printf("max14830 ID 0x%02x does not match\r\n", val);
		return -1;
	}
	else
		printf("max14830 found\r\n");

	return 0;
}
int max310x_update_best_err(unsigned long f, long *besterr)
{
	/* Use baudrate 115200 for calculate error */
	long err = f % (115200 * 16);

	if ((*besterr < 0) || (*besterr > err)) {
		*besterr = err;
		return 0;
	}

	return 1;
}
unsigned long DIV_ROUND_CLOSEST(unsigned long x, unsigned int divisor)
{
         unsigned int __divisor = divisor;
		 unsigned long ret;
		 ret = (((x)+ ((__divisor) / 2)) / (__divisor));

		 printf("DIV_ROUND_CLOSEST %ld, %d, %ld\r\n",x,divisor,ret);
		 return ret;
}

int max310x_set_ref_clk(unsigned long freq,
			       bool xtal)
{
	unsigned int div, clksrc, pllcfg = 0;
	long besterr = -1;
	unsigned long fdiv, fmul, bestfreq = freq;

	/* First, update error without PLL */
	max310x_update_best_err(freq, &besterr);

	/* Try all possible PLL dividers */
	for (div = 1; (div <= 63) && besterr; div++) {
		fdiv = DIV_ROUND_CLOSEST(freq, div);

		/* Try multiplier 6 */
		fmul = fdiv * 6;
		if ((fdiv >= 500000) && (fdiv <= 800000))
			if (!max310x_update_best_err(fmul, &besterr)) {
				pllcfg = (0 << 6) | div;
				bestfreq = fmul;
			}
		/* Try multiplier 48 */
		fmul = fdiv * 48;
		if ((fdiv >= 850000) && (fdiv <= 1200000))
			if (!max310x_update_best_err(fmul, &besterr)) {
				pllcfg = (1 << 6) | div;
				bestfreq = fmul;
			}
		/* Try multiplier 96 */
		fmul = fdiv * 96;
		if ((fdiv >= 425000) && (fdiv <= 1000000))
			if (!max310x_update_best_err(fmul, &besterr)) {
				pllcfg = (2 << 6) | div;
				bestfreq = fmul;
			}
		/* Try multiplier 144 */
		fmul = fdiv * 144;
		if ((fdiv >= 390000) && (fdiv <= 667000))
			if (!max310x_update_best_err(fmul, &besterr)) {
				pllcfg = (3 << 6) | div;
				bestfreq = fmul;
			}
	}

	/* Configure clock source */
	clksrc = xtal ? MAX310X_CLKSRC_CRYST_BIT : MAX310X_CLKSRC_EXTCLK_BIT;

	/* Configure PLL */
	if (pllcfg) {
		clksrc |= MAX310X_CLKSRC_PLL_BIT;
		spi_write( MAX310X_PLLCFG_REG, pllcfg);
	} else
		clksrc |= MAX310X_CLKSRC_PLLBYP_BIT;

	spi_write( MAX310X_CLKSRC_REG, clksrc);

	/* Wait for crystal */
	if (pllcfg && xtal)
		delay_ms(10);

	return (int)bestfreq;
}
void spi_update_bits(uint8_t reg, uint8_t mask, uint8_t val)
{
	uint8_t tmp, orig;
	spi_read(reg, &orig);

	tmp = orig & ~mask;
    tmp |= val & mask;

	if (tmp != orig)
		spi_write(reg, tmp);
}
int max310x_set_baud(int port,int baud)
{
	unsigned int mode = 0, clk = max18430_xtal, div = clk / baud;

	/* Check for minimal value for divider */
	if (div < 16)
		div = 16;

	if (clk % baud && (div / 16) < 0x8000) {
		/* Mode x2 */
		mode = MAX310X_BRGCFG_2XMODE_BIT;
		clk = max18430_xtal * 2;
		div = clk / baud;

		if (clk % baud && (div / 16) < 0x8000) {
			/* Mode x4 */
			mode = MAX310X_BRGCFG_4XMODE_BIT;
			clk = max18430_xtal * 4;
			div = clk / baud;
		}
	}

	spi_write( MAX310X_BRGDIVMSB_REG + port*0x20, (div / 16) >> 8);
	spi_write( MAX310X_BRGDIVLSB_REG + port*0x20, div / 16);
	spi_write( MAX310X_BRGCFG_REG + port*0x20, (div % 16) | mode);

	return DIV_ROUND_CLOSEST(clk, div);
}

#if 1
void max310x_set_termios(int port,
				uint8_t lcr,
				uint8_t xon1,
				uint8_t xoff1,
				uint8_t flow,
				int baud)
{
#if 1
	spi_write(MAX310X_LCR_REG + port*0x20, lcr);
	spi_write(MAX310X_XON1_REG + port*0x20, xon1);
	spi_write(MAX310X_XOFF1_REG + port*0x20, xoff1);
	spi_write(MAX310X_FLOWCTRL_REG + port*0x20, flow);
	max310x_set_baud(port, baud);
#else
	unsigned int lcr, flow = 0;
	int baud;

	/* Mask termios capabilities we don't support */
	termios->c_cflag &= ~CMSPAR;

	/* Word size */
	switch (termios->c_cflag & CSIZE) {
	case CS5:
		lcr = MAX310X_LCR_WORD_LEN_5;
		break;
	case CS6:
		lcr = MAX310X_LCR_WORD_LEN_6;
		break;
	case CS7:
		lcr = MAX310X_LCR_WORD_LEN_7;
		break;
	case CS8:
	default:
		lcr = MAX310X_LCR_WORD_LEN_8;
		break;
	}

	/* Parity */
	if (termios->c_cflag & PARENB) {
		lcr |= MAX310X_LCR_PARITY_BIT;
		if (!(termios->c_cflag & PARODD))
			lcr |= MAX310X_LCR_EVENPARITY_BIT;
	}

	/* Stop bits */
	if (termios->c_cflag & CSTOPB)
		lcr |= MAX310X_LCR_STOPLEN_BIT; /* 2 stops */

	/* Update LCR register */
	max310x_port_write(port, MAX310X_LCR_REG, lcr);

	/* Set read status mask */
	port->read_status_mask = MAX310X_LSR_RXOVR_BIT;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= MAX310X_LSR_RXPAR_BIT |
					  MAX310X_LSR_FRERR_BIT;
	if (termios->c_iflag & (IGNBRK | BRKINT | PARMRK))
		port->read_status_mask |= MAX310X_LSR_RXBRK_BIT;

	/* Set status ignore mask */
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNBRK)
		port->ignore_status_mask |= MAX310X_LSR_RXBRK_BIT;
	if (!(termios->c_cflag & CREAD))
		port->ignore_status_mask |= MAX310X_LSR_RXPAR_BIT |
					    MAX310X_LSR_RXOVR_BIT |
					    MAX310X_LSR_FRERR_BIT |
					    MAX310X_LSR_RXBRK_BIT;

	/* Configure flow control */
	max310x_port_write(port, MAX310X_XON1_REG, termios->c_cc[VSTART]);
	max310x_port_write(port, MAX310X_XOFF1_REG, termios->c_cc[VSTOP]);
	if (termios->c_cflag & CRTSCTS)
		flow |= MAX310X_FLOWCTRL_AUTOCTS_BIT |
			MAX310X_FLOWCTRL_AUTORTS_BIT;
	if (termios->c_iflag & IXON)
		flow |= MAX310X_FLOWCTRL_SWFLOW3_BIT |
			MAX310X_FLOWCTRL_SWFLOWEN_BIT;
	if (termios->c_iflag & IXOFF)
		flow |= MAX310X_FLOWCTRL_SWFLOW1_BIT |
			MAX310X_FLOWCTRL_SWFLOWEN_BIT;
	max310x_port_write(port, MAX310X_FLOWCTRL_REG, flow);

	/* Get baud rate generator configuration */
	baud = uart_get_baud_rate(port, termios, old,
				  port->uartclk / 16 / 0xffff,
				  port->uartclk / 4);

	/* Setup baudrate generator */
	baud = max310x_set_baud(port, baud);

	/* Update timeout according to new baud rate */
	uart_update_timeout(port, termios->c_cflag, baud);
#endif
}
#endif
int max310x_startup(int port)
{
	uint8_t val;

	/* Configure MODE1 register */
	spi_update_bits( MAX310X_MODE1_REG + port*0x20,
			    MAX310X_MODE1_TRNSCVCTRL_BIT, 0);

	/* Configure MODE2 register & Reset FIFOs*/
	val = MAX310X_MODE2_RXEMPTINV_BIT | MAX310X_MODE2_FIFORST_BIT;
	spi_write( MAX310X_MODE2_REG + port*0x20, val);
	spi_update_bits( MAX310X_MODE2_REG + port*0x20,
			    MAX310X_MODE2_FIFORST_BIT, 0);

	/* Configure flow control levels */
	/* Flow control halt level 96, resume level 48 */
	spi_write( MAX310X_FLOWLVL_REG + port*0x20,
			   MAX310X_FLOWLVL_RES(48) | MAX310X_FLOWLVL_HALT(96));

	/* Clear IRQ status register */
	spi_read(MAX310X_IRQSTS_REG + port*0x20, &val);

	/* Enable RX, TX, CTS change interrupts */
	val = MAX310X_IRQ_RXEMPTY_BIT | MAX310X_IRQ_TXEMPTY_BIT;
	spi_write( MAX310X_IRQEN_REG + port*0x20, val | MAX310X_IRQ_CTS_BIT);

	return 0;
}
unsigned int max310x_tx_empty(int port)
{
	uint8_t lvl, sts;

	spi_read(MAX310X_TXFIFOLVL_REG + port*0x20, &lvl);
	spi_read(MAX310X_IRQSTS_REG + port*0x20, &sts);

	return ((sts & MAX310X_IRQ_TXEMPTY_BIT) && !lvl) ? 1 : 0;
}
int max14830_tx(int port, unsigned char *buf, int len)
{
		uint8_t txlen,to_send;
		int i=0;
		spi_read(MAX310X_TXFIFOLVL_REG+port*0x20,&txlen);
		txlen = MAX310X_FIFO_SIZE - txlen;
		to_send = (len > txlen) ? txlen : len;

		while (to_send--) {
			spi_write(MAX310X_THR_REG+port*0x20,
					   buf[i]);
			i++;
		}
		return (len > txlen) ? txlen : len;
}
void max14830_rx(int port, unsigned char *buf, unsigned int rxlen)
{
	unsigned char ch;
	int i = 0;

	printf("port %d ,rx %d bytes\r\n",port,rxlen);
	while (rxlen--) {
		spi_read(MAX310X_RHR_REG + port*0x20, &ch);
		buf[i]=ch;
		i++;
		printf("%x ",ch);
		}
	printf("\r\n");
}

void max14830_init()
{
	int i,uartclk;
	unsigned long freq = max18430_xtal;
	bool xtal = true;
	uint8_t ret;
	Spi_Init();
	if(max14830_detect()) {
		printf("Can't find max14830\n");
		return ;
	}
	for (i = 0; i < 4; i++) {
		unsigned int offs = i << 5;

		/* Reset port */
		spi_write( MAX310X_MODE2_REG + offs,
				 MAX310X_MODE2_RST_BIT);
		/* Clear port reset */
		spi_write( MAX310X_MODE2_REG + offs, 0);

		/* Wait for port startup */
		do {
			spi_read(
					MAX310X_BRGDIVLSB_REG + offs, &ret);
		} while (ret != 0x01);

		spi_update_bits(MAX310X_MODE1_REG + offs,
				   MAX310X_MODE1_AUTOSLEEP_BIT,
				   MAX310X_MODE1_AUTOSLEEP_BIT);
	}
	uartclk = max310x_set_ref_clk(freq, xtal);
	printf("Reference clock set to %i Hz\r\n", uartclk);

	for (i = 0; i < 4; i++) {
		unsigned int offs = i << 5;
		/* Disable all interrupts */
		spi_write(MAX310X_IRQEN_REG + offs, 0);
		/* Clear IRQ status register */
		spi_read(MAX310X_IRQSTS_REG + offs, &ret);
		/* Enable IRQ pin */
		spi_update_bits(MAX310X_MODE1_REG + offs,
				    MAX310X_MODE1_IRQSEL_BIT,
				    MAX310X_MODE1_IRQSEL_BIT);

		max310x_set_termios(
			i, 
			MAX310X_LCR_WORD_LEN_8,
			0x00,
			0x00,
			0x00,
			115200);
		max310x_startup(i);
	}
	max14830_int_init();
}
void EXTI9_5_IRQHandler(void)
{
	int i;
	if(EXTI_GetITStatus(EXTI_Line5) != RESET)
	{
		for (i = 0; i < 4; i++) {
			do {
				unsigned char ists, lsr, rxlen;

				/* Read IRQ status & RX FIFO level */
				spi_read( MAX310X_IRQSTS_REG+i*0x20, &ists);
				spi_read( MAX310X_RXFIFOLVL_REG+i*0x20, &rxlen);
				if (!ists && !rxlen)
					break;

				printf("port %d irq ists %x, rxlen %d \r\n",i,ists,rxlen);

				if (ists & MAX310X_IRQ_CTS_BIT) {
					spi_read( MAX310X_LSR_IRQSTS_REG+i*0x20, &lsr);
					//uart_handle_cts_change(port,
					//			   !!(lsr & MAX310X_LSR_CTS_BIT));
				}
				if (rxlen)
					max14830_rx(i, buf, rxlen);
				if (ists & MAX310X_IRQ_TXEMPTY_BIT) {
					printf("port %d send done\r\n",i);
				}
			} while (1);
		}

		/* Clear the  EXTI line 8 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line5);
	}
	
}
int main(void)
{	
	int i=0;
	led_init();
	delay_init(72);
	Debug_uart_Init();
	max14830_init();
	for(i=0;i<1024;i++)
		buf[i]=i;
	
	while(1)
	{
		max14830_tx(i,buf,100);
		GPIO_SetBits(GPIOA,GPIO_Pin_5);
		delay_ms(1000);
		GPIO_ResetBits(GPIOA,GPIO_Pin_5);
		delay_ms(1000);
		i++;
		if(i==4)
			i=0;
	}
	
}
