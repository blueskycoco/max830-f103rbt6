#include <rtthread.h>
#include "ili9325.h"
#include "tslib.h"
#include "font.h"
#include "mymisc.h"
#include <stm32f30x.h>

#define KEY1_PIN    GPIO_Pin_5//GET_PIN(C, 5)
#define KEY2_PIN    GPIO_Pin_2//GET_PIN(D, 2)

#define PEN_PIN    	GPIO_Pin_1//GET_PIN(C, 1)
#define CS_PIN    	GPIO_Pin_13//GET_PIN(C, 13)
#define MOSI_PIN    GPIO_Pin_3//GET_PIN(C, 3)
#define MISO_PIN    GPIO_Pin_2//GET_PIN(C, 2)
#define SCLK_PIN    GPIO_Pin_0//GET_PIN(C, 0)
unsigned int color = RED;
static struct rt_semaphore touch_sem;
uint8_t j = 0;
int g_need_bytes = 0;
uint8_t cal_finished = 0;
extern struct tsdev *ts;
static struct rt_thread touch_thread;
ALIGN(RT_ALIGN_SIZE)
static char touch_thread_stack[512];

static void stm32_udelay(rt_uint32_t us)
{
	rt_uint32_t ticks;
	rt_uint32_t told, tnow, tcnt = 0;
	rt_uint32_t reload = SysTick->LOAD;

	ticks = us * reload / (1000000 / RT_TICK_PER_SECOND);
	told = SysTick->VAL;
	while (1)
	{
		tnow = SysTick->VAL;
		if (tnow != told)
		{
			if (tnow < told)
			{
				tcnt += told - tnow;
			}
			else
			{
				tcnt += reload - tnow + told;
			}
			told = tnow;
			if (tcnt >= ticks)
			{
				break;
			}
		}
	}
}
static uint32_t spi_gpio_rw_ext(uint8_t cmd)
{
	int i;

	uint8_t tdata = cmd;
	uint32_t rdata = 0;

	GPIO_ResetBits(GPIOC, CS_PIN);

	for (i = 0; i < 24; i++) {
		if (tdata & 0x80)
			GPIO_SetBits(GPIOC, MOSI_PIN);
		else
			GPIO_ResetBits(GPIOC, MOSI_PIN);
		GPIO_ResetBits(GPIOC, SCLK_PIN);

		rdata = rdata << 1;
		if (GPIO_ReadInputDataBit(GPIOC, MISO_PIN) == SET)
			rdata |= 0x01;

		GPIO_SetBits(GPIOC, SCLK_PIN);
		tdata = tdata << 1;
	}

	GPIO_SetBits(GPIOC, CS_PIN);

	return rdata;
}
#if 0
static uint16_t spi_gpio_rw(uint8_t cmd)
{
	int i;

	uint8_t tdata = cmd;
	uint16_t rdata = 0;

	GPIO_ResetBits(GPIOC, CS_PIN);
	stm32_udelay(10);

	for (i = 0; i < 8; i++) {
		if (tdata & 0x80)
			GPIO_SetBits(GPIOC, MOSI_PIN);
		else
			GPIO_ResetBits(GPIOC, MOSI_PIN);
		GPIO_ResetBits(GPIOC, SCLK_PIN);
		stm32_udelay(10);
		GPIO_SetBits(GPIOC, SCLK_PIN);
		stm32_udelay(10);
		tdata = tdata << 1;
	}

	for (i = 0; i < 16; i++) {
		rdata = rdata << 1;
		GPIO_ResetBits(GPIOC, SCLK_PIN);
		stm32_udelay(10);
		if (GPIO_ReadInputDataBit(GPIOC, MISO_PIN) == SET)
			rdata |= 0x01;
		GPIO_SetBits(GPIOC, SCLK_PIN);
		stm32_udelay(10);
	}
	stm32_udelay(10);
	GPIO_SetBits(GPIOC, CS_PIN);

	return rdata;
}
#endif
int dev_touchscreen_read(struct ts_sample *samp, int nr)
{
	static int x,y,pressure;
	g_need_bytes = nr;
	j = 0;
	if (GPIO_ReadInputDataBit(GPIOC, PEN_PIN) == RESET) {
		x = spi_gpio_rw_ext(0x90);
		y = spi_gpio_rw_ext(0xd0);
		pressure = 200;
	} else
		pressure = 0;
	samp->x = x;
	samp->y = y;
	samp->pressure = pressure;
	g_need_bytes = 0;
	return nr;
}
void EXTI9_5_IRQHandler(void)
{
	rt_interrupt_enter();
	if(EXTI_GetITStatus(EXTI_Line5) != RESET)
	{
		if (RESET == GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_5))
			rt_sem_release(&touch_sem);
		EXTI_ClearITPendingBit(EXTI_Line5);
	}
	rt_interrupt_leave();
}
void EXTI1_IRQHandler(void)
{
	rt_interrupt_enter();
	if(EXTI_GetITStatus(EXTI_Line1) != RESET)
	{
		if (RESET == GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1))
			rt_sem_release(&touch_sem);
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
	rt_interrupt_leave();
}
void EXTI2_TS_IRQHandler(void)
{
	rt_interrupt_enter();
	if(EXTI_GetITStatus(EXTI_Line2) != RESET)
	{
		if (RESET == GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_2))
			rt_sem_release(&touch_sem);
		EXTI_ClearITPendingBit(EXTI_Line2);
	}
	rt_interrupt_leave();
}
static void pin_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = CS_PIN | MOSI_PIN | SCLK_PIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Pin = MISO_PIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}
static void irq_init()
{
	EXTI_InitTypeDef   EXTI_InitStructure;
	GPIO_InitTypeDef   GPIO_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

	GPIO_InitStructure.GPIO_Pin  = PEN_PIN | KEY1_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin  = KEY2_PIN;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource5);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource1);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource2);

	EXTI_InitStructure.EXTI_Line = EXTI_Line1;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line2;
	EXTI_Init(&EXTI_InitStructure);
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line5;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_EnableIRQ(EXTI9_5_IRQn);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_EnableIRQ(EXTI1_IRQn);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_TS_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_EnableIRQ(EXTI2_TS_IRQn);
}
extern int ts_calibrate(void);
static void ads7843_handler()
{
	struct ts_sample samp;
	pin_init();
	rt_sem_init(&touch_sem, "touch", 0, RT_IPC_FLAG_FIFO);
	//rt_pin_mode(KEY1_PIN, PIN_MODE_INPUT);
	//rt_pin_mode(KEY2_PIN, PIN_MODE_INPUT);
	//rt_pin_mode(PEN_PIN, PIN_MODE_INPUT);
	//rt_pin_mode(MISO_PIN, PIN_MODE_INPUT);
	//rt_pin_mode(MOSI_PIN, PIN_MODE_OUTPUT);
	//rt_pin_mode(SCLK_PIN, PIN_MODE_OUTPUT);
	//rt_pin_mode(CS_PIN, PIN_MODE_OUTPUT);

	GPIO_ResetBits(GPIOC, SCLK_PIN);
	GPIO_SetBits(GPIOC, CS_PIN);
	GPIO_SetBits(GPIOC, MOSI_PIN);
	GPIO_SetBits(GPIOC, SCLK_PIN);

	//rt_pin_attach_irq(PEN_PIN, PIN_IRQ_MODE_FALLING, touch_isr, RT_NULL);
	//rt_pin_attach_irq(KEY1_PIN, PIN_IRQ_MODE_FALLING, touch_isr, RT_NULL);
	//rt_pin_attach_irq(KEY2_PIN, PIN_IRQ_MODE_FALLING, touch_isr, RT_NULL);
	//rt_pin_irq_enable(PEN_PIN, RT_TRUE);
	//rt_pin_irq_enable(KEY1_PIN, RT_TRUE);
	//rt_pin_irq_enable(KEY2_PIN, RT_TRUE);
	irq_init();
	rt_kprintf("touch inited\r\n");

	while (1) {
		rt_sem_take(&touch_sem, RT_WAITING_FOREVER);

		if (GPIO_ReadInputDataBit(GPIOC, KEY1_PIN) == RESET) {
			rt_kprintf("Key 1 pressed\r\n");
			fb_clr(BLACK);
		}

		if (GPIO_ReadInputDataBit(GPIOD, KEY2_PIN) == RESET) {
			rt_kprintf("Key 2 pressed\r\n");
			if (color == RED)
				color = BLUE;
			else if (color == BLUE)
				color = WHITE;
			else if (color == WHITE)
				color = GREEN;
			else if (color == GREEN)
				color = BLACK;
			else if (color == BLACK)
				color = RED;
			put_string(200, 300, "Hello", color);
		}

		while (GPIO_ReadInputDataBit(GPIOC, PEN_PIN) == RESET && cal_finished) {
			ts_read(ts, &samp, 1);
			set_pixel(color, samp.x, samp.y);	
		}
		if (cal_finished) {
			ts_read(ts, &samp, 1);
			set_pixel(color, samp.x, samp.y);	
		}
	}
}

void ads7843_init()
{
	rt_thread_t tid = &touch_thread;
	rt_err_t result = rt_thread_init(&touch_thread,
			"touch",
			ads7843_handler, RT_NULL,
			&touch_thread_stack[0], sizeof(touch_thread_stack),
			3, 10);
	if (tid != RT_NULL && result == RT_EOK)
		rt_thread_startup(tid);
}
