#include <stm32f30x.h>
#include <rtthread.h>
#include "ili9325.h"
#include "font.h"
#include "mymisc.h"
#define CS_PIN    GPIO_Pin_9//GET_PIN(C, 9)
#define RS_PIN    GPIO_Pin_8//GET_PIN(C, 8)
#define WR_PIN    GPIO_Pin_7//GET_PIN(C, 7)
#define RD_PIN    GPIO_Pin_6//GET_PIN(C, 6)
#if 0
#define FB_0	GET_PIN(B, 0)
#define FB_1	GET_PIN(B, 1)
#define FB_2	GET_PIN(B, 2)
#define FB_3	GET_PIN(B, 3)
#define FB_4	GET_PIN(B, 4)
#define FB_5	GET_PIN(B, 5)
#define FB_6	GET_PIN(B, 6)
#define FB_7	GET_PIN(B, 7)
#define FB_8	GET_PIN(B, 8)
#define FB_9	GET_PIN(B, 9)
#define FB_10	GET_PIN(B, 10)
#define FB_11	GET_PIN(B, 11)
#define FB_12	GET_PIN(B, 12)
#define FB_13	GET_PIN(B, 13)
#define FB_14	GET_PIN(B, 14)
#define FB_15	GET_PIN(B, 15)
#endif
static void pin_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 |
									GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 |
									GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 |
									GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 |
									GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 |
									GPIO_Pin_15;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_7 | GPIO_Pin_8;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void dpi_w(rt_uint16_t cmd, rt_uint16_t data)
{
	//rt_pin_write(RS_PIN, PIN_LOW);
	//rt_pin_write(RD_PIN, PIN_HIGH);
	GPIO_ResetBits(GPIOC, RS_PIN);
	GPIO_SetBits(GPIOC, RD_PIN);
	GPIOB->ODR = cmd;
	//rt_pin_write(WR_PIN, PIN_LOW);
	//rt_pin_write(WR_PIN, PIN_HIGH);
	GPIO_ResetBits(GPIOC, WR_PIN);
	GPIO_SetBits(GPIOC, WR_PIN);
	//rt_pin_write(RS_PIN, PIN_HIGH);
	//rt_pin_write(RD_PIN, PIN_HIGH);
	GPIO_SetBits(GPIOC, RS_PIN);
	GPIO_SetBits(GPIOC, RD_PIN);
	GPIOB->ODR = data;
	//rt_pin_write(WR_PIN, PIN_LOW);
	//rt_pin_write(WR_PIN, PIN_HIGH);
	GPIO_ResetBits(GPIOC, WR_PIN);
	GPIO_SetBits(GPIOC, WR_PIN);
}

void stm32_lcd_init()
{
#if 0
	rt_pin_mode(CS_PIN, PIN_MODE_OUTPUT);
	rt_pin_mode(RS_PIN, PIN_MODE_OUTPUT);
	rt_pin_mode(WR_PIN, PIN_MODE_OUTPUT);
	rt_pin_mode(RD_PIN, PIN_MODE_OUTPUT);

	rt_pin_mode(FB_0, PIN_MODE_OUTPUT);
	rt_pin_mode(FB_1, PIN_MODE_OUTPUT);
	rt_pin_mode(FB_2, PIN_MODE_OUTPUT);
	rt_pin_mode(FB_3, PIN_MODE_OUTPUT);
	rt_pin_mode(FB_4, PIN_MODE_OUTPUT);
	rt_pin_mode(FB_5, PIN_MODE_OUTPUT);
	rt_pin_mode(FB_6, PIN_MODE_OUTPUT);
	rt_pin_mode(FB_7, PIN_MODE_OUTPUT);
	rt_pin_mode(FB_8, PIN_MODE_OUTPUT);
	rt_pin_mode(FB_9, PIN_MODE_OUTPUT);
	rt_pin_mode(FB_10, PIN_MODE_OUTPUT);
	rt_pin_mode(FB_11, PIN_MODE_OUTPUT);
	rt_pin_mode(FB_12, PIN_MODE_OUTPUT);
	rt_pin_mode(FB_13, PIN_MODE_OUTPUT);
	rt_pin_mode(FB_14, PIN_MODE_OUTPUT);
	rt_pin_mode(FB_15, PIN_MODE_OUTPUT);
#endif
	pin_init();
	//rt_pin_write(CS_PIN, PIN_LOW);
	GPIO_ResetBits(GPIOC, CS_PIN);
	dpi_w(0x0001,0x0100); 
	dpi_w(0x0002,0x0700);
	dpi_w(0x0003,0x1030);

	dpi_w(0x0004,0x0000);
	dpi_w(0x0008,0x0207);
	dpi_w(0x0009,0x0000);
	dpi_w(0x000A,0x0000);
	dpi_w(0x000C,0x0000);
	dpi_w(0x000D,0x0000);
	dpi_w(0x000F,0x0000);
	rt_thread_mdelay(50);
	//delay_ms(50);
	dpi_w(0x0007,0x0101);
	rt_thread_mdelay(50);
	//delay_ms(50);
	dpi_w(0x0010,0x16B0);
	dpi_w(0x0011,0x0001);
	dpi_w(0x0017,0x0001);
	dpi_w(0x0012,0x0138);
	dpi_w(0x0013,0x0800);
	dpi_w(0x0029,0x0009);
	dpi_w(0x002a,0x0009);
	dpi_w(0x00a4,0x0000);

	dpi_w(0x0050,0x0000);
	dpi_w(0x0051,0x00EF);
	dpi_w(0x0052,0x0000);
	dpi_w(0x0053,0x013F);


	dpi_w(0x0060,0xA700);  								  
	dpi_w(0x0061,0x0003);
	dpi_w(0x006A,0x0000);

	dpi_w(0x0080,0x0000);
	dpi_w(0x0081,0x0000);
	dpi_w(0x0082,0x0000);
	dpi_w(0x0083,0x0000);
	dpi_w(0x0084,0x0000);
	dpi_w(0x0085,0x0000);
	dpi_w(0x0090,0x0013);
	dpi_w(0x0092,0x0000);
	dpi_w(0x0093,0x0003);
	dpi_w(0x0095,0x0110);
	dpi_w(0x0007,0x0173); 						
	//rt_pin_write(CS_PIN, PIN_HIGH);
	GPIO_SetBits(GPIOC, CS_PIN);
	rt_thread_mdelay(50);
	//delay_ms(50)
}

void set_pixel(const rt_uint16_t pixel, int x, int y)
{
	//rt_pin_write(CS_PIN, PIN_LOW);
	GPIO_ResetBits(GPIOC, CS_PIN);
	dpi_w(0x0020, x); 						
	dpi_w(0x0021, y); 						
	dpi_w(0x0022, pixel); 						
	//rt_pin_write(CS_PIN, PIN_HIGH);
	GPIO_SetBits(GPIOC, CS_PIN);
}

void draw_hline(const rt_uint16_t pixel, int x1, int x2, int y)
{
	int i;
	for (i = x1; i <= x2; i++)
		set_pixel(pixel, i, y);
}

void draw_vline(const rt_uint16_t pixel, int x, int y1, int y2)
{
	int i;
	for (i = y1; i <= y2; i++)
		set_pixel(pixel, x, i);
}

void fb_clr(uint16_t color)
{
	rt_uint32_t i;

	//rt_pin_write(CS_PIN, PIN_LOW);
	//rt_pin_write(RS_PIN, PIN_LOW);
	//rt_pin_write(RD_PIN, PIN_HIGH);
	GPIO_ResetBits(GPIOC, CS_PIN);
	GPIO_ResetBits(GPIOC, RS_PIN);
	GPIO_SetBits(GPIOC, RD_PIN);
	GPIOB->ODR = 0x0022;
	//rt_pin_write(WR_PIN, PIN_LOW);
	//rt_pin_write(WR_PIN, PIN_HIGH);
	GPIO_ResetBits(GPIOC, WR_PIN);
	GPIO_SetBits(GPIOC, WR_PIN);

	for( i = 0; i < 240 * 320; i++ )
	{
		//rt_pin_write(RS_PIN, PIN_HIGH);
		//rt_pin_write(RD_PIN, PIN_HIGH);
		GPIO_SetBits(GPIOC, RS_PIN);
		GPIO_SetBits(GPIOC, RD_PIN);
		GPIOB->ODR = color;
		//rt_pin_write(WR_PIN, PIN_LOW);
		//rt_pin_write(WR_PIN, PIN_HIGH);
		GPIO_ResetBits(GPIOC, WR_PIN);
		GPIO_SetBits(GPIOC, WR_PIN);
	}
	//rt_pin_write(CS_PIN, PIN_HIGH);
	GPIO_SetBits(GPIOC, CS_PIN);
}

void clr_cross(int x, int y)
{
	draw_hline(BLACK, x - 10, x - 4, y);
	draw_hline(BLACK, x + 4, x + 10, y);
	draw_vline(BLACK, x, y - 10, y - 4);
	draw_vline(BLACK, x, y + 4, y + 10);

	draw_hline(BLACK, x - 9, x - 6, y - 9);
	draw_vline(BLACK, x - 9, y - 8, y - 6);
	draw_vline(BLACK, x - 9, y + 6, y + 9);
	draw_hline(BLACK, x - 8, x - 6, y + 9);
	draw_hline(BLACK, x + 6, x + 9, y + 9);
	draw_vline(BLACK, x + 9, y + 6, y + 8);
	draw_vline(BLACK, x + 9, y - 9, y - 6);
	draw_hline(BLACK, x + 6, x + 8, y - 9);
}
void put_cross(int x, int y)
{
	draw_hline(WHITE, x - 10, x - 4, y);
	draw_hline(WHITE, x + 4, x + 10, y);
	draw_vline(WHITE, x, y - 10, y - 4);
	draw_vline(WHITE, x, y + 4, y + 10);

	draw_hline(WHITE, x - 9, x - 6, y - 9);
	draw_vline(WHITE, x - 9, y - 8, y - 6);
	draw_vline(WHITE, x - 9, y + 6, y + 9);
	draw_hline(WHITE, x - 8, x - 6, y + 9);
	draw_hline(WHITE, x + 6, x + 9, y + 9);
	draw_vline(WHITE, x + 9, y + 6, y + 8);
	draw_vline(WHITE, x + 9, y - 9, y - 6);
	draw_hline(WHITE, x + 6, x + 8, y - 9);
}
void put_char(int x, int y, int c, int colidx)
{
	int i,j,bits;
	uint8_t* p;
	

	p = (uint8_t *)font_vga_8x8.path;//need fix
	for (i = 0; i < font_vga_8x8.height; i++) 
	{
		bits =	p[font_vga_8x8.height * c + i];
		for (j = 0; j < font_vga_8x8.width; j++, bits <<= 1)
		{
			if (bits & 0x80)
			{
				set_pixel(colidx, x + j, y + i);
			}
		}
	}
}

void put_string(int x, int y, char *s, unsigned colidx)
{
	int i;
	
	for (i = 0; *s; i++, x += font_vga_8x8.width, s++)
		put_char(x, y, *s, colidx);
}

