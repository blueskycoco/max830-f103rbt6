#include "stm32f30x.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <rtthread.h>
#include "mymisc.h"
#include "ili9325.h"
#include "tslib.h"

extern void ads7843_init();
extern uint8_t cal_finished;
struct tsdev *ts;
extern calibration cal;
int main(void)
{	
	led_init();
	rt_kprintf("float system on\r\n");
	stm32_lcd_init();
	fb_clr(BLACK);
	ads7843_init();
#if 0
	rt_thread_mdelay(200);
	ts_calibrate();
#else
	cal_finished = 1;
	cal.a[0] = -7;
	cal.a[1] = 549;
	cal.a[2] = -973653;
	cal.a[3] = 738;
	cal.a[4] = 16;
	cal.a[5] = -1314850;
	cal.a[6] = 65536;
	ts = ts_open_module();
#endif
	rt_kprintf("go to here\r\n");
	while(1) {
		led(0);
		rt_thread_mdelay(1000);
		led(1);
		rt_thread_mdelay(1000);
	}
	return 0;
} 
