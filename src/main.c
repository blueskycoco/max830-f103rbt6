#include "stm32f30x.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "mymisc.h"

int main(void)
{	
	delay_init(72);
	led_init();
//	Uart_Init();
//	nprintf("float system on\r\n");
	while(1) {
		led(0);
		delay_ms(1000);
//		nprintf("led off\r\n");
		led(1);
		delay_ms(1000);
//		nprintf("led on\r\n");
	}
	return 0;
} 
