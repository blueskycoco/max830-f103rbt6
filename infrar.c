#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stm32f0xx.h>
#include <string.h>
#include <stdlib.h>
#include "mymisc.h"

int main(void)
{	
	led_init();
	delay_init(48);
	Debug_uart_Init();
	while(1) {
		unsigned char *ptr = (unsigned char *)malloc(1024);
		if (ptr == NULL)
			printf("malloc 1024 failed\r\n");
		else
		{
			printf("malloc 1024 ok\r\n");
			free(ptr);
		}
		delay_ms(500);
		led(1);
		delay_ms(500);
		led(0);
		
	}

}
