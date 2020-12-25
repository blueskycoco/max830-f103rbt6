#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stm32f0xx.h>
#include <string.h>
#include <stdlib.h>
#include "mymisc.h"
#include "usbd_custom_hid_core.h"
#include  "usbd_usr.h"
USB_CORE_HANDLE  USB_Device_dev ;
int main(void)
{	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	SYSCFG->CFGR1 |= SYSCFG_CFGR1_PA11_PA12_RMP;

	USBD_Init(&USB_Device_dev,
			&USR_desc,
			&USBD_HID_cb,
			&USR_cb);
	led_init();
	delay_init(48);
	Uart_Init();
	nprintf("float system on\r\n");
	while(1) {
		led(0);
		delay_ms(1000);
		nprintf("led off\r\n");
		led(1);
		delay_ms(1000);
		nprintf("led on\r\n");
	}
}
