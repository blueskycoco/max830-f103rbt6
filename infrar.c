#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stm32f10x.h>
#include <string.h>
#include "mymisc.h"
#include "can.h"
extern void SWO_Enable(void);
#define DEVICE_MODE		0xD1
#define ID_CODE_LEN		4
#define STM32_CODE_LEN	6
#define ID_CODE			0x00000001
#define CMD_REG_CODE		0x0000
#define CMD_REG_CODE_ACK	0x0001
#define CMD_CONFIRM_CODE	0x0014
#define CMD_CONFIRM_CODE_ACK	0x0015
#define CMD_ALARM		0x0006
#define CMD_ALARM_ACK	0x0007
#define CMD_LOW_POWER	0x000c
#define CMD_LOW_POWER_ACK	0x000d
#define CMD_CUR_STATUS	0x0010
#define CMD_CUR_STATUS_ACK	0x0011

#define DEVICE_TYPE		0x02
#define MSG_HEAD0		0x6c
#define MSG_HEAD1		0xaa

#define KEY_CODE	0x01
#define KEY_INFRAR	0x02
#define KEY_S1		0x04
#define KEY_TIMER 	0x08
#define KEY_CAN	0x10
//#define KEY_LOWPOWER	0x20

#define STATE_ASK_CC1101_ADDR		0
#define STATE_CONFIRM_CC1101_ADDR	1
#define STATE_PROTECT_ON			2
#define STATE_PROTECT_OFF			3
unsigned char g_state = STATE_ASK_CC1101_ADDR;
#define MIN_BAT		0x96
unsigned char b_protection_state = 0;	/*protection state*/
unsigned char last_sub_cmd = 0x00; /*0x01 s1_alarm, 0x02 infrar_alarm, 0x04 low_power_alarm, 0x08 cur_status*/
volatile unsigned char key = 0x0;
unsigned char stm32_id[STM32_CODE_LEN] = {0};
unsigned char zero_id[STM32_CODE_LEN] = {0};
unsigned char can_addr = 3;
uint32_t addr_buf[1024] = {0};
#define STM32_ADDR	0x01
unsigned char can_rcv = 0;
unsigned char can_broadcast = 0;
unsigned char protect_status = 0;
void int_init()
{
	EXTI_InitTypeDef   EXTI_InitStructure;
	GPIO_InitTypeDef   GPIO_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource12);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource13);

	EXTI_InitStructure.EXTI_Line = EXTI_Line12|EXTI_Line13;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);
	
	/* Configure EXTI Line17(RTC Alarm) to generate an interrupt on rising edge */
	EXTI_ClearITPendingBit(EXTI_Line17);
	EXTI_InitStructure.EXTI_Line = EXTI_Line17;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = RTCAlarm_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);
	/* RTC clock source configuration ------------------------------------------*/
	/* Allow access to BKP Domain */
	PWR_BackupAccessCmd(ENABLE);

	/* Reset Backup Domain */
	BKP_DeInit();

	/* Enable the LSE OSC */
	RCC_LSEConfig(RCC_LSE_ON);
	/* Wait till LSE is ready */
	while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
	{
	}

	/* Select the RTC Clock Source */
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

	/* Enable the RTC Clock */
	RCC_RTCCLKCmd(ENABLE);

	/* RTC configuration -------------------------------------------------------*/
	/* Wait for RTC APB registers synchronisation */
	RTC_WaitForSynchro();

	/* Set the RTC time base to 1s */
	RTC_SetPrescaler(32767);	
	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();

	/* Enable the RTC Alarm interrupt */
	RTC_ITConfig(RTC_IT_ALR, ENABLE);
	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();

}
void SYSCLKConfig_STOP(void)
{
	ErrorStatus HSEStartUpStatus;
  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if(HSEStartUpStatus == SUCCESS)
  {

    /* Enable PLL */ 
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  }
}

void EXTI15_10_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line13) != RESET)
	{
		/* infrar */
		key |= KEY_INFRAR;
		EXTI_ClearITPendingBit(EXTI_Line13);
		//ctl_int(EXTI_Line13,0);
	}
	
	if(EXTI_GetITStatus(EXTI_Line12) != RESET)
	{
		/* S1 key*/
		key |= KEY_S1;
		EXTI_ClearITPendingBit(EXTI_Line12);
		can_broadcast = 1;
		protect_status = !protect_status;
		//ctl_int(EXTI_Line12,0);
	}

	if(EXTI_GetITStatus(EXTI_Line10) != RESET)
	{
		/* S1 key*/
		key |= KEY_CAN;
		EXTI_ClearITPendingBit(EXTI_Line10);
		//ctl_int(EXTI_Line10,0);
	}
	
    if(PWR_GetFlagStatus(PWR_FLAG_WU) != RESET)
    {
      /* Clear Wake Up flag */
      PWR_ClearFlag(PWR_FLAG_WU);
    }
}
void RTCAlarm_IRQHandler(void)
{
  if(RTC_GetITStatus(RTC_IT_ALR) != RESET)
  {

    /* Clear EXTI line17 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line17);

    /* Check if the Wake-Up flag is set */
    if(PWR_GetFlagStatus(PWR_FLAG_WU) != RESET)
    {
      /* Clear Wake Up flag */
      PWR_ClearFlag(PWR_FLAG_WU);
    }

    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();   
    /* Clear RTC Alarm interrupt pending bit */
    RTC_ClearITPendingBit(RTC_IT_ALR);
    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();
	key |= KEY_TIMER;
  }
}

uint16_t get_addr_offs(uint32_t id)
{
	int i=3;
	for (i=3;i<can_addr;i++)
		if (id == addr_buf[i])
			break;
		
	if (i==can_addr)
	{
		addr_buf[i] = id;
		can_addr++;
		return can_addr;
	}

	return i;
}
void CAN1_RX0_IRQHandler(void)
{
unsigned char resp[8] = {0};
unsigned char cmd[8] = {0};
unsigned short stdid;
unsigned char len;
uint16_t addr;
uint32_t id;
#ifdef MASTER
  if ((stdid = can_read(resp, &len)) != 0) {
				  id = resp[4];
				  id = (id<<8) + resp[5];
				  id = (id<<8) + resp[6];
				  id = (id<<8) + resp[7];
				  addr = get_addr_offs(id);
				  if (resp[0] == 0x00 && resp[1] == 0x00)
				  {   //assign can addr
					  cmd[0] = 0x00;cmd[1]=0x01;
					  cmd[2] = (addr >> 8) & 0xff;
					  cmd[3] = addr&0xff;
					  memcpy(cmd+4, resp+4, 4);
					  can_send(2,cmd,8);
				  } else if (resp[0] == 0x00 && resp[1] == 0x06) {
					  //alarm
					  cmd[0] = 0x00;cmd[1]=0x07;
					  cmd[2] = resp[2];
					  cmd[3] = protect_status;
					  memcpy(cmd+4, resp+4, 4);
					  can_send(addr,cmd,8);				  
				  }
				  led(1);
				  delay_ms(1000);
				  led(0); 
			  } 	  

#else
handle_can_resp();

#endif
}

/*
	103c8t6 -> stm32
	cmd_type 2 sub_cmd_type/device_mode 1 device_type 1 103c8t6_id 4
*/
void handle_can_addr(uint8_t *id, uint8_t res) 
{	
	unsigned char cmd[32] = {0x00};
	unsigned char ofs = 0;
	cmd[ofs++] = (CMD_REG_CODE >> 8) & 0xff;
	cmd[ofs++] = CMD_REG_CODE & 0xff;
	cmd[ofs++] = DEVICE_TYPE;
	cmd[ofs++] = DEVICE_MODE;
	cmd[ofs++] = ((long)ID_CODE >> 24) & 0xff;
	cmd[ofs++] = ((long)ID_CODE >> 16) & 0xff;
	cmd[ofs++] = ((long)ID_CODE >> 8) & 0xff;
	cmd[ofs++] = ((long)ID_CODE >> 0) & 0xff;
	can_send(0x01, cmd, ofs);
}
void handle_can_cmd(uint16_t main_cmd, uint8_t sub_cmd) 
{	
	unsigned char cmd[32] = {0x00};
	unsigned char ofs = 0;
	cmd[ofs++] = (main_cmd >> 8) & 0xff;
	cmd[ofs++] = main_cmd & 0xff;
	cmd[ofs++] = sub_cmd;
	cmd[ofs++] = DEVICE_TYPE;
	cmd[ofs++] = ((long)ID_CODE >> 24) & 0xff;
	cmd[ofs++] = ((long)ID_CODE >> 16) & 0xff;
	cmd[ofs++] = ((long)ID_CODE >> 8) & 0xff;
	cmd[ofs++] = ((long)ID_CODE >> 0) & 0xff;

	if (main_cmd == CMD_ALARM) {
		if (sub_cmd == 0x01)
			last_sub_cmd |= 0x02;
		else if(sub_cmd == 0x02)
			last_sub_cmd |= 0x01;
	} else if (main_cmd == CMD_LOW_POWER) {
		last_sub_cmd |= 0x04;
	}
	
	can_send(0x01, cmd, ofs);
}

void switch_protect(unsigned char state)
{
	b_protection_state = state;
	if (b_protection_state) {
		/*switch to protect on*/
		//timer off
		//infrar int on
		ctl_int(EXTI_Line13, 1);
		GPIO_SetBits(GPIOA,GPIO_Pin_2);
		led(1);
	} else {
		/*switch to protect off*/
		//timer on
		//infrar int off
		//TACTL = TASSEL_1 + MC_2 + TAIE + ID0;
		ctl_int(EXTI_Line13,0);
		GPIO_ResetBits(GPIOA,GPIO_Pin_2);
		led(0);
	}				
}
/*	
	stm32 -> 103c8t6
	cmd_type 2 sub_cmd_type 1 result/protect_status/addr 1 103c8t6_id 4
*/
void handle_can_resp()
{
	unsigned char resp[32] = {0};
	unsigned char len = 32;
	unsigned short cmd_type = 0;
	int result = can_read(resp, &len);
	if (result !=0 && len > 0) {
		#if 0
		if (resp[2] != MSG_HEAD0 || resp[3] != MSG_HEAD1)
			return ;		
		if (resp[4] != len -5)
			return ;
		/*check subdevice id = local device id*/
		if (ID_CODE !=((resp[11]<<24)|(resp[12]<<16)|(resp[13]<<8)|(resp[14]<<0)))
			return ;
		/*check stm32 id = saved stm32 id*/
		if (memcmp(stm32_id , zero_id, STM32_CODE_LEN) !=0) {
			if (memcmp(stm32_id, resp+5, STM32_CODE_LEN) !=0 && g_state !=STATE_ASK_CC1101_ADDR)
				return ;
		}
		len = resp[4];
		unsigned short crc = Packet_CRC(resp, len+3);
		/*check crc*/
		if (crc != (resp[len+3] << 8 | resp[len+4]))
			return ;
		
	cmd_type = resp[15]<<8 | resp[16];
	#endif
	uint32_t id = resp[4];
	id = (id << 8) + resp[5];
	id = (id << 8) + resp[6];
	id = (id << 8) + resp[7];
	if (ID_CODE !=id)
			return ;
	cmd_type = resp[0]<<8 | resp[1];
	switch (cmd_type) {
		case CMD_REG_CODE_ACK:
			set_id(resp[2]<<8 | resp[3]);		
			g_state = STATE_PROTECT_ON;			
			break;
		
		case CMD_ALARM_ACK:
			if (b_protection_state != resp[3]) {
				switch_protect(resp[3]);
			}
			switch (resp[2]) {
				case 0x01:
					last_sub_cmd &= ~0x02;
					break;
				case 0x02:
					last_sub_cmd &= ~0x01;
					break;
				default:
					break;		
			}
			break;
		case CMD_LOW_POWER_ACK:
		case CMD_CUR_STATUS_ACK:
			if (b_protection_state != resp[3]) {
				switch_protect(resp[3]);
			}
			if (cmd_type == CMD_LOW_POWER_ACK)
				last_sub_cmd &= ~0x04;
			if (cmd_type == CMD_CUR_STATUS_ACK)
				last_sub_cmd &= ~0x08;
			break;
		default:
			break;
	}

	}
}

void reconfig_rtc()
{
	 RTC_ClearFlag(RTC_FLAG_SEC);
	 while(RTC_GetFlagStatus(RTC_FLAG_SEC) == RESET);
	 RTC_SetAlarm(RTC_GetCounter()+ 5);
	 RTC_WaitForLastTask();
}
void handle_timer()
{
	if (g_state == STATE_ASK_CC1101_ADDR)
		handle_can_addr(NULL, 0);
	else {
		if (last_sub_cmd & 0x01)
			handle_can_cmd(CMD_ALARM,0x02);
		if (b_protection_state) {
			if (last_sub_cmd & 0x02)
				handle_can_cmd(CMD_ALARM,0x01);
		} else {
			last_sub_cmd &= ~0x02;
		}

		if (last_sub_cmd & 0x04)
			handle_can_cmd(CMD_LOW_POWER,0x00);
	}

	if (last_sub_cmd !=0 || g_state != STATE_PROTECT_ON)
	reconfig_rtc();
 
	//unsigned short bat = read_adc();
	//if (bat < MIN_BAT)
	//	handle_can_cmd(CMD_LOW_POWER,0x00);
}
void task()
{		
	led(1);
	delay_ms(1000);
	led(0);	
	delay_ms(1000);
	led(1);
	delay_ms(1000);
	led(0);
	printf("begin to ask addr\r\n");
	//while(1) delay_ms(1000);
	handle_can_addr(NULL, 0);
	reconfig_rtc();
	while (1) {
		led(0);
		//PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
		//SYSCLKConfig_STOP();
		__disable_irq();
		led(1);
		//printf("wake from stop\r\n");
		if (key & KEY_TIMER) {
			key &= ~KEY_TIMER;
			printf("handle timer\r\n");
			handle_timer();
		}
		#if 1
		if (key & KEY_S1) {
			key &= ~KEY_S1;
			/*send s1 alarm to stm32*/
			handle_can_cmd(CMD_ALARM, 0x02);
			//ctl_int(EXTI_Line12,1);
		}

		if (key & KEY_INFRAR) {
			key &= ~KEY_INFRAR;
			/*send infrar alarm to stm32*/
			//add int count then make decision
			if (b_protection_state)
			handle_can_cmd(CMD_ALARM, 0x01);
			//ctl_int(EXTI_Line13,1);
		}

		if (key & KEY_CAN) {
			key &= ~KEY_CAN;
			/*new data come from stm32*/
			handle_can_resp();
			ctl_int(EXTI_Line10,1);
		}
		#endif
		//printf("goto stop\r\n");
		__enable_irq();
		delay_ms(1000);
	}
	return ;
}
/*
  * slave -> master
  * 00 00 02 d1 00 00 00 01 (stdid = 0x01 , req can addr)
  * 00 01 xx xx 00 00 00 01 (stdid = 0x02 , ack can addr)
  */
void task_master()
{
	static unsigned short can_addr=0;
	unsigned char resp[8] = {0};
	unsigned char cmd[8] = {0};
	unsigned char len = 32;
	unsigned short stdid = 0;

	while (1) {
		memset(cmd,0,8);
		if (can_rcv) {
			can_rcv = 0;
			if ((stdid = can_read(resp, &len)) != 0) {
				if (resp[0] == 0x00 && resp[1] == 0x00)
				{	//assign can addr
					cmd[0] = 0x00;cmd[1]=0x01;
					cmd[2] = (can_addr >> 8) & 0xff;
					cmd[3] = can_addr&0xff;
					memcpy(cmd+4, resp+4, 4);
					can_send(2,cmd,8);
					delay_ms(2000);
					can_send(can_addr,cmd,8);
					can_addr++;
				} else if (resp[0] == 0x00 && resp[1] == 0x06) {
					//alarm
					cmd[0] = 0x00;cmd[1]=0x07;
					cmd[2] = resp[2];
					cmd[3] = protect_status;
					memcpy(cmd+4, resp+4, 4);
					can_send(stdid,cmd,8);					
				}
				led(1);
				delay_ms(1000);
				led(0);	
			}			
		}

		if (can_broadcast) {
			can_broadcast = 0;
			cmd[0] = 0x00;cmd[1]=0x11;
			cmd[2] = 0x00;
			cmd[3] = protect_status;
			can_send(2,cmd,8);					
		}
	}
}
int main(void)
{	
	led_init();
	delay_init(72);
	SWO_Enable();
	Debug_uart_Init();
	int_init();
	can_init();
	#ifdef MASTER
	while(1) delay_ms(1000);
	#else
	task();
	#endif
}
