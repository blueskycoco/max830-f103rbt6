#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stm32f10x.h>
#include <string.h>
#include "mymisc.h"
#include "can.h"

#define ADDR_SN						4
#define ADDR_DATE					1
extern void SWO_Enable(void);
#define DEVICE_MODE					0xD211
#define CMD_REG_CODE				0x0000
#define CMD_REG_CODE_ACK			0x0001
#define CMD_INFO_CODE				0x0002
#define CMD_INFO_CODE_ACK			0x0003
#define CMD_ALARM					0x0006
#define CMD_ALARM_ACK				0x0007
#define CMD_CUR_STATUS				0x0010
#define CMD_CUR_STATUS_ACK			0x0011
#define DEVICE_TYPE					0x22
#define KEY_INFRAR					0x02
#define KEY_S1						0x04
#define KEY_TIMER 					0x08
#define KEY_CAN						0x10
#define STATE_ASK_CC1101_ADDR		0
#define STATE_PROTECT_ON			2
#define STATE_UPDATE_INFO			1
int timer_5s = 0;
unsigned char g_cnt 				= 0;
unsigned char g_state 				= STATE_ASK_CC1101_ADDR;
unsigned char b_protection_state 	= 0;	/*protection state*/
/*0x01 s1_alarm, 0x02 infrar_alarm, 0x04 low_power_alarm, 0x08 cur_status*/
unsigned char last_sub_cmd 			= 0x00; 
volatile unsigned char key 			= 0x0;
unsigned char g_id 					= 0x01;
int g_heart_cnt	= 0;

void read_info(char addr, unsigned char *buf, char len)
{
		memcpy(buf, (const void *)(0x0800fff0+addr), len);
}
void int_init()
{
	EXTI_InitTypeDef   EXTI_InitStructure;
	GPIO_InitTypeDef   GPIO_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;//|GPIO_Pin_13|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	//GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource12);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource13);
	//GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource7);
	EXTI_InitStructure.EXTI_Line = EXTI_Line13;//|EXTI_Line12|EXTI_Line7;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);
#if 0//ndef MASTER
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_Init(&NVIC_InitStructure);
#endif
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
	/* RTC clock source configuration ---------------------------------------*/
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

	/* RTC configuration ----------------------------------------------------*/
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
void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line7) != RESET)
	{
		key |= KEY_CAN;
		EXTI_ClearITPendingBit(EXTI_Line7);
	}
}
void EXTI15_10_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line13) != RESET)
	{
		if (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13) == RESET) {
		key |= KEY_INFRAR;
		printf("infrar int\r\n");
		//	if (b_protection_state)
		//		handle_can_cmd(CMD_ALARM, 0x01);
		}
		EXTI_ClearITPendingBit(EXTI_Line13);
	}

	//if(EXTI_GetITStatus(EXTI_Line12) != RESET)
	//{
	//	key |= KEY_S1;
	//	EXTI_ClearITPendingBit(EXTI_Line12);
	//}

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
			//SYSCLKConfig_STOP();
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

/*
   103c8t6 -> stm32
   cmd_type 2 (sub_cmd_type device_type)/device_mode 2 103c8t6_id 4
   */
void handle_can_addr(uint8_t *id, uint8_t res) 
{	
	unsigned char cmd[32] = {0x00};
	unsigned char ofs = 0;
	cmd[ofs++] = (CMD_REG_CODE >> 8) & 0xff;
	cmd[ofs++] = CMD_REG_CODE & 0xff;
	cmd[ofs++] = (DEVICE_MODE >> 8) & 0xff;
	cmd[ofs++] = DEVICE_MODE & 0xff;
	read_info(ADDR_SN, cmd+ofs, 4);
	ofs += 4;
	//cmd[ofs++] = ((long)ID_CODE >> 24) & 0xff;
	//cmd[ofs++] = ((long)ID_CODE >> 16) & 0xff;
	//cmd[ofs++] = ((long)ID_CODE >> 8) & 0xff;
	//cmd[ofs++] = ((long)ID_CODE >> 0) & 0xff;
	if (!can_send(0x01, cmd, ofs)) {
		can_init();
		g_state	= STATE_ASK_CC1101_ADDR;
		last_sub_cmd = 0;	
		b_protection_state = 0;
		switch_protect(0);
		set_id(0x02);	
	}
	//if (poll_can())
	//		handle_can_resp();

}
void handle_can_info(uint8_t *id, uint8_t res) 
{	
	unsigned char cmd[32] = {0x00};
	unsigned char ofs = 0;
	cmd[ofs++] = (CMD_INFO_CODE >> 8) & 0xff;
	cmd[ofs++] = CMD_INFO_CODE & 0xff;
	cmd[ofs++] = g_id;
	cmd[ofs++] = DEVICE_TYPE & 0xff;
	ofs++;
	read_info(ADDR_DATE, cmd+ofs, 3);
	ofs += 3;
	//cmd[ofs++] = ((long)FACT_TIME >> 24) & 0xff;
	//cmd[ofs++] = ((long)FACT_TIME >> 16) & 0xff;
	//cmd[ofs++] = ((long)FACT_TIME >> 8) & 0xff;
	//cmd[ofs++] = ((long)FACT_TIME >> 0) & 0xff;
	if (!can_send(0x01, cmd, ofs)) {
		can_init();
		g_state	= STATE_ASK_CC1101_ADDR;
		last_sub_cmd = 0;	
		b_protection_state = 0;
		switch_protect(0);
		set_id(0x02);	
	}
	//if (poll_can())
	//		handle_can_resp();

}
void handle_can_cmd(uint16_t main_cmd, uint8_t sub_cmd) 
{	
	unsigned char cmd[32] = {0x00};
	unsigned char ofs = 0;
	cmd[ofs++] = (main_cmd >> 8) & 0xff;
	cmd[ofs++] = main_cmd & 0xff;
	cmd[ofs++] = g_id;
	cmd[ofs++] = sub_cmd;
	read_info(ADDR_SN, cmd+ofs, 4);
	ofs += 4;
	//cmd[ofs++] = ((long)ID_CODE >> 24) & 0xff;
	//cmd[ofs++] = ((long)ID_CODE >> 16) & 0xff;
	//cmd[ofs++] = ((long)ID_CODE >> 8) & 0xff;
	//cmd[ofs++] = ((long)ID_CODE >> 0) & 0xff;

	if (main_cmd == CMD_ALARM) {
		if (sub_cmd == 0x01)
			last_sub_cmd |= 0x02;
		else if(sub_cmd == 0x02)
			last_sub_cmd |= 0x01;
	} else if (main_cmd == CMD_CUR_STATUS) {
		last_sub_cmd |= 0x08;
	}
	g_cnt++;
	if (!can_send(0x01, cmd, ofs)) {
		can_init();
		g_state	= STATE_ASK_CC1101_ADDR;
		last_sub_cmd = 0;	
		b_protection_state = 0;
		switch_protect(0);
		set_id(0x02);	
	}
	//if (poll_can())
	//		handle_can_resp();
}

void switch_protect(unsigned char state)
{
	b_protection_state = state;
	if (b_protection_state) {
		GPIO_SetBits(GPIOA,GPIO_Pin_2);
		led(1);
	} else {
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
		uint32_t id = resp[4];
		uint8_t ids[4];
		id = (id << 8) + resp[5];
		id = (id << 8) + resp[6];
		id = (id << 8) + resp[7];
		read_info(ADDR_SN, ids, 4);
		printf("ids %x %x %x %x\r\n", ids[0], ids[1], ids[2], ids[3]);	
		if (memcmp(ids, resp+4, 4) !=0) {
			printf("id %08x is not correct %02x%02x%02x%02x\r\n", 
					id,ids[0],ids[1],ids[2],ids[3]);
			if (g_state == STATE_UPDATE_INFO)
			{
				g_state = STATE_ASK_CC1101_ADDR; 
				set_id(0x02);	
			}
			return ;
		}
		printf("<$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\r\n");
		printf("id\t\t%08x\r\n", id);
		cmd_type = resp[0]<<8 | resp[1];
		switch (cmd_type) {
			case CMD_REG_CODE_ACK:
				set_id(resp[2]<<8 | resp[3]);	
				printf("resp\t\tassign addr\r\n");
				printf("new addr\t%x\r\n",resp[2]<<8|resp[3]);
				g_id = (resp[2]<<8) | resp[3];
				g_state = STATE_UPDATE_INFO;		
				g_cnt = 0;				
				break;
			case CMD_INFO_CODE_ACK:
				g_state = STATE_PROTECT_ON;		
				if (b_protection_state != resp[3]) {
					switch_protect(resp[3]);
				}
				g_cnt = 0;				
				printf("resp\t\tregister done\r\n");
				printf("status\t\t%s\r\n", (resp[3]==0)?"che fang":"bu fang");
				printf("can addr\t%x\r\n", g_id);
				break;
			case CMD_ALARM_ACK:
				g_cnt = 0;
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
				printf("resp\t\talarm ack\r\n");
				printf("status\t\t%s\r\n", (resp[3]==0)?"che fang":"bu fang");
				printf("alarm\t\t%d\r\n", resp[2]);
				printf("can addr\t%x\r\n", g_id);
				break;
			case CMD_CUR_STATUS_ACK:
				g_cnt = 0;
				if (b_protection_state != resp[3]) {
					switch_protect(resp[3]);
				}
				if (cmd_type == CMD_CUR_STATUS_ACK)
					last_sub_cmd &= ~0x08;
				printf("resp\t\tack status\r\n");
				printf("status\t\t%s\r\n", (resp[3]==0)?"che fang":"bu fang");
				printf("can addr\t%x\r\n", g_id);
				break;
			default:
				break;
		}
		printf("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$>\r\n");

	}
}

void reconfig_rtc(int sec)
{
	RTC_ClearFlag(RTC_FLAG_SEC);
	while(RTC_GetFlagStatus(RTC_FLAG_SEC) == RESET);
	RTC_SetAlarm(RTC_GetCounter()+ sec);
	RTC_WaitForLastTask();
}
void handle_timer()
{
	printf("handle timer in %d, state %d, key %x\r\n",g_cnt,g_state,key);
	if (g_cnt > 3) {
		g_state	= STATE_ASK_CC1101_ADDR;
		last_sub_cmd = 0;	
		b_protection_state = 0;
		switch_protect(0);
		set_id(0x02);	
	}
	if (g_state == STATE_ASK_CC1101_ADDR)
		handle_can_addr(NULL, 0);
	else if (g_state == STATE_UPDATE_INFO) {
		handle_can_info(NULL, 0);
	}
	else {
		if (last_sub_cmd & 0x01)
			handle_can_cmd(CMD_ALARM,0x02);
		if (b_protection_state) {
			if (last_sub_cmd & 0x02)
				handle_can_cmd(CMD_ALARM,0x01);
		} else {
			last_sub_cmd &= ~0x02;
			handle_can_cmd(CMD_CUR_STATUS,0x01);
		}

	}

	g_heart_cnt++;
	if (g_heart_cnt >= 24) {
		handle_can_cmd(CMD_CUR_STATUS,0x01);
		g_heart_cnt = 0;
	}
}
void CAN1_RX0_IRQHandler(void)
{
	//printf("can1 rx0 %d\r\n", CAN_GetITStatus(CAN1,CAN_IT_FMP0));
	if (CAN_GetITStatus(CAN1,CAN_IT_FMP0) != RESET) { 
		handle_can_resp();
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
	}
}
/*
 * slave -> master
 * 00 00 02 d1 00 00 00 01 (stdid = 0x01 , req can addr)
 * 00 01 xx xx 00 00 00 01 (stdid = 0x02 , ack can addr)
 */
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
	reconfig_rtc(1);
	while (1) {
		//key = 0;
		//PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
		//SYSCLKConfig_STOP();		
		//delay_ms(10);
		if (key != 0) {
		printf("leave stop %02x\r\n",key);
		if (key & KEY_TIMER) {
			key &= ~KEY_TIMER;
			printf("handle timer\r\n");
			handle_timer();
		}
//		if (key & KEY_S1) {
//			key &= ~KEY_S1;
//			printf("S1 pressed\r\n");
//			handle_can_cmd(CMD_ALARM, 0x02);
//		}

		if (key & KEY_INFRAR) {
			key &= ~KEY_INFRAR;
			printf("infrar pressed\r\n");
			if (b_protection_state)
			{
				led(1);
				handle_can_cmd(CMD_ALARM, 0x01);
				delay_ms(1000);
			}
		}

//		if (key & KEY_CAN) {
//			key &= ~KEY_CAN;
//			printf("can data in\r\n");
//			handle_can_resp();
//		}
		printf("enter stop %02x\r\n",key);
		//delay_ms(30);
		led(0);

//		if (!b_protection_state || last_sub_cmd !=0 || 
//				g_state != STATE_PROTECT_ON)
		{
			printf("%d %d %d %02x\r\n",b_protection_state,last_sub_cmd,g_state,key);
			reconfig_rtc(5);
		}
		} else 
			delay_ms(100);
	}
	return ;
}

int main(void)
{	
	led_init();
	delay_init(72);
	SWO_Enable();
	Debug_uart_Init();
	int_init();
	can_init();
	task();
}

