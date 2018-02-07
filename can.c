#include <stdio.h>
#include <stm32f10x.h>
#include <string.h>
#include "can.h"

void NVIC_Config(void);
void CAN_Config(void);
void LED_Display(uint8_t Ledstatus);
void Init_RxMes(CanRxMsg *RxMessage);

CanRxMsg RxMessage;
CAN_InitTypeDef        CAN_InitStructure;
CAN_FilterInitTypeDef  CAN_FilterInitStructure;
CanTxMsg TxMessage;

#define RCC_APB2Periph_GPIO_CAN1    RCC_APB2Periph_GPIOB
#define GPIO_Remapping_CAN1         GPIO_Remap1_CAN1
#define GPIO_CAN1                   GPIOB
#define GPIO_Pin_CAN1_RX            GPIO_Pin_8
#define GPIO_Pin_CAN1_TX            GPIO_Pin_9
#define CANx					   CAN1
#define GPIO_CAN				   GPIO_CAN1
#define GPIO_Remapping_CAN		   GPIO_Remapping_CAN1
#define GPIO_CAN				   GPIO_CAN1
#define GPIO_Pin_CAN_RX 		   GPIO_Pin_CAN1_RX
#define GPIO_Pin_CAN_TX 		   GPIO_Pin_CAN1_TX

void CAN1_RX0_IRQHandler(void)
{
  CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
  if ((RxMessage.StdId == 0x321)&&(RxMessage.IDE == CAN_ID_STD) && (RxMessage.DLC == 1))
  {
	//LED_Display(RxMessage.Data[0]);
	//KeyNumber = RxMessage.Data[0];
  }
}
int can_send(unsigned char *payload, unsigned char payload_len)
{
	memcpy(TxMessage.Data, payload, 8);
	CAN_Transmit(CANx, &TxMessage);
	return 1;
}
int can_read(unsigned char *buf, unsigned char *buf_len)
{
	*buf_len = 0;
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
	if ((RxMessage.StdId == 0x321)&&(RxMessage.IDE == CAN_ID_STD) && (RxMessage.DLC == 1))
  	{
		memcpy(buf, RxMessage.Data, 8);
		*buf_len = 8;
	}
	return 1;
}
void can_init()
{
	uint8_t i = 0;
	NVIC_InitTypeDef  NVIC_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);


	CAN_ITConfig(CANx, CAN_IT_FMP0, ENABLE);

	/* GPIO clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_CAN1, ENABLE);
	/* Configure CAN pin: RX */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_CAN_RX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIO_CAN, &GPIO_InitStructure);

	/* Configure CAN pin: TX */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_CAN_TX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIO_CAN, &GPIO_InitStructure);

	GPIO_PinRemapConfig(GPIO_Remapping_CAN , ENABLE);

	/* CANx Periph clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);


	/* CAN register init */
	CAN_DeInit(CANx);
	CAN_StructInit(&CAN_InitStructure);

	/* CAN cell init */
	CAN_InitStructure.CAN_TTCM = DISABLE;
	CAN_InitStructure.CAN_ABOM = DISABLE;
	CAN_InitStructure.CAN_AWUM = DISABLE;
	CAN_InitStructure.CAN_NART = DISABLE;
	CAN_InitStructure.CAN_RFLM = DISABLE;
	CAN_InitStructure.CAN_TXFP = DISABLE;
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;

	/* CAN Baudrate = 1MBps*/
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_5tq;
	CAN_InitStructure.CAN_Prescaler = 4;
	CAN_Init(CANx, &CAN_InitStructure);

	/* CAN filter init */
	CAN_FilterInitStructure.CAN_FilterNumber = 0;
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);

	/* Transmit */
	TxMessage.StdId = 0x321;
	TxMessage.ExtId = 0x01;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.DLC = 1;


	RxMessage.StdId = 0x00;
	RxMessage.ExtId = 0x00;
	RxMessage.IDE = CAN_ID_STD;
	RxMessage.DLC = 0;
	RxMessage.FMI = 0;
	for (i = 0;i < 8;i++)
	{
		RxMessage.Data[i] = 0x00;
	}
}
