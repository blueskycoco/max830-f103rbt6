#include <stdio.h>
#include <stm32f10x.h>
#include <string.h>
#include "can.h"

CanRxMsg 							RxMessage;
CAN_FilterInitTypeDef  				CAN_FilterInitStructure;
CanTxMsg 							TxMessage;
uint16_t 							local_addr = 0x02;
#define RCC_APB2Periph_GPIO_CAN1   	RCC_APB2Periph_GPIOB
#define CANx			   			CAN1
#define GPIO_CAN		   			GPIOB
#define GPIO_Remapping_CAN	   		GPIO_Remap1_CAN1
#define GPIO_Pin_CAN_RX 	   		GPIO_Pin_8
#define GPIO_Pin_CAN_TX 	   		GPIO_Pin_9

int can_send(unsigned short id, unsigned char *payload, 
		unsigned char payload_len)
{
	int i;
	uint8_t status,TransmitMailbox;
	TxMessage.StdId = id;
	memcpy(TxMessage.Data, payload, 8);
#ifdef DEBUG
	printf("CAN send:\r\n");
	for (i=0;i<payload_len;i++)
		printf("%02x ", payload[i]);
	printf("\r\n");
#endif
	TransmitMailbox = CAN_Transmit(CANx, &TxMessage);
	if (TransmitMailbox == CAN_TxStatus_NoMailBox)
		return 0;

	i = 0;
	while(((status = CAN_TransmitStatus(CANx, TransmitMailbox)) != CANTXOK) 
			&& (i != 0xFFFF))
	{
		i++;
	}
	printf("i is %x\r\n", i);
	if (i == 0xFFFF)
		return 0;
	
	return 1;
}
int can_read(unsigned char *buf, unsigned char *buf_len)
{
	*buf_len = 0;
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
#ifdef DEBUG
	printf("DLC %x, ExtId %x, FMI %x, IDE %x, RTR %x, StdId %x\r\n",
			RxMessage.DLC,(unsigned int)RxMessage.ExtId,RxMessage.FMI,
			RxMessage.IDE,RxMessage.RTR,(unsigned int)RxMessage.StdId);
#endif
	if ((RxMessage.StdId == local_addr)&&(RxMessage.IDE == CAN_ID_STD) 
			&& (RxMessage.DLC == 8))
	{
		memcpy(buf, RxMessage.Data, 8);
		*buf_len = 8;
#ifdef DEBUG
		printf("we got message %02x %02x %02x %02x %02x %02x %02x %02x\r\n", 
				RxMessage.Data[0],RxMessage.Data[1],RxMessage.Data[2],
				RxMessage.Data[3],RxMessage.Data[4],RxMessage.Data[5],
				RxMessage.Data[6],RxMessage.Data[7]);
#endif
		return RxMessage.StdId;
	}
	return 0;
}
void set_id(unsigned short id)
{
	local_addr = id & 0x3ff;
	CAN_FilterInitStructure.CAN_FilterIdHigh = local_addr << 5;
	CAN_FilterInit(&CAN_FilterInitStructure);
}
void can_init()
{
	uint8_t i = 0;
	NVIC_InitTypeDef  	NVIC_InitStructure;
	GPIO_InitTypeDef	GPIO_InitStructure;
	CAN_InitTypeDef		CAN_InitStructure;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);


	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_CAN1, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_CAN_RX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIO_CAN, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_CAN_TX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIO_CAN, &GPIO_InitStructure);

	GPIO_PinRemapConfig(GPIO_Remapping_CAN , ENABLE);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);


	CAN_DeInit(CANx);
	CAN_StructInit(&CAN_InitStructure);

	CAN_InitStructure.CAN_TTCM = DISABLE;
	CAN_InitStructure.CAN_ABOM = DISABLE;
	CAN_InitStructure.CAN_AWUM = DISABLE;
	CAN_InitStructure.CAN_NART = DISABLE;
	CAN_InitStructure.CAN_RFLM = DISABLE;
	CAN_InitStructure.CAN_TXFP = DISABLE;
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;

	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_5tq;
	CAN_InitStructure.CAN_Prescaler = 4;
	CAN_Init(CANx, &CAN_InitStructure);

	CAN_FilterInitStructure.CAN_FilterNumber = 0;
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
#ifdef MASTER
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0020;
#else
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0040;
#endif
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xffff;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0xffff;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);

	TxMessage.StdId = 0x001;
	TxMessage.ExtId = 0x00;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.DLC = 8;


	RxMessage.StdId = 0x00;
	RxMessage.ExtId = 0x00;
	RxMessage.IDE = CAN_ID_STD;
	RxMessage.DLC = 0;
	RxMessage.FMI = 0;
	for (i = 0;i < 8;i++)
	{
		RxMessage.Data[i] = 0x00;
	}
	uint8_t num = CAN_MessagePending(CANx,CAN_FIFO0); 
	if (num > 0)
	while (num--) {
		CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
	}
//	CAN_ITConfig(CANx, CAN_IT_FMP0, ENABLE);
}
