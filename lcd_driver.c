#include "app_types.h"
#include "lcd_driver.h"
#include <stm32f10x.h>



/****************************************************************************
*��������TFT_GPIO_Config
*��  �룺��
*��  ������
*��  �ܣ���ʼ��TFT��IO�ڡ�
****************************************************************************/      
void TFT_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    printf("\r\n    TFT_GPIO_Config... ");

    // ��ʱ��ʹ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);

    // CS(PD7/FSMC-NE1/FSMC-NCE2)
    // RD/WR/CS/A16(RS)
    GPIO_InitStructure.GPIO_Pin   = ( GPIO_Pin_4 | GPIO_Pin_5 /*| GPIO_Pin_7 | GPIO_Pin_11*/);
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    // D2--D3 D13--D15 D0--D1 
    GPIO_InitStructure.GPIO_Pin   = (GPIO_Pin_0 | GPIO_Pin_1 
                                     | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 
                                     | GPIO_Pin_14 | GPIO_Pin_15);
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    // D4--D12
    GPIO_InitStructure.GPIO_Pin   = (GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9
                                     | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12
                                     | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

#if 1
    // A16(RS)
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // һֱ��Ч
    GPIO_WriteBit(GPIOD, GPIO_Pin_11, (BitAction)1);         // A16(RS)
#endif

#if 1
    // CS
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // һֱ��Ч
    GPIO_WriteBit(GPIOD, GPIO_Pin_7, (BitAction)0);         // CS
#endif
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // һֱ��Ч
    GPIO_WriteBit(GPIOD, GPIO_Pin_12, (BitAction)1);         // A16(RS)
}



/****************************************************************************
* Function Name  : TFT_FSMC_Config
* Description    : ��ʼ��FSMC
* Input          : None
* Output         : None
* Return         : None
****************************************************************************/
void TFT_FSMC_Config(void)
{
    FSMC_NORSRAMInitTypeDef        FSMC_NORSRAMInitStructure;
    FSMC_NORSRAMTimingInitTypeDef  FSMC_NORSRAMTiming;

    printf("\r\n    TFT_FSMC_Config... ");

    /* ��FSMC��ʱ�� */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);

    /* ���ö�дʱ�򣬸�FSMC_NORSRAMInitStructure���� */
    /* ��ַ����ʱ�䣬4��HCLK���� */
    FSMC_NORSRAMTiming.FSMC_AddressSetupTime = 0x03;
    /* ��ַ����ʱ�䣬1��HCLK���� */
    FSMC_NORSRAMTiming.FSMC_AddressHoldTime  = 0x02;
    /* ���ݽ���ʱ�䣬6��HCLK���� */
    FSMC_NORSRAMTiming.FSMC_DataSetupTime    = 0x05;
    /* ���߻ָ�ʱ������ */
    FSMC_NORSRAMTiming.FSMC_BusTurnAroundDuration = 0x02;
    /* ʱ�ӷ�Ƶ���� */
    FSMC_NORSRAMTiming.FSMC_CLKDivision = 0x00;                     // ��Ƶ��ʱ��      
    /* ���ݱ���ʱ�䣬1��HCLK���� */
    FSMC_NORSRAMTiming.FSMC_DataLatency = 0x02;
    /* ����ģʽ������ڵ�ַ/���ݲ�����ʱ��ABCDģʽ�����𲻴� */
    FSMC_NORSRAMTiming.FSMC_AccessMode  = FSMC_AccessMode_A;


    /*����FSMC_NORSRAMInitStructure������*/
    /* FSMC���ĸ��洢�飨bank��������ʹ�õ�һ����bank1�� */
    /* ͬʱ����ʹ�õ���bank����ĵ�1��RAM�� */
    FSMC_NORSRAMInitStructure.FSMC_Bank               = FSMC_Bank1_NORSRAM1;                // FSMC_Bank1_NORSRAM4
    FSMC_NORSRAMInitStructure.FSMC_DataAddressMux     = FSMC_DataAddressMux_Disable;  
    /* ��������ʹ��SRAMģʽ */
    FSMC_NORSRAMInitStructure.FSMC_MemoryType         = FSMC_MemoryType_SRAM;               // FSMC_MemoryType_NOR
    /* ʹ�õ����ݿ��Ϊ16λ */
    FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth    = FSMC_MemoryDataWidth_16b;  
    FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode    = FSMC_BurstAccessMode_Disable;  
    FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait   = FSMC_AsynchronousWait_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;  
    FSMC_NORSRAMInitStructure.FSMC_WrapMode           = FSMC_WrapMode_Disable;  
    FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive   = FSMC_WaitSignalActive_BeforeWaitState;  
    /* ����дʹ�ܴ� */
    FSMC_NORSRAMInitStructure.FSMC_WriteOperation     = FSMC_WriteOperation_Enable;  
    FSMC_NORSRAMInitStructure.FSMC_WaitSignal         = FSMC_WaitSignal_Disable;  
    /* ѡ����չģʽʹ�ܣ������ö���д�ò�ͬ��ʱ�� */
    FSMC_NORSRAMInitStructure.FSMC_ExtendedMode       = FSMC_ExtendedMode_Disable;          // FSMC_ExtendedMode_Enable
    FSMC_NORSRAMInitStructure.FSMC_WriteBurst         = FSMC_WriteBurst_Disable;      

 
    /* ���ö�дʱ�� */
    FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &FSMC_NORSRAMTiming;
    /* ����дʱ�� */
    FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct     = &FSMC_NORSRAMTiming;

    FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure); 

    /* Enable FSMC Bank1_SRAM Bank */
    FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1, ENABLE);           // FSMC_Bank1_NORSRAM4
}


#if 1

void TFT_WriteCmd(uint16 cmd)
{
    GPIO_ResetBits(GPIOD, GPIO_Pin_11);         // A16(RS)
    GPIO_ResetBits(GPIOD, GPIO_Pin_7);          // CS

    BOARD_LCD_R(BOARD_LCD_BASE) = cmd;

    GPIO_SetBits(GPIOD, GPIO_Pin_7);            // CS
}


void TFT_WriteData(uint16 dat)
{
    GPIO_SetBits(GPIOD, GPIO_Pin_11);           // A16(RS)
    GPIO_ResetBits(GPIOD, GPIO_Pin_7);          // CS

    BOARD_LCD_D(BOARD_LCD_BASE) = dat;

    GPIO_SetBits(GPIOD, GPIO_Pin_7);            // CS
}


uint16  TFT_Read(void)
{
    GPIO_SetBits(GPIOD, GPIO_Pin_11);           // A16(RS)
    GPIO_ResetBits(GPIOD, GPIO_Pin_7);          // CS

    return BOARD_LCD_R(BOARD_LCD_BASE);
}


void TFT_SetWindow(uint16 Xstart, uint16 Xend, uint16 Ystart, uint16 Yend)
{    
    TFT_WriteCmd(0x2a);   
    TFT_WriteData(Xstart>>8);
    TFT_WriteData(Xstart&0xff);
    TFT_WriteData(Xend>>8);
    TFT_WriteData(Xend&0xff);

    TFT_WriteCmd(0x2b);   
    TFT_WriteData(Ystart>>8);
    TFT_WriteData(Ystart&0xff);
    TFT_WriteData(Yend>>8);
    TFT_WriteData(Yend&0xff);

    TFT_WriteCmd(0x2c);
}

#endif


// ����Ϊָ��ɫ��
void TFT_ClearScreen(uint16 color)
{
    unsigned int i,j;

    TFT_SetWindow(0,TFT_XMAX,0,TFT_YMAX);

    for(i = 0; i <= TFT_XMAX; i++)
    for(j = 0; j <= TFT_YMAX; j++)
    {    
        TFT_WriteData(color);
    }
}


// ������
void GUI_Point(uint16 x, uint16 y, uint16 color)
{  
    TFT_SetWindow(x, x, y, y);              //���õ��λ��
    TFT_WriteData(color);                   //����    
}


// ��������ɫ��
void GUI_DispColor(unsigned int Xstart,unsigned int Xend,unsigned int Ystart,unsigned int Yend, unsigned int color)
{
    uint16  width, height;
    uint16  i,j;

    TFT_SetWindow(Xstart, Xend, Ystart, Yend);

    width  = Xend - Xstart;
    height = Yend - Ystart;

    for(i = 0; i <= width; i++)
    for(j = 0; j <= height; j++)
    {    
        TFT_WriteData(color);
    }
}


/****************************************************************************
* Function Name  : TFT_Init
* Description    : ��ʼ��LCD��
* Input          : None
* Output         : None
* Return         : None
****************************************************************************/
void TFT_Init(void)
{
    printf("\r\n TFT_Init...");

    TFT_GPIO_Config();
    TFT_FSMC_Config();

    delay_ms(100); 

    printf("\r\n    TFT_Init... 1 ");

    TFT_WriteCmd(0x0001);

    delay_ms(50); 

    printf("\r\n    TFT_Init... 2 ");

    TFT_WriteCmd(0XF7);
    TFT_WriteData(0xA9);
    TFT_WriteData(0x51);
    TFT_WriteData(0x2C);
    TFT_WriteData(0x82);

    TFT_WriteCmd(0xC0);
    TFT_WriteData(0x11);
    TFT_WriteData(0x09);

    TFT_WriteCmd(0xC1);
    TFT_WriteData(0x41);

    TFT_WriteCmd(0XC5);
    TFT_WriteData(0x00);
    TFT_WriteData(0x0A);
    TFT_WriteData(0x80);

    TFT_WriteCmd(0xB1);
    TFT_WriteData(0xB0);
    TFT_WriteData(0x11);

    TFT_WriteCmd(0xB4);
    TFT_WriteData(0x02);

    TFT_WriteCmd(0xB6);
    TFT_WriteData(0x02);
    TFT_WriteData(0x22);

    TFT_WriteCmd(0xB7);
    TFT_WriteData(0xc6);

    TFT_WriteCmd(0xBE);
    TFT_WriteData(0x00);
    TFT_WriteData(0x04);

    TFT_WriteCmd(0xE9);
    TFT_WriteData(0x00);

    TFT_WriteCmd(0x36);
    TFT_WriteData(0x08);

    TFT_WriteCmd(0x3A);
    TFT_WriteData(0x55);
    //TFT_WriteCmd(0x21);


    TFT_WriteCmd(0xE0);
    TFT_WriteData(0x00);
    TFT_WriteData(0x07);
    TFT_WriteData(0x10);
    TFT_WriteData(0x09);
    TFT_WriteData(0x17);
    TFT_WriteData(0x0B);
    TFT_WriteData(0x41);
    TFT_WriteData(0x89);
    TFT_WriteData(0x4B);
    TFT_WriteData(0x0A);
    TFT_WriteData(0x0C);
    TFT_WriteData(0x0E);
    TFT_WriteData(0x18);
    TFT_WriteData(0x1B);
    TFT_WriteData(0x0F);

    TFT_WriteCmd(0XE1);
    TFT_WriteData(0x00);
    TFT_WriteData(0x17);
    TFT_WriteData(0x1A);
    TFT_WriteData(0x04);
    TFT_WriteData(0x0E);
    TFT_WriteData(0x06);
    TFT_WriteData(0x2F);
    TFT_WriteData(0x45);
    TFT_WriteData(0x43);
    TFT_WriteData(0x02);
    TFT_WriteData(0x0A);
    TFT_WriteData(0x09);
    TFT_WriteData(0x32);
    TFT_WriteData(0x36);
    TFT_WriteData(0x0F);

    TFT_WriteCmd(0x11);
    delay_us(120);
    TFT_WriteCmd(0x29);

    printf("\r\n    TFT_Init... 3 ");

    delay_ms(50);

    printf("\r\n    TFT_Init... 4 ");

    delay_ms(50);

    TFT_WriteCmd(0x04);
 
    //printf("\r\n    TFT_Read %#x, %#x, %#x, %#x\r\n", TFT_Read(), TFT_Read(), TFT_Read(),TFT_Read());

    //TFT_WriteCmd(0x0202);

    TFT_ClearScreen(BLACK);
}



