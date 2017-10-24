
#ifndef __LCD_DRIVER_H
#define __LCD_DRIVER_H

// 包含头文件
#include"stm32f10x.h"


#define BOARD_LCD_BASE                  (0x60000000)
#define BOARD_LCD_RS                    (1 << 16)

#define BOARD_LCD_R(baseAddr)            (*((uint32 *)(baseAddr)))
#define BOARD_LCD_D(baseAddr)            (*((uint32 *)(baseAddr + BOARD_LCD_RS)))


// 定义屏的大小
#define TFT_XMAX                BOARD_LCD_WIDTH                       //设置TFT屏的大小
#define TFT_YMAX                BOARD_LCD_HEIGHT


#define TFT_WriteCmdEXT         TFT_WriteCmd
#define TFT_WriteDataEXT        TFT_WriteData
#define TFT_SetWindowEXT        TFT_SetWindow


void TFT_Init(void);

void TFT_ClearScreen(uint16 color);

uint16  TFT_Read(void);

void  TFT_WriteCmd(uint16 cmd);
void  TFT_WriteData(uint16 dat);
void  TFT_SetWindow(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd);

void  GUI_DispColor(unsigned int Xstart,unsigned int Xend,unsigned int Ystart,unsigned int Yend, unsigned int color);
void  GUI_Point(uint16 x, uint16 y, uint16 color);


#endif



