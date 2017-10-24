/*****************************************************************************************
版权所有:        ---------  
版本号:            1.0.0
生成日期:        2008.08.31
文件名:            gui_loadbmp.h
作者:            影舞者
功能说明:        显示彩色图形
其它说明:        无
所属文件关系:    本文件为工程规约代码文件

修改记录:
记录1:
修改者:
修改日期:
修改内容:
修改原因:
*****************************************************************************************/

#ifndef GUI_LOADBMP_PRESENT
#define GUI_LOADBMP_PRESENT

#ifdef  GUI_LOADBMP_GLOBALS                                            
    #define GUI_LOADBMP_EXT                                                
#else
    #define GUI_LOADBMP_EXT    extern                                
#endif 




/*****************************************************************************************
版权所有:   影舞者
版本号:     1.0.0
生成日期:   2008.08.31
功能说明:   函数接口
其它说明:    无
*****************************************************************************************/
typedef struct {    
    unsigned int width;
    unsigned int height;
    unsigned int bps;
    unsigned int *bmp_data;
}GUI_BITMAP;  




/*****************************************************************************************
版权所有:   影舞者
版本号:     1.0.0
生成日期:   2008.08.31
功能说明:   函数接口
其它说明:    无
*****************************************************************************************/
GUI_LOADBMP_EXT  void  GUI_LoadBmp(INT16U x, INT16U y, GUI_BITMAP *bmp);




#endif





