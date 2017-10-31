//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//�о�԰����
//���̵�ַ��http://shop73023976.taobao.com/?spm=2013.1.0.0.M4PqC2
//
//  �� �� ��   : main.c
//  �� �� ��   : v2.0
//  ��    ��   : HuangKai
//  ��������   : 2014-0101
//  ����޸�   : 
//  ��������   : OLED 4�ӿ���ʾ����(51ϵ��)
//              ˵��: 
//              ----------------------------------------------------------------
//              GND    ��Դ��
//              VCC  ��5V��3.3v��Դ
//              D0   ��PD6��SCL��
//              D1   ��PD7��SDA��
//              RES  ��PD4
//              DC   ��PD5
//              CS   ��PD10               
//              ----------------------------------------------------------------
// �޸���ʷ   :
// ��    ��   : 
// ��    ��   : HuangKai
// �޸�����   : �����ļ�
//��Ȩ���У�����ؾ���
//Copyright(C) �о�԰����2014/3/16
//All rights reserved
//******************************************************************************/
#ifndef __OLED_H
#define __OLED_H 
#include "stm32f1xx_hal.h"
#include "stdlib.h"	    	
//OLEDģʽ����
//0:4�ߴ���ģʽ
//1:����8080ģʽ
#define OLED_MODE 0
#define SIZE 12
#define XLevelL		0x00
#define XLevelH		0x10
#define Max_Column	128
#define Max_Row		64
#define	Brightness	0xFF 
#define X_WIDTH 	128
#define Y_WIDTH 	64	    						  
//-----------------OLED�˿ڶ���----------------  					   

#define OLED_CS_Clr()  HAL_GPIO_WritePin(oled_cs_GPIO_Port,oled_cs_Pin,GPIO_PIN_RESET)//CS
#define OLED_CS_Set()  HAL_GPIO_WritePin(oled_cs_GPIO_Port,oled_cs_Pin,GPIO_PIN_SET)

#define OLED_RST_Clr() HAL_GPIO_WritePin(oled_res_GPIO_Port,oled_res_Pin,GPIO_PIN_RESET)//RES
#define OLED_RST_Set() HAL_GPIO_WritePin(oled_res_GPIO_Port,oled_res_Pin,GPIO_PIN_SET)

#define OLED_DC_Clr() HAL_GPIO_WritePin(oled_ds_GPIO_Port,oled_ds_Pin,GPIO_PIN_RESET)//DC
#define OLED_DC_Set() HAL_GPIO_WritePin(oled_ds_GPIO_Port,oled_ds_Pin,GPIO_PIN_SET)

#define OLED_WR_Clr() HAL_GPIO_WritePin(GPIOG,GPIO_Pin_14,1)
#define OLED_WR_Set() HAL_GPIO_WritePin(GPIOG,GPIO_Pin_14,0)

#define OLED_RD_Clr() HAL_GPIO_WritePin(GPIOG,GPIO_Pin_13,1)
#define OLED_RD_Set() HAL_GPIO_WritePin(GPIOG,GPIO_Pin_13,0)



//PC0~7,��Ϊ������
#define DATAOUT(x) GPIO_Write(GPIOC,x);//���  
//ʹ��4�ߴ��нӿ�ʱʹ�� 

#define OLED_SCLK_Clr() HAL_GPIO_WritePin(oled_d0_GPIO_Port,oled_d0_Pin,GPIO_PIN_RESET)//CLK
#define OLED_SCLK_Set() HAL_GPIO_WritePin(oled_d0_GPIO_Port,oled_d0_Pin,GPIO_PIN_SET)

#define OLED_SDIN_Clr() HAL_GPIO_WritePin(oled_d1_GPIO_Port,oled_d1_Pin,GPIO_PIN_RESET)//DIN
#define OLED_SDIN_Set() HAL_GPIO_WritePin(oled_d1_GPIO_Port,oled_d1_Pin,GPIO_PIN_SET)

 		     
#define OLED_CMD  0	//д����
#define OLED_DATA 1	//д����


//OLED�����ú���
void OLED_WR_Byte(uint8_t dat,uint8_t cmd);	    
void OLED_Display_On(void);
void OLED_Display_Off(void);	   							   		    
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(uint8_t x,uint8_t y,uint8_t t);
void OLED_Fill(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2,uint8_t dot);
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr);
void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size);
void OLED_ShowString(uint8_t x,uint8_t y, uint8_t *p);
void OLED_Set_Pos(uint8_t x, uint8_t y);
void OLED_ShowCHinese(uint8_t x,uint8_t y,uint8_t no);
void OLED_DrawBMP(uint8_t x0, uint8_t y0,uint8_t x1, uint8_t y1,const uint8_t BMP[]);
void OLED_DrawPoint(uint8_t x,uint8_t y,uint8_t write);
void OLED_DrawLine(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2);
void OLED_DrawBlock(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2);
void OLED_DrawRect(uint8_t x1,uint8_t y1,uint8_t width,uint8_t height,uint8_t fill);
void OLED_CirclePoint(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2);
void OLED_DrawCircle(uint8_t x,uint8_t y,uint8_t r);
void oled_printf(uint8_t x,uint8_t y,const char* format,...);
void OLED_DisplayChar(uint8_t x,uint8_t y,uint8_t chr);

void OLED_ClearScreen(void);
void OLED_Updata(void);
void OLED_UpdataPart(uint8_t row1,uint8_t col1,uint8_t row2,uint8_t col2,uint8_t direct);
extern const uint8_t fast_bmp[288];
extern const uint8_t slow_bmp[288];
#endif  
	 



