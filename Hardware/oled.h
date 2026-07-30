#ifndef __OLED_H
#define __OLED_H
#include "ti_msp_dl_config.h"
#include "board.h"

#define OLED_CMD  0 //Command //写命令
#define OLED_DATA 1 //Data //写数据

#define OLED_WIDTH 128
#define OLED_PAGES 8

/* Most I2C OLED modules use 7-bit address 0x3C, sent as 0x78 on the bus. */
#define OLED_I2C_ADDR 0x78

/* 1.3 inch I2C OLED modules are commonly SH1106 and need a 2-column offset.
 * If your module is SSD1306, change this value to 0.
 */
#define OLED_COLUMN_OFFSET 2

extern uint8_t OLED_GRAM[OLED_WIDTH][OLED_PAGES];

//Oled control function
//OLED控制用函数
void OLED_WR_Byte(uint8_t dat,uint8_t cmd);
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Refresh_Gram(void);
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(uint8_t x,uint8_t y,uint8_t t);
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t size,uint8_t mode);
void OLED_ShowNumber(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size);
void OLED_ShowString(uint8_t x,uint8_t y,const char *p);
void OLED_SCLK_Clr(void);
void OLED_SCLK_Set(void);
void OLED_SDIN_Clr(void);
void OLED_SDIN_Set(void);
#define CNSizeWidth  16
#define CNSizeHeight 16
//extern char Hzk16[][16];
void OLED_ShowCHinese(uint8_t x,uint8_t y,uint8_t no,uint8_t font_width,uint8_t font_height);
void OLED_Set_Pos(unsigned char x, unsigned char y);
#endif


