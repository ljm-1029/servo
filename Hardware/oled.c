#include "oled.h"
#include "stdlib.h"
#include "oledfont.h"

#include "board.h"
#include "stdio.h"

uint8_t OLED_GRAM[OLED_WIDTH][OLED_PAGES];

static void OLED_I2C_Delay(void)
{
    delay_us(2);
}

static void OLED_SDA_Input(void)
{
    DL_GPIO_disableOutput(OLED_SDA_PORT, OLED_SDA_PIN_SDA_PIN);
}

static void OLED_SDA_Output(void)
{
    DL_GPIO_enableOutput(OLED_SDA_PORT, OLED_SDA_PIN_SDA_PIN);
}

static uint8_t OLED_Read_SDA(void)
{
    return (DL_GPIO_readPins(OLED_SDA_PORT, OLED_SDA_PIN_SDA_PIN) != 0U);
}

static void OLED_I2C_Start(void)
{
    OLED_SDIN_Set();
    OLED_SCLK_Set();
    OLED_I2C_Delay();
    OLED_SDIN_Clr();
    OLED_I2C_Delay();
    OLED_SCLK_Clr();
}

static void OLED_I2C_Stop(void)
{
    OLED_SCLK_Clr();
    OLED_SDIN_Clr();
    OLED_I2C_Delay();
    OLED_SCLK_Set();
    OLED_I2C_Delay();
    OLED_SDIN_Set();
    OLED_I2C_Delay();
}

static uint8_t OLED_I2C_WriteByte(uint8_t dat)
{
    uint8_t i;
    uint8_t ack;

    OLED_SDA_Output();
    for(i = 0; i < 8; i++)
    {
        OLED_SCLK_Clr();
        if(dat & 0x80)
        {
            OLED_SDIN_Set();
        }
        else
        {
            OLED_SDIN_Clr();
        }
        OLED_I2C_Delay();
        OLED_SCLK_Set();
        OLED_I2C_Delay();
        dat <<= 1;
    }

    OLED_SCLK_Clr();
    OLED_SDIN_Set();
    OLED_SDA_Input();
    OLED_I2C_Delay();
    OLED_SCLK_Set();
    OLED_I2C_Delay();
    ack = OLED_Read_SDA();
    OLED_SCLK_Clr();
    OLED_SDA_Output();
    OLED_SDIN_Set();

    return ack;
}

static void OLED_I2C_WriteDataBurst(uint8_t page)
{
    uint8_t n;

    OLED_I2C_Start();
    (void)OLED_I2C_WriteByte(OLED_I2C_ADDR);
    (void)OLED_I2C_WriteByte(0x40);
    for(n = 0; n < OLED_WIDTH; n++)
    {
        (void)OLED_I2C_WriteByte(OLED_GRAM[n][page]);
    }
    OLED_I2C_Stop();
}

/**************************************************************************
Function: Refresh the OLED screen
Input   : none
Output  : none
函数功能：刷新OLED屏幕
入口参数：无
返回  值：无
**************************************************************************/
void OLED_Refresh_Gram(void)
{
    uint8_t i;
    for(i = 0; i < OLED_PAGES; i++)
    {
        OLED_WR_Byte(0xb0 + i, OLED_CMD); //Set page address (0~7) //设置页地址（0~7）
        OLED_WR_Byte(0x00 + (OLED_COLUMN_OFFSET & 0x0f), OLED_CMD);
        OLED_WR_Byte(0x10 + ((OLED_COLUMN_OFFSET >> 4) & 0x0f), OLED_CMD);
        OLED_I2C_WriteDataBurst(i);
    }
}
/**************************************************************************
Function: Refresh the OLED screen
Input   : Dat: data/command to write, CMD: data/command flag 0, represents the command;1, represents data
Output  : none
函数功能：向OLED写入一个字节
入口参数：dat:要写入的数据/命令，cmd:数据/命令标志 0,表示命令;1,表示数据
返回  值：无
**************************************************************************/
void OLED_WR_Byte(uint8_t dat,uint8_t cmd)
{
    OLED_I2C_Start();
    (void)OLED_I2C_WriteByte(OLED_I2C_ADDR);
    (void)OLED_I2C_WriteByte(cmd ? 0x40 : 0x00);
    (void)OLED_I2C_WriteByte(dat);
    OLED_I2C_Stop();
}
/**************************************************************************
Function: Turn on the OLED display
Input   : none
Output  : none
函数功能：开启OLED显示
入口参数：无
返回  值：无
**************************************************************************/
void OLED_Display_On(void)
{
    OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}
/**************************************************************************
Function: Turn off the OLED display
Input   : none
Output  : none
函数功能：关闭OLED显示
入口参数：无
返回  值：无
**************************************************************************/
void OLED_Display_Off(void)
{
    OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}
/**************************************************************************
Function: Screen clear function, clear the screen, the entire screen is black, and did not light up the same
Input   : none
Output  : none
函数功能：清屏函数,清完屏,整个屏幕是黑色的，和没点亮一样
入口参数：无
返回  值：无
**************************************************************************/
void OLED_Clear(void)
{
    uint8_t i,n;
    for(i = 0; i < OLED_PAGES; i++)for(n = 0; n < OLED_WIDTH; n++)OLED_GRAM[n][i]=0X00;
    OLED_Refresh_Gram(); //Update the display //更新显示
}
/**************************************************************************
Function: Draw point
Input   : x,y: starting coordinate;T :1, fill,0, empty
Output  : none
函数功能：画点
入口参数：x,y :起点坐标; t:1,填充,0,清空
返回  值：无
**************************************************************************/
void OLED_DrawPoint(uint8_t x,uint8_t y,uint8_t t)
{
    uint8_t pos,bx,temp=0;
    if(x>(OLED_WIDTH - 1)||y>63)return;//超出范围了.
    pos=7-y/8;
    bx=y%8;
    temp=1<<(7-bx);
    if(t)OLED_GRAM[x][pos]|=temp;
    else OLED_GRAM[x][pos]&=~temp;
}
/**************************************************************************
Function: Displays a character, including partial characters, at the specified position
Input   : x,y: starting coordinate;Len: The number of digits;Size: font size;Mode :0, anti-white display,1, normal display
Output  : none
函数功能：在指定位置显示一个字符,包括部分字符
入口参数：x,y :起点坐标; len :数字的位数; size:字体大小; mode:0,反白显示,1,正常显示
返回  值：无
**************************************************************************/
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t size,uint8_t mode)
{
    uint8_t temp,t,t1;
    uint8_t y0=y;
    chr=chr-' '; //Get the offset value //得到偏移后的值
    for(t=0;t<size;t++)
    {
        if(size==12)temp=oled_asc2_1206[chr][t];  //Invoke 1206 font   //调用1206字体
        else temp=oled_asc2_1608[chr][t];         //Invoke the 1608 font //调用1608字体
        for(t1=0;t1<8;t1++)
        {
            if(temp&0x80)OLED_DrawPoint(x,y,mode);
            else OLED_DrawPoint(x,y,!mode);
            temp<<=1;
            y++;
            if((y-y0)==size)
            {
                y=y0;
                x++;
                break;
            }
        }
    }
}
/**************************************************************************
Function: Find m to the NTH power
Input   : m: base number, n: power number
Output  : none
函数功能：求m的n次方的函数
入口参数：m：底数，n：次方数
返回  值：无
**************************************************************************/
uint32_t oled_pow(uint8_t m,uint8_t n)
{
    uint32_t result=1;
    while(n--)result*=m;
    return result;
}

/**************************************************************************
Function: Displays 2 numbers
Input   : x,y: starting coordinate;Len: The number of digits;Size: font size;Mode: mode, 0, fill mode, 1, overlay mode;Num: value (0 ~ 4294967295);
Output  : none
函数功能：显示2个数字
入口参数：x,y :起点坐标; len :数字的位数; size:字体大小; mode:模式, 0,填充模式, 1,叠加模式; num:数值(0~4294967295);
返回  值：无
**************************************************************************/
void OLED_ShowNumber(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size)
{
    uint8_t t,temp;
    uint8_t enshow=0;
    for(t=0;t<len;t++)
    {
        temp=(num/oled_pow(10,len-t-1))%10;
        if(enshow==0&&t<(len-1))
        {
            if(temp==0)
            {
                OLED_ShowChar(x+(size/2)*t,y,' ',size,1);
                continue;
            }else enshow=1;

        }
        OLED_ShowChar(x+(size/2)*t,y,temp+'0',size,1);
    }
}
/**************************************************************************
Function: Display string
Input   : x,y: starting coordinate;*p: starting address of the string
Output  : none
函数功能：显示字符串
入口参数：x,y :起点坐标; *p:字符串起始地址
返回  值：无
**************************************************************************/
void OLED_ShowString(uint8_t x,uint8_t y,const char *p)
{
#define MAX_CHAR_POSX 122
#define MAX_CHAR_POSY 58
    while(*p!='\0')
    {
        if(x>MAX_CHAR_POSX){x=0;y+=16;}
        if(y>MAX_CHAR_POSY){y=x=0;OLED_Clear();}
        OLED_ShowChar(x,y,*p,12,1);
        x+=8;
        p++;
    }
}
/**************************************************************************
Function: Initialize the OLED
Input   : none
Output  : none
函数功能：初始化OLED
入口参数: 无
返回  值：无
**************************************************************************/
void OLED_Init(void)
{
    OLED_SCLK_Set();
    OLED_SDIN_Set();

    delay_ms(120);

    OLED_WR_Byte(0xAE,OLED_CMD); //Close display //关闭显示
    OLED_WR_Byte(0xD5,OLED_CMD); //Display clock divide ratio/oscillator frequency
    OLED_WR_Byte(0x80,OLED_CMD);
    OLED_WR_Byte(0xA8,OLED_CMD); //Multiplex ratio
    OLED_WR_Byte(0x3F,OLED_CMD);
    OLED_WR_Byte(0xD3,OLED_CMD); //Display offset
    OLED_WR_Byte(0x00,OLED_CMD);
    OLED_WR_Byte(0x40,OLED_CMD); //Display start line
    OLED_WR_Byte(0xAD,OLED_CMD); //SH1106 DC-DC control
    OLED_WR_Byte(0x8B,OLED_CMD);
    OLED_WR_Byte(0xA1,OLED_CMD); //Segment remap
    OLED_WR_Byte(0xC0,OLED_CMD); //COM scan direction
    OLED_WR_Byte(0xDA,OLED_CMD); //COM pins hardware configuration
    OLED_WR_Byte(0x12,OLED_CMD);
    OLED_WR_Byte(0x81,OLED_CMD); //Contrast
    OLED_WR_Byte(0xEF,OLED_CMD);
    OLED_WR_Byte(0xD9,OLED_CMD); //Pre-charge period
    OLED_WR_Byte(0x1F,OLED_CMD);
    OLED_WR_Byte(0xDB,OLED_CMD); //VCOMH deselect level
    OLED_WR_Byte(0x40,OLED_CMD);
    OLED_WR_Byte(0xA4,OLED_CMD); //Entire display on from RAM
    OLED_WR_Byte(0xA6,OLED_CMD); //Normal display
    OLED_WR_Byte(0xAF,OLED_CMD); //Open display //开启显示
    OLED_Clear();
}

/**************************************************************************
Function: Display character
Input   : x: indicates the horizontal coordinates displayed; Y: the vertical coordinates that show the display;
          no: the line number in the array of the Chinese character (module) in the hzk-and "array", which is determined by the line number to determine the characters shown in the array,
          The value of the width of the font here must be consistent with the size of the dot matrix value of the use of the word mold.
          font_height: the font is high for the use of the word mold, because my screen pixels are 32hours, 128----0~ 7, and four bits per page
Output  : none
Note: this method is used to show that the Chinese character must satisfy the size of the word that the word model generates the software to generate the same size as the dot matrix
函数功能：显示汉字
入口参数: x：表示显示的水平坐标; y: 表示显示的垂直坐标;
          no: 表示要显示的汉字（模组）在hzk[][]数组中的行号,通过行号来确定在数组中要显示的汉字,
              这里字体的宽font_width的值必须与用字模制作软件生成字模时的点阵值大小一致;
          font_height:为用字模制作软件生成字模时字体的高,由于我的屏像素为32*128-----0~7共8页，每页4个位
返回  值：无
注意：用这种方法来显示汉字一定要满足用字模生成软件生成的字宽与点阵大小相同才行，否者容易乱码
**************************************************************************/
void OLED_ShowCHinese(uint8_t x,uint8_t y,uint8_t no,uint8_t font_width,uint8_t font_height)
{
    uint8_t t, i;
    for(i=0;i<(font_height/8);i++)
    {
        OLED_Set_Pos(x,y+i);
        for(t=0;t<font_width;t++)
        {
            OLED_WR_Byte(Hzk16[(font_height/8)*no+i][t],OLED_DATA);
        }
    }
}
/**************************************************************************
Function: Set the coordinates (position) displayed on the screen.
Input   : x, y: starting point coordinates
Output  : none
函数功能：设置汉字在屏幕上显示的坐标（位置）
入口参数: x,y :起点坐标
返回  值：无
**************************************************************************/
void OLED_Set_Pos(unsigned char x, unsigned char y)
{
    x += OLED_COLUMN_OFFSET;
    OLED_WR_Byte(0xb0+y,OLED_CMD);
    OLED_WR_Byte(((x&0xf0)>>4)|0x10,OLED_CMD);
    OLED_WR_Byte((x&0x0f),OLED_CMD);
}


void OLED_SCLK_Clr(void)
{
    DL_GPIO_enableOutput(OLED_SCL_PORT,OLED_SCL_PIN_SCL_PIN);
    DL_GPIO_clearPins(OLED_SCL_PORT,OLED_SCL_PIN_SCL_PIN); ////SCL
}

void OLED_SCLK_Set(void)
{
    DL_GPIO_setPins(OLED_SCL_PORT,OLED_SCL_PIN_SCL_PIN); ////SCL
    DL_GPIO_enableOutput(OLED_SCL_PORT,OLED_SCL_PIN_SCL_PIN);
}

void OLED_SDIN_Clr(void)
{
    OLED_SDA_Output();
    DL_GPIO_clearPins(OLED_SDA_PORT,OLED_SDA_PIN_SDA_PIN); ////SDA
}

void OLED_SDIN_Set(void)
{
    DL_GPIO_setPins(OLED_SDA_PORT,OLED_SDA_PIN_SDA_PIN); ////SDA released high
    OLED_SDA_Input();
}




