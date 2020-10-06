#define __SSD1306_C__
    #include "ssd1306.h"
#undef  __SSD1306_C__

#include "stm32f4xx_gpio.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_rcc.h"

#define MAX_COL        64
#define MAX_PAGE        6

#define CS_H            GPIO_WriteBit(GPIOI,GPIO_Pin_0,Bit_SET)
#define CS_L            GPIO_WriteBit(GPIOI,GPIO_Pin_0,Bit_RESET)

#define DC_H            GPIO_WriteBit(GPIOI,GPIO_Pin_2,Bit_SET)
#define DC_L            GPIO_WriteBit(GPIOI,GPIO_Pin_2,Bit_RESET)

#define RES_H           GPIO_WriteBit(GPIOI,GPIO_Pin_4,Bit_SET)
#define RES_L           GPIO_WriteBit(GPIOI,GPIO_Pin_4,Bit_RESET)


static void SSD1306_WriteData(uint8_t data);
static void SSD1306_WriteCmd(uint8_t cmd);
static void Spi_Init(void);
static void Gpio_Init(void);
static void SSD1306_UTIL_DelayMS(uint16_t wMS);
static void SSD1306_UTIL_DelayUS(uint16_t wUS);
static void SPI_SendByte(uint8_t data);
static void Set_Page_Address(uint8_t add);
static void Set_Column_Address(uint8_t add);
static void Display_Refresh(void);
static void SetPoint(uint8_t x,uint8_t y);
static void ClearPoint(uint8_t x,uint8_t y);

static uint8_t FrameBuffer[MAX_PAGE][MAX_COL];
static gl_sFONT *SSD1306_pFont;

void SSD1306_Init(void)
{
    Gpio_Init();
    Spi_Init();
    
    CS_H;
    RES_H;
    SSD1306_UTIL_DelayMS(1);
    RES_L;
    SSD1306_UTIL_DelayMS(1);
    RES_H;
    
    SSD1306_WriteCmd(0xAE);
    
    SSD1306_WriteCmd(0xD5);
    SSD1306_WriteCmd(0x80);
    
    SSD1306_WriteCmd(0xA8);
    SSD1306_WriteCmd(0x2F);
    
    SSD1306_WriteCmd(0xD3);
    SSD1306_WriteCmd(0x00);
    
    SSD1306_WriteCmd(0x40);
    
    SSD1306_WriteCmd(0x8D);
    SSD1306_WriteCmd(0x14);
    
    SSD1306_WriteCmd(0xA1);
    
    SSD1306_WriteCmd(0xC8);
    
    SSD1306_WriteCmd(0xDA);
    SSD1306_WriteCmd(0x12);
    
    SSD1306_WriteCmd(0x81);
    SSD1306_WriteCmd(0xFF);
    
    SSD1306_WriteCmd(0xD9);
    SSD1306_WriteCmd(0x22);
    
    SSD1306_WriteCmd(0xD8);
    SSD1306_WriteCmd(0x00);
    
    SSD1306_WriteCmd(0xA4);
    
    SSD1306_WriteCmd(0xA6);

    SSD1306_WriteCmd(0x20); //Set Memory Addressing Mode
    SSD1306_WriteCmd(0x00); //Horizontal Addressing Mode
    
    SSD1306_WriteCmd(0x21); //Set Column Address
    SSD1306_WriteCmd(32);   //Column Start : 32
    SSD1306_WriteCmd(95);   //Column End   : 95
    
    SSD1306_WriteCmd(0x22); //Set Page Address
    SSD1306_WriteCmd(2);    //Page Start : 2
    SSD1306_WriteCmd(7);    //Page End   : 7    
    
    SSD1306_WriteCmd(0xAF);    
    
    SSD1306_Clear();
}


void SSD1306_DrawLineH(uint8_t x,uint8_t y,uint8_t leng)
{
    for(int i=x; i<=x+leng; i++)
        SetPoint(i,y);
    
    Display_Refresh();
}

void SSD1306_DrawLineV(uint8_t x,uint8_t y,uint8_t leng)
{
    for(int i=y; i<=y+leng; i++)
        SetPoint(x,i);
    
    Display_Refresh();
}

void SSD1306_SetFont(gl_sFONT *pFont)
{
    SSD1306_pFont = pFont;
}

void SSD1306_Rect(uint8_t x,uint8_t y,uint8_t width,uint8_t height)
{
    SSD1306_DrawLineV(x,y,height);
    SSD1306_DrawLineV(x+width,y,height);
    SSD1306_DrawLineH(x,y,width);
    SSD1306_DrawLineH(x,y+height,width);
}


void SSD1306_DrawChar(uint16_t x, uint16_t y,uint16_t ch)
{
    uint16_t w,xa,ya;
    
    xa = ya = 0;
    
    for(int i=0; i<SSD1306_pFont->nData; i++)
    {
        uint8_t data = SSD1306_pFont->table[ch*SSD1306_pFont->nData + i];
        
        if( SSD1306_pFont->Width%8 == 0 )
            w = 8;
        else if( ((i+1)%(SSD1306_pFont->Width/8+1)) == 0 )
            w = SSD1306_pFont->Width%8;
        else 
            w = 8;
        
        for(int ix=0; ix<w; ix++)
        {
            if(data&0x80)       SetPoint(x+xa,y+ya);
            else                ClearPoint(x+xa,y+ya);
            
            data <<= 1;
            
            if(xa == (SSD1306_pFont->Width-1))
            {
                ya++;
                xa = 0;
                break;
            }
            else xa++;  
        }
    }
  Display_Refresh();
}

void SSD1306_DrawText(uint16_t x, uint16_t y, char *str)
{
    uint16_t cnt = 0;

    while(*str)
    {
         SSD1306_DrawChar(x+cnt*SSD1306_pFont->Width,y,*str++);
         cnt++;
    }
 
}

static void SetPoint(uint8_t x,uint8_t y)
{
    if(x > MAX_COL-1)      return;
    if(y > (MAX_PAGE*8)-1) return;
    
    FrameBuffer[y/8][x] |= (1<<(y%8));
}

static void ClearPoint(uint8_t x,uint8_t y)
{
    if(x > MAX_COL-1)      return;
    if(y > (MAX_PAGE*8)-1) return;
    
    FrameBuffer[y/8][x] &= ~(1<<(y%8));
}

static void Display_Refresh(void)
{
    Set_Page_Address(0);
    Set_Column_Address(0);
    for(int y=0; y<MAX_PAGE; y++)
        for(int x=0; x<MAX_COL; x++)
            SSD1306_WriteData(FrameBuffer[y][x]);
}

void SSD1306_Clear(void)
{
    for(int y=0; y<MAX_PAGE; y++)
        for(int x=0; x<MAX_COL; x++)
            FrameBuffer[y][x] = 0;
    
    Display_Refresh();
}

static void Set_Page_Address(uint8_t add)
{// 0 <= add <= 7
    add=0xb0|add;
    SSD1306_WriteCmd(add);
}

static void Set_Column_Address(uint8_t add)
{
    SSD1306_WriteCmd((0x10|(add>>4))+0x02);
    SSD1306_WriteCmd((0x0f&add));
}

static void SSD1306_WriteData(uint8_t data)
{
    CS_L;
    SSD1306_UTIL_DelayUS(1);
    DC_H;
    SSD1306_UTIL_DelayUS(1);
    SPI_SendByte(data);
    SSD1306_UTIL_DelayUS(1); 
    CS_H;
    SSD1306_UTIL_DelayUS(1);
}

static void SSD1306_WriteCmd(uint8_t cmd)
{
    CS_L;
    SSD1306_UTIL_DelayUS(1);
    DC_L;
    SSD1306_UTIL_DelayUS(1);
    SPI_SendByte(cmd);
    SSD1306_UTIL_DelayUS(1); 
    CS_H;
    SSD1306_UTIL_DelayUS(1);  
}

static void Spi_Init(void)
{
    SPI_InitTypeDef  SPI_InitStructure;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
   
    /*!< SPI configuration */
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;

    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI2, &SPI_InitStructure);

    /*!< Enable the sFLASH_SPI  */
    SPI_Cmd(SPI2, ENABLE);
}

static void Gpio_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;      
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI , ENABLE);	
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_2 | GPIO_Pin_4;
    GPIO_Init(GPIOI, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_3;
    GPIO_Init(GPIOI, &GPIO_InitStructure);
    
    GPIO_PinAFConfig(GPIOI, GPIO_PinSource1, GPIO_AF_SPI2);
    GPIO_PinAFConfig(GPIOI, GPIO_PinSource3, GPIO_AF_SPI2);
}

static void SSD1306_UTIL_DelayMS(uint16_t wMS)
{
    register uint16_t i;

    for (i=0; i<wMS; i++)
        SSD1306_UTIL_DelayUS(1000);         // 1000us => 1ms
}

static void SSD1306_UTIL_DelayUS(uint16_t wUS)
{
    volatile uint32_t Dly = (uint32_t)wUS*17;
    for(; Dly; Dly--);
}

static void SPI_SendByte(uint8_t data)
{
    while( ((SPI2->SR) & (SPI_FLAG_TXE)) != (SPI_FLAG_TXE) );
    SPI2->DR = data;
}