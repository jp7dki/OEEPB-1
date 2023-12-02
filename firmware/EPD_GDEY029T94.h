#ifndef _EPD_GDEY029T94_DEFINED
#define _EPD_GDEY029T94_DEFINED

#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"

//IO settings
#define BUSY_Pin 13 
#define RES_Pin 12 
#define DC_Pin 8 
#define CS_Pin 9
#define EPD_SCK 10
#define EPD_MOSI 11
#define EPD_MISO 26

#define EPD_W21_CS_0 gpio_put(CS_Pin,0)
#define EPD_W21_CS_1 gpio_put(CS_Pin,1)
#define EPD_W21_DC_0  gpio_put(DC_Pin,0)
#define EPD_W21_DC_1  gpio_put(DC_Pin,1)
#define EPD_W21_RST_0 gpio_put(RES_Pin,0)
#define EPD_W21_RST_1 gpio_put(RES_Pin,1)
#define isEPD_W21_BUSY gpio_get(BUSY_Pin)

#define EPD_SPI spi1

extern const unsigned char gImage_numdot[256];
extern const unsigned char gImage_white[256];
extern const unsigned char gImage_basemap[4736];
extern const unsigned char gImage_1[4736];

////////FUNCTION//////
   
void SPI_Write(unsigned char value);
void Epaper_Write_Command(unsigned char command);
void Epaper_Write_Data(unsigned char command);
//EPD
void Epaper_HW_SW_RESET(void);
void EPD_HW_Init(void); //Electronic paper initialization
void EPD_Update(void);

void EPD_Part_Init(void);//Local refresh initialization
void EPD_Part_Update(void); 

void EPD_WhiteScreen_White(void);
void EPD_DeepSleep(void);
//Display 
void EPD_WhiteScreen_ALL(const unsigned char *datas);
void EPD_SetRAMValue_BaseMap(const unsigned char * datas);
void EPD_Dis_Part(unsigned int x_start,unsigned int y_start,const unsigned char * datas,unsigned int PART_COLUMN,unsigned int PART_LINE);
void EPD_Dis_Part_myself(unsigned int x_startA,unsigned int y_startA,const unsigned char * datasA,
                         unsigned int x_startB,unsigned int y_startB,const unsigned char * datasB,
                         unsigned int x_startC,unsigned int y_startC,const unsigned char * datasC,
                         unsigned int x_startD,unsigned int y_startD,const unsigned char * datasD,
                         unsigned int x_startE,unsigned int y_startE,const unsigned char * datasE,
                         unsigned int PART_COLUMN,unsigned int PART_LINE
                        );
void EPD_HW_Init_Fast(void);
void EPD_WhiteScreen_ALL_Fast(const unsigned char *datas);
//Display canvas function
void EPD_HW_Init_GUI(void); //EPD init GUI
void EPD_Display(unsigned char *Image); 
void EPD_HW_Init_P(void);
void EPD_Standby(void);

void EPD_HW_Init_Fast(void);
void EPD_WhiteScreen_ALL_Fast(const unsigned char *datas);
void setup();

#endif