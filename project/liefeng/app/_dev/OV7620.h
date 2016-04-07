#include "common.h"

#ifndef _OV7620_H_
#define _OV7620_H_


#define H 320  //图像宽度
#define V 240  //图像高度
#define PHOTO_SIZE H*V

void ov7620_dma_start(void);
uint8 ov7620_init();
void OV_porta_Visr(void);
void OV_portb_Hisr(void);
void OV_gpio_init(void);
void OV_dma_init(void);
void OV_delay(void);
void OV_display(void);

#endif
