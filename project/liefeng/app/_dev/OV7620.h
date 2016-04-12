#include "common.h"

#ifndef _OV7620_H_
#define _OV7620_H_


#define OV_P 320u  //图像宽度
#define OV_H 240u  //图像高度
#define PHOTO_SIZE OV_P*OV_H

void ov7620_dma_start(void);
uint8 ov7620_init();
void OV_porta_Visr(void);
void OV_portb_Hisr(void);
void OV_gpio_init(void);
void OV_dma_init(void);
void OV_delay(void);
void OV_display(void);

#endif
