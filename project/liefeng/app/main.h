#include "common.h"
#include "GPIOConfig.h"
#include "interrupt.h"
#include "time.h"
#include "UART.h"
#include "ADCConfig.h"
#include "OV7620.h"
#include "Capture.h"

/**
 * @brief Macro to access a single bit of SRAM (bit band region
 *        0x20000000 to 0x200FFFFF) using the bit-band alias region access.
 * @param Line Picture`s lines to access, ranging in [0~inf].
 * @param Pix Lines` pixels to access, ranging in [0~OV_P-1].
 * @return Value of the targeted bit in the bit band region.
 */
#define SRAM_U_BASE						0x20000000u
#define OV_binary_BASE					SRAM_U_BASE
#define OV_pics_LineNum(Lines)			(4u*((uint32_t)Lines*OV_P))
#define OV_binary_data(Line,Pix) (*((uint32_t volatile*)(0x22000000u + (OV_pics_LineNum(Line)) + (4u*(uint32_t)Pix))))

// constant for initializing peripherals
mypwm Pwm0_A4={FTM0, FTM_Ch1, PTA4};  // 舵机
mypwm Pwm1_A9={FTM1, FTM_Ch1, PTA9};  // 电机
mypwm Pwm1_A8={FTM1, FTM_Ch0, PTA8};
myadc Adc0_0P1={ADC0, &adc0_isr, DAD1, TRIGGER_PIT0, PIT0};
myadc Adc1_1P1={ADC1, &adc1_isr, DAD1, TRIGGER_PIT1, PIT1};
myic ic2_A10={FTM2, FTM_Ch0, PTA10, &ic2_isr};  //编码器
myic ic2_A11={FTM2, FTM_Ch1, PTA11, &ic3_isr};

// Global variables (online)
uint8 Is_DispPhoto; //图像采集完成标志
uint8 Pix_Data[PHOTO_SIZE]; //采集320行 240列的图像数据  
uint32 Freq2 = 0; 
uint32 Freq3 = 0;

