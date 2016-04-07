#include "common.h"
#include "GPIOConfig.h"
#include "interrupt.h"
#include "time.h"
#include "UART.h"
#include "ADCConfig.h"
#include "OV7620.h"
#include "Capture.h"

// constant for initializing peripherals
mypwm Pwm0_C1={FTM0, FTM_Ch0, PTC1};
mypwm Pwm1_A9={FTM1, FTM_Ch1, PTA9};
mypwm Pwm1_A8={FTM1, FTM_Ch0, PTA8};
mypwm Pwm2_B19={FTM2, FTM_Ch1, PTB19};
mypwm Pwm3_D0={FTM3, FTM_Ch0, PTD0};
myadc Adc0_0P1={ADC0, &adc0_isr, DAD1, TRIGGER_PIT0, PIT0};
myadc Adc1_1P1={ADC1, &adc1_isr, DAD1, TRIGGER_PIT1, PIT1};
myic ic2_B18={FTM2, FTM_Ch0, PTB18, &ic2_isr};
myic ic3_D0={FTM3, FTM_Ch0, PTD0, &ic3_isr};
// Global variables (online)
uint8 Is_DispPhoto; //图像采集完成标志
uint8 Pix_Data[PHOTO_SIZE]; //采集320行 240列的图像数据  
uint32 Freq2 = 0; 
uint32 Freq3 = 0;
