#include "common.h"
#include "GPIOConfig.h"
#include "interrupt.h"
#include "time.h"
#include "UART.h"
#include "ADCConfig.h"
#include "OV7725_eagle.h"
#include "Capture.h"

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
uint32 Freq2 = 0; 
uint32 Freq3 = 0;

