#include "common.h"
#include "GPIOConfig.h"
#include "interrupt.h"
#include "time.h"
#include "UART.h"
#include "ADCConfig.h"
#include "OV7725_eagle.h"
#include "Capture.h"
#include "PID.h"

// constant for initializing peripherals
mypwm Pwm0_C1={FTM0, FTM_Ch0, PTC1, 200,  (10000 - (uint32)(10000*(0.5+(2.0f*(90.0-0)/180))/(1000/200))), 1};  // 舵机
mypwm Pwm1_A9={FTM1, FTM_Ch1, PTA9, 500, 1000, 2};  // 电机
mypwm Pwm1_A8={FTM1, FTM_Ch0, PTA8, 500, 1000, 3};
myadc Adc0_0P1={ADC0, &adc0_isr, DAD1, TRIGGER_PIT0, PIT0};
myadc Adc1_1P1={ADC1, &adc1_isr, DAD1, TRIGGER_PIT1, PIT1};
myic ic2_A10={FTM2, FTM_Ch0, PTA10, &ic2_isr};  //编码器
myic ic2_A11={FTM2, FTM_Ch1, PTA11, &ic3_isr};
PIDTypeDef motor_PID = {PID_KP, PID_KI, PID_KD, PID_NOWE, PID_LASTE, PID_PASTE, PID_OUT}; // 电机PID

// Global variables (online)
uint8 Is_DispPhoto; //图像采集完成标志
#define TARGET_VELOSITY				100 // 10(cm/s)
uint32 Freq2 = 2; 
uint32 Freq3 = 3;
uint32 duty = 0; // (00.00%)
uint32 velosity = 0; // (cm/s)


