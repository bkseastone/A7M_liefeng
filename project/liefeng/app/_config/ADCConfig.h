#include "common.h"

#ifndef _ADCCONFIG_H_
#define _ADCCONFIG_H_

typedef struct
{
	ADC_Type *ADC_Adcx;
	void (*isr)();
	AdcChnEnum_Type chn;
	uint8 PDB_TriggerInputSourceSel;
	PITx PIT_Pitx;
} myadc;
typedef enum
{
	OK = 0,
    error_init_pdb = 1,
	error_init_adc = 2,
	error_init_pit = 3
} ADCstatus;
ADCstatus adc_init(myadc *adc, uint32 PIT_PeriodUs);

#endif
