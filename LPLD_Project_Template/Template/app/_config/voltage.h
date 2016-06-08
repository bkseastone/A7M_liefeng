#include "common.h"
#ifndef _VOLTAGE_H_
#define _VOLTAGE_H_

typedef struct
{
	void	(*init)();
	ADC_Type	*ADC_Adcx;
	void	(*isr)();
	AdcChnEnum_Type chn;
	uint8	PDB_TriggerInputSourceSel;
	PITx	PIT_Pitx;
	uint32	PIT_PeriodUs;
	uint16	Vol;
} myadc;
void adc0_0P1_init(void);
void Battery_pit_init(void);
void Battery_pdb_init(void);
void adc0_isr(void);
void adc1_isr(void);
#endif
