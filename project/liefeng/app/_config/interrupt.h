#include "common.h"

#ifndef _INTERRUPT_H_
#define _INTERRUPT_H_

void portc4_isr(void);
void uart4_isr(void);
void uart0_isr(void);
void pit0_isr(void);
void pdb_isr(void);
void adc0_isr(void);
void adc1_isr(void);

#endif
