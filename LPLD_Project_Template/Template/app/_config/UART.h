#include "common.h"
#ifndef _UART_H_
#define _UART_H_

typedef struct
{
	void (*init)();
	void (*transmit)();
	uint16 DMA_ByteCnt;
	uint32 DMA_SourceAddr;
	char	Is_OK;
	struct{
		char	command_num;
		uint32	PID_P;
		uint32	PID_I;
		uint32	PID_D;
	} Command;
} BluetoothTypeDef;
void uart4_bluetooth_init(void);
void Bluetooth_DMA_isr(void);
void BluetoothStart(void);
//void uart0_init(void);
#endif
