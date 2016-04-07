#include "UART.h"
#include "interrupt.h"

UART_InitTypeDef uart5_init_struct;

void uart5_init(void)
{
	uart5_init_struct.UART_Uartx = UART5; //使用UART5
	uart5_init_struct.UART_BaudRate = 115200; //设置波特率115200
	uart5_init_struct.UART_RxPin = PTE9;  //接收引脚为PTE9
	uart5_init_struct.UART_TxPin = PTE8;  //发送引脚为PTE8
	uart5_init_struct.UART_RxIntEnable = TRUE;  //使能接收中断
	uart5_init_struct.UART_RxIsr = uart5_isr;  //设置接收中断函数	
	//初始化UART
	LPLD_UART_Init(uart5_init_struct);
	LPLD_UART_EnableIrq(uart5_init_struct);
	LPLD_UART_PutCharArr(UART5, "Initial uart5 successful.\r\n", 29);
}