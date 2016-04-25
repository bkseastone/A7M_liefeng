#include "UART.h"
#include "interrupt.h"

UART_InitTypeDef uart0_init_struct;
UART_InitTypeDef uart4_init_struct;

#pragma optimize=size
void uart4_bluetooth_init(void)
{
	uart4_init_struct.UART_Uartx = UART4; //使用UART5
	uart4_init_struct.UART_BaudRate = 115200; //设置波特率115200
	uart4_init_struct.UART_RxPin = PTE25;  //接收引脚为PTE25
	uart4_init_struct.UART_TxPin = PTE24;  //发送引脚为PTE24
	uart4_init_struct.UART_RxIntEnable = TRUE;  //使能接收中断
	uart4_init_struct.UART_RxIsr = uart4_isr;  //设置接收中断函数	
	//初始化UART
	LPLD_UART_Init(uart4_init_struct);
	LPLD_UART_EnableIrq(uart4_init_struct);
	LPLD_UART_PutCharArr(UART4, "Initial uart4 successful.\r\n", 29);
}

#pragma optimize=size
void uart0_init(void)
{
	uart0_init_struct.UART_Uartx = UART0; //使用UART5
	uart0_init_struct.UART_BaudRate = 115200; //设置波特率115200
	uart0_init_struct.UART_RxPin = PTA15;  //接收引脚为PTE9
	uart0_init_struct.UART_TxPin = PTA14;  //发送引脚为PTE8
	uart0_init_struct.UART_RxIntEnable = TRUE;  //使能接收中断
	uart0_init_struct.UART_RxIsr = uart0_isr;  //设置接收中断函数	
	//初始化UART
	LPLD_UART_Init(uart0_init_struct);
	LPLD_UART_EnableIrq(uart0_init_struct);
	LPLD_UART_PutCharArr(UART0, "Initial uart0 successful.\r\n", 29);
}