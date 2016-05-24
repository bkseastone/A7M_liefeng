#include "interrupt.h"
#include "time.h"
#include "motor.h"
//#pragma optimize=speed
//void portc4_isr(void)
//{
//	if(LPLD_GPIO_IsPinxExt(PORTC, GPIO_Pin4))
//	{
//		delay();
//		if(PTCn_I(4)==0)
//		{
//			printf("Button1-PTB6 Interrupt!\r\n");
//			LPLD_GPIO_Toggle_b(PTC, 0);
//		}
//	}
//}


//#pragma optimize=speed
//void uart0_isr(void)
//{
//	int8 recv;
//	recv = LPLD_UART_GetChar(UART0);
//	LPLD_UART_PutChar(UART0, recv);
//}

