#include "UART.h"
#include "interrupt.h"
#include "motor.h"
#include "PID.h"
extern MotorTypeDef		*MotorF;
extern MotorTypeDef		*MotorB;
BluetoothTypeDef Bluetooth_struct = {&uart4_bluetooth_init, &BluetoothStart,
									2,  //字节数
									0,  //源地址
									0,	//FIFO传输完成标志位
									{0,	//command table
									0, 0, 0 //PID
									}
									};
BluetoothTypeDef *Bluetooth = &Bluetooth_struct;
UART_InitTypeDef uart4_init_struct;
//DMA_InitTypeDef uart4_dma_init_struct;
#pragma optimize=size
void uart4_bluetooth_init(void)
{
	uart4_init_struct.UART_Uartx = UART4;
	uart4_init_struct.UART_BaudRate = 115200; 
	uart4_init_struct.UART_RxPin = PTE25; 
	uart4_init_struct.UART_TxPin = PTE24; 
//	uart4_init_struct.UART_TxDMAEnable = TRUE;
//	uart4_init_struct.UART_TxIntEnable = TRUE;
	uart4_init_struct.UART_RxIntEnable = TRUE; 
	uart4_init_struct.UART_RxIsr = uart4_isr; 

	LPLD_UART_Init(uart4_init_struct);
	LPLD_UART_EnableIrq(uart4_init_struct);
	
//	uart4_dma_init_struct.DMA_CHx = DMA_CH1; 
//	uart4_dma_init_struct.DMA_Req = UART4_TRAN_DMAREQ; 
//	uart4_dma_init_struct.DMA_MajorLoopCnt = Bluetooth->DMA_ByteCnt;
//	uart4_dma_init_struct.DMA_MinorByteCnt = 1; 
//	uart4_dma_init_struct.DMA_SourceAddr = Bluetooth->DMA_SourceAddr;
//	uart4_dma_init_struct.DMA_DestAddr =  0x400EA007u;     // UART4_D
//	uart4_dma_init_struct.DMA_SourceAddrOffset = 1;       //源地址偏移：每次读入增加1
//	uart4_dma_init_struct.DMA_AutoDisableReq = TRUE;    //自动禁用请求
//	uart4_dma_init_struct.DMA_MajorCompleteIntEnable = TRUE;
//	uart4_dma_init_struct.DMA_Isr = Bluetooth_DMA_isr;
//	//初始化DMA
//	LPLD_DMA_Init(uart4_dma_init_struct);
//	LPLD_DMA_EnableIrq(uart4_dma_init_struct);
	
	//LPLD_UART_PutCharArr(UART4, "Initial uart4 successful.\r\n", 29);
}

#pragma optimize=speed
void uart4_isr(void)
{
	static uint32 tmp_duty = 0;
	int8 recv;
	recv = LPLD_UART_GetChar(UART4);
	switch(recv)
	{
		case 'A': 
		   Bluetooth->Command.command_num = 0;
		   Bluetooth->Command.PID_P = 0;
		   Bluetooth->Command.PID_I = 0;
		   Bluetooth->Command.PID_D = 0;
		   break;
		case '0':
		   if((Bluetooth->Command.command_num) == 0xF0){
				Bluetooth->Command.command_num += 0x01;	//0xF1
		   		tmp_duty = MotorB->Duty;
		   		MotorB->Duty = 0;
				LPLD_FTM_PWM_ChangeDuty(MotorB->FTM_Ftmx, MotorB->chn, MotorB->Duty);
				Bluetooth->Is_OK = 0;
		   }
		   else if((Bluetooth->Command.command_num) == 0x00)
				Bluetooth->Command.command_num += 0x80;	
		   else if((Bluetooth->Command.command_num) == 0x91){
				Bluetooth->Command.PID_P = 10*(Bluetooth->Command.PID_P)+0; 
		   }
		   else if((Bluetooth->Command.command_num) == 0x92){
				Bluetooth->Command.PID_I = 10*(Bluetooth->Command.PID_I)+0; 
		   }
		   else if((Bluetooth->Command.command_num) == 0x93){
				Bluetooth->Command.PID_D = 10*(Bluetooth->Command.PID_D)+0; 
		   }
		   break;
		case '1':
		   if((Bluetooth->Command.command_num) == 0xF0){
				Bluetooth->Command.command_num += 0x02;	//0xF2
		   		tmp_duty = MotorB->Duty;
		   		MotorB->Duty = 0;
				LPLD_FTM_PWM_ChangeDuty(MotorB->FTM_Ftmx, MotorB->chn, MotorB->Duty);
				Bluetooth->Is_OK = 1;
		   }
		   else if((Bluetooth->Command.command_num) == 0x80){
				Bluetooth->Command.command_num += 0x03; //0x83
		   		MotorB->Duty = tmp_duty;
				LPLD_FTM_PWM_ChangeDuty(MotorB->FTM_Ftmx, MotorB->chn, MotorB->Duty);
				Bluetooth->Is_OK = 1;
		   }
		   else if((Bluetooth->Command.command_num) == 0x91){
				Bluetooth->Command.PID_P = 10*(Bluetooth->Command.PID_P)+1; 
		   }
		   else if((Bluetooth->Command.command_num) == 0x92){
				Bluetooth->Command.PID_I = 10*(Bluetooth->Command.PID_I)+1; 
		   }
		   else if((Bluetooth->Command.command_num) == 0x93){
				Bluetooth->Command.PID_D = 10*(Bluetooth->Command.PID_D)+1; 
		   }
		   break;
		case '2':
		   if((Bluetooth->Command.command_num) == 0x80){
				Bluetooth->Command.command_num += 0x04;	//0x84
		   }
		   else if((Bluetooth->Command.command_num) == 0x91){
				Bluetooth->Command.PID_P = 10*(Bluetooth->Command.PID_P)+2; 
		   }
		   else if((Bluetooth->Command.command_num) == 0x92){
				Bluetooth->Command.PID_I = 10*(Bluetooth->Command.PID_I)+2; 
		   }
		   else if((Bluetooth->Command.command_num) == 0x93){
				Bluetooth->Command.PID_D = 10*(Bluetooth->Command.PID_D)+2; 
		   }
		   break;
		case '3':
		   if((Bluetooth->Command.command_num) == 0x80){
				Bluetooth->Command.command_num += 0x05;	//0x85
		   }
		   else if((Bluetooth->Command.command_num) == 0x91){
				Bluetooth->Command.PID_P = 10*(Bluetooth->Command.PID_P)+3; 
		   }
		   else if((Bluetooth->Command.command_num) == 0x92){
				Bluetooth->Command.PID_I = 10*(Bluetooth->Command.PID_I)+3; 
		   }
		   else if((Bluetooth->Command.command_num) == 0x93){
				Bluetooth->Command.PID_D = 10*(Bluetooth->Command.PID_D)+3; 
		   }
		   break;
	 	case '4':
		   if((Bluetooth->Command.command_num) == 0x91){
				Bluetooth->Command.PID_P = 10*(Bluetooth->Command.PID_P)+4; 
		   }
		   else if((Bluetooth->Command.command_num) == 0x92){
				Bluetooth->Command.PID_I = 10*(Bluetooth->Command.PID_I)+4; 
		   }
		   else if((Bluetooth->Command.command_num) == 0x93){
				Bluetooth->Command.PID_D = 10*(Bluetooth->Command.PID_D)+4; 
		   }
		   break;
		case '5':
		   if((Bluetooth->Command.command_num) == 0x91){
				Bluetooth->Command.PID_P = 10*(Bluetooth->Command.PID_P)+5; 
		   }
		   else if((Bluetooth->Command.command_num) == 0x92){
				Bluetooth->Command.PID_I = 10*(Bluetooth->Command.PID_I)+5; 
		   }
		   else if((Bluetooth->Command.command_num) == 0x93){
				Bluetooth->Command.PID_D = 10*(Bluetooth->Command.PID_D)+5; 
		   }
		   break;
		case '6':
		   if((Bluetooth->Command.command_num) == 0x91){
				Bluetooth->Command.PID_P = 10*(Bluetooth->Command.PID_P)+6; 
		   }
		   else if((Bluetooth->Command.command_num) == 0x92){
				Bluetooth->Command.PID_I = 10*(Bluetooth->Command.PID_I)+6; 
		   }
		   else if((Bluetooth->Command.command_num) == 0x93){
				Bluetooth->Command.PID_D = 10*(Bluetooth->Command.PID_D)+6; 
		   }
		   break;
		case '7':
		   if((Bluetooth->Command.command_num) == 0x91){
				Bluetooth->Command.PID_P = 10*(Bluetooth->Command.PID_P)+7; 
		   }
		   else if((Bluetooth->Command.command_num) == 0x92){
				Bluetooth->Command.PID_I = 10*(Bluetooth->Command.PID_I)+7; 
		   }
		   else if((Bluetooth->Command.command_num) == 0x93){
				Bluetooth->Command.PID_D = 10*(Bluetooth->Command.PID_D)+7; 
		   }
		   break;
		case '8':
		   if((Bluetooth->Command.command_num) == 0x91){
				Bluetooth->Command.PID_P = 10*(Bluetooth->Command.PID_P)+8; 
		   }
		   else if((Bluetooth->Command.command_num) == 0x92){
				Bluetooth->Command.PID_I = 10*(Bluetooth->Command.PID_I)+8; 
		   }
		   else if((Bluetooth->Command.command_num) == 0x93){
				Bluetooth->Command.PID_D = 10*(Bluetooth->Command.PID_D)+8; 
		   }
		   break;
		case '9':
		   if((Bluetooth->Command.command_num) == 0x00)
			   	Bluetooth->Command.command_num += 0xF0;
		   else if((Bluetooth->Command.command_num) == 0x91){
				Bluetooth->Command.PID_P = 10*(Bluetooth->Command.PID_P)+9; 
		   }
		   else if((Bluetooth->Command.command_num) == 0x92){
				Bluetooth->Command.PID_I = 10*(Bluetooth->Command.PID_I)+9; 
		   }
		   else if((Bluetooth->Command.command_num) == 0x93){
				Bluetooth->Command.PID_D = 10*(Bluetooth->Command.PID_D)+9; 
		   }
		   break;
	 	case 'P':
		   if((Bluetooth->Command.command_num) == 0x84)
				Bluetooth->Command.command_num = 0x91;
		   break;
	 	case 'I':
		   if((Bluetooth->Command.command_num) == 0x84)
				Bluetooth->Command.command_num = 0x92;
		   break;
	 	case 'D':
		   if((Bluetooth->Command.command_num) == 0x84)
				Bluetooth->Command.command_num = 0x93;
		   break;
	 	case '\x0d':
		   Bluetooth->Is_OK = (Bluetooth->Is_OK)?0:1;
		default: break;
	}
}

void BluetoothStart(void)
{
	LPLD_DMA_LoadDstAddr(DMA_CH1, Bluetooth->DMA_SourceAddr);
	LPLD_DMA_EnableReq(DMA_CH1);
}

void Bluetooth_DMA_isr(void)
{
	
}
//#pragma optimize=size
//void uart0_init(void)
//{
//	uart0_init_struct.UART_Uartx = UART0; //使用UART5
//	uart0_init_struct.UART_BaudRate = 115200; //设置波特率115200
//	uart0_init_struct.UART_RxPin = PTA15;  //接收引脚为PTE9
//	uart0_init_struct.UART_TxPin = PTA14;  //发送引脚为PTE8
//	uart0_init_struct.UART_RxIntEnable = TRUE;  //使能接收中断
//	uart0_init_struct.UART_RxIsr = uart0_isr;  //设置接收中断函数	
//	//初始化UART
//	LPLD_UART_Init(uart0_init_struct);
//	LPLD_UART_EnableIrq(uart0_init_struct);
//	LPLD_UART_PutCharArr(UART0, "Initial uart0 successful.\r\n", 29);
//}