#include "OV7620.h"
#include "DEV_SCCB.h"

/****************************************
说明：
*将OV7620上位机波特率设置为115200
*使用上位机查看运行结果
****************************************/
extern uint8 Pix_Data[PHOTO_SIZE];       //采集320行 240列的图像数据  
uint32 V_Cnt;                     //行采集计数
extern uint8 Is_DispPhoto;               //图像发送标志

void ov7620_dma_start(void)
{
	V_Cnt=0;          //行计数
	Is_DispPhoto=0;   //显示图像
	OV_gpio_init();
	while(!ov7620_init()); //初始配置OV7620 (by SCCB protocol)
	OV_dma_init();
	OV_delay();
	
	EnableInterrupts;//使能全局中断
	enable_irq(PORTB_IRQn);//使能行中断、场中断
	enable_irq(PORTA_IRQn);
}

GPIO_InitTypeDef OV_pta_init;
GPIO_InitTypeDef OV_ptb_init;
GPIO_InitTypeDef OV_pte_init;
GPIO_InitTypeDef OV_ptd_init;
void OV_gpio_init(void)
{
	//OV数据口初始化：PTD8~PTD15
	OV_ptd_init.GPIO_PTx = PTD;
	OV_ptd_init.GPIO_Dir = DIR_INPUT;
	OV_ptd_init.GPIO_Pins = GPIO_Pin8_15; 
	OV_ptd_init.GPIO_PinControl = IRQC_DIS | INPUT_PULL_DIS;  //     INPUT_PULL_DOWN
	LPLD_GPIO_Init(OV_ptd_init);
	//OV行信号接口初始化：PTB3-H
	OV_ptb_init.GPIO_PTx = PTB;
	OV_ptb_init.GPIO_Dir = DIR_INPUT;
	OV_ptb_init.GPIO_Pins = GPIO_Pin3;
	OV_ptb_init.GPIO_PinControl = IRQC_RI|INPUT_PULL_DOWN;
	OV_ptb_init.GPIO_Isr = OV_portb_Hisr;
	LPLD_GPIO_Init(OV_ptb_init); 
	//OV场信号接口初始化：PTA5-V
	OV_pta_init.GPIO_PTx = PTA;
	OV_pta_init.GPIO_Dir = DIR_INPUT;
	OV_pta_init.GPIO_Pins = GPIO_Pin5;
	OV_pta_init.GPIO_PinControl = IRQC_RI|INPUT_PULL_DOWN;
	OV_pta_init.GPIO_Isr = OV_porta_Visr;
	LPLD_GPIO_Init(OV_pta_init); 
	//OV PCLK信号接口初始化：PTE6-PCLK
	OV_pte_init.GPIO_PTx = PTE;
	OV_pte_init.GPIO_Pins = GPIO_Pin6;
	OV_pte_init.GPIO_Dir = DIR_INPUT;
	OV_pte_init.GPIO_PinControl = IRQC_DMARI | INPUT_PULL_DIS;  //   INPUT_PULL_DIS
	LPLD_GPIO_Init(OV_pte_init); 
}

DMA_InitTypeDef OV_dma_init_struct;
void OV_dma_init(void)
{
	//DMA参数配置
	OV_dma_init_struct.DMA_CHx = DMA_CH0;    //CH0通道
	OV_dma_init_struct.DMA_Req = PORTE_DMAREQ;       //PORTE为请求源
	OV_dma_init_struct.DMA_MajorLoopCnt = H; //主循环计数值：行采集点数，宽度
	OV_dma_init_struct.DMA_MinorByteCnt = 1; //次循环字节计数：每次读入1字节
	OV_dma_init_struct.DMA_SourceAddr = (uint32)&PTD->PDIR+1;        //源地址：PTD8~15  0x400FF0D1u
	OV_dma_init_struct.DMA_DestAddr = (uint32)Pix_Data;      //目的地址：存放图像的数组
	OV_dma_init_struct.DMA_DestAddrOffset = 1;       //目的地址偏移：每次读入增加1
	OV_dma_init_struct.DMA_AutoDisableReq = TRUE;    //自动禁用请求
	//初始化DMA
	LPLD_DMA_Init(OV_dma_init_struct);
}

void OV_porta_Visr(void)
{
	if(LPLD_GPIO_IsPinxExt(PORTA, GPIO_Pin5))
	{
		//检测到场开始信号，加载目的地址
		LPLD_DMA_LoadDstAddr(DMA_CH0, Pix_Data);
		//清行中断标志，防止进入无效行中断
		LPLD_GPIO_ClearIntFlag(PORTB);
		enable_irq(PORTB_IRQn);
		//printf("porta_Visr");
	}
  
}

void OV_portb_Hisr(void)
{
	if(LPLD_GPIO_IsPinxExt(PORTB, GPIO_Pin3))
	{
		//检测到行开始信号，使能DMA请求
		if(V_Cnt<V){
			LPLD_DMA_EnableReq(DMA_CH0);  
			V_Cnt++; 
		}
		//行数采集已满，关闭中断
		else
		{
			//关GPIO中断 
			disable_irq(PORTA_IRQn);
			disable_irq(PORTB_IRQn);
			Is_DispPhoto = 1;//可以显示图像
			V_Cnt=0;  
		}
	}
}

uint8 ov7620_init(void)
{
	LPLD_SCCB_Init();
	OV_delay();
	while( 0 == LPLD_SCCB_WriteReg(0x11, 0x00) );
	while( 0 == LPLD_SCCB_WriteReg(0x14, 0x24) );
	while( 0 == LPLD_SCCB_WriteReg(0x28, 0x20) );
	OV_delay();
	return 1;
}

/*
 * 延时一段时间
 */
void OV_delay()
{
  uint16 i, n;
  for(i=0;i<30000;i++)
  {
    for(n=0;n<200;n++)
    {
      asm("nop");
    }
  }
}

void OV_display(void)
{
	uint16 row, col;
	Is_DispPhoto = 0;
	printf("%c%c%c%c", 0, 255, 1, 0); //上位机命令字
	for(row=0;row<=V-1;row++)
	{
		for(col=0;col<=H-1;col++)
		{
			LPLD_UART_PutChar(UART5, Pix_Data[row*H + col]);
		}
	}
	LPLD_GPIO_ClearIntFlag(PORTA);//清PORTA中断标志
	enable_irq(PORTA_IRQn);//使能PORTA中断
}
