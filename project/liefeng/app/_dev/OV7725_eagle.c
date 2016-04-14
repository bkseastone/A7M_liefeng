#include "OV7725_eagle.h"
#include "DEV_SCCB.h"

/****************************************
说明：
*将OV7725上位机波特率设置为115200
*使用上位机查看运行结果
****************************************/
extern uint8 Is_DispPhoto;               //图像发送标志

void ov7725_dma_start(void)
{
	//V_Cnt=0;          //行计数
	Is_DispPhoto=0;   //显示图像
	ov7725_init(); //初始配置OV7725 (by SCCB protocol)
	OV_dma_init();
	disable_irq(PORTE_IRQn);
	disable_irq(PORTA_IRQn);
	OV_gpio_init();
	OV_delay();
	
	EnableInterrupts;//使能全局中断
	enable_irq(PORTE_IRQn);//使能Pclk中断、场中断
	enable_irq(PORTA_IRQn);
}

/*OV7725初始化配置表*/
reg_s ov7725_eagle_reg[] =
{
    //寄存器，寄存器值次
    {OV7725_COM4         , 0xC1},
    {OV7725_CLKRC        , 0x00},
    {OV7725_COM2         , 0x03},
    {OV7725_COM3         , 0xD0},
    {OV7725_COM7         , 0x40},
    {OV7725_HSTART       , 0x3F},
    {OV7725_HSIZE        , 0x50},
    {OV7725_VSTRT        , 0x03},
    {OV7725_VSIZE        , 0x78},
    {OV7725_HREF         , 0x00},
    {OV7725_SCAL0        , 0x0A},
    {OV7725_AWB_Ctrl0    , 0xE0},
    {OV7725_DSPAuto      , 0xff},
    {OV7725_DSP_Ctrl2    , 0x0C},
    {OV7725_DSP_Ctrl3    , 0x00},
    {OV7725_DSP_Ctrl4    , 0x00},

#if (CAMERA_W == 80)
    {OV7725_HOutSize     , 0x14},
#elif (CAMERA_W == 160)
    {OV7725_HOutSize     , 0x28},
#elif (CAMERA_W == 240)
    {OV7725_HOutSize     , 0x3c},
#elif (CAMERA_W == 320)
    {OV7725_HOutSize     , 0x50},
#else

#endif

#if (CAMERA_H == 60 )
    {OV7725_VOutSize     , 0x1E},
#elif (CAMERA_H == 120 )
    {OV7725_VOutSize     , 0x3c},
#elif (CAMERA_H == 180 )
    {OV7725_VOutSize     , 0x5a},
#elif (CAMERA_H == 240 )
    {OV7725_VOutSize     , 0x78},
#else

#endif

    {OV7725_EXHCH        , 0x00},
    {OV7725_GAM1         , 0x0c},
    {OV7725_GAM2         , 0x16},
    {OV7725_GAM3         , 0x2a},
    {OV7725_GAM4         , 0x4e},
    {OV7725_GAM5         , 0x61},
    {OV7725_GAM6         , 0x6f},
    {OV7725_GAM7         , 0x7b},
    {OV7725_GAM8         , 0x86},
    {OV7725_GAM9         , 0x8e},
    {OV7725_GAM10        , 0x97},
    {OV7725_GAM11        , 0xa4},
    {OV7725_GAM12        , 0xaf},
    {OV7725_GAM13        , 0xc5},
    {OV7725_GAM14        , 0xd7},
    {OV7725_GAM15        , 0xe8},
    {OV7725_SLOP         , 0x20},
    {OV7725_LC_RADI      , 0x00},
    {OV7725_LC_COEF      , 0x13},
    {OV7725_LC_XC        , 0x08},
    {OV7725_LC_COEFB     , 0x14},
    {OV7725_LC_COEFR     , 0x17},
    {OV7725_LC_CTR       , 0x05},
    {OV7725_BDBase       , 0x99},
    {OV7725_BDMStep      , 0x03},
    {OV7725_SDE          , 0x04},
    {OV7725_BRIGHT       , 0x00},
    {OV7725_CNST         , 0xFF},
    {OV7725_SIGN         , 0x06},
    {OV7725_UVADJ0       , 0x11},
    {OV7725_UVADJ1       , 0x02},

};

uint8 ov7725_eagle_cfgnum = ARR_SIZE( ov7725_eagle_reg ) ; /*结构体数组成员数目*/
void ov7725_init(void)
{
	uint16 i = 0;
	uint8 Sensor_IDCode = 0;
	LPLD_SCCB_Init();
	while( 0 == LPLD_SCCB_WriteReg(OV7725_COM7, 0x80) ); //复位sensor
	OV_delay();
	LPLD_SCCB_ReadReg(OV7725_VER, &Sensor_IDCode, 1); // 读取sensor ID号
	printf("\nGet ID success, SENSOR ID is 0x%x", Sensor_IDCode);
	printf("\nConfig Register Number is %d ", ov7725_eagle_cfgnum);
	while(Sensor_IDCode != OV7725_ID);
	for( i = 0 ; i < ov7725_eagle_cfgnum ; i++ )
	{
		if( 0 == LPLD_SCCB_WriteReg(ov7725_eagle_reg[i].addr, ov7725_eagle_reg[i].val) )
		{
			printf("\n警告:写寄存器0x%x失败\n", ov7725_eagle_reg[i].addr);
			while(1)
			{;}
		}
	}
	printf("\nOV7725 Register Config Success!\n");
}

DMA_InitTypeDef OV_dma_init_struct;
void OV_dma_init(void)
{
	//DMA参数配置
	OV_dma_init_struct.DMA_CHx = DMA_CH0;    //CH0通道
	OV_dma_init_struct.DMA_Req = PORTE_DMAREQ;       //PORTE为请求源
	OV_dma_init_struct.DMA_MajorLoopCnt = CAMERA_W*CAMERA_H/8; //主循环计数值：行采集点数，宽度
	OV_dma_init_struct.DMA_MinorByteCnt = 1; //次循环字节计数：每次读入1字节
	OV_dma_init_struct.DMA_SourceAddr = (uint32)&PTD->PDIR+1;        //源地址：PTD8~15  0x400FF0D1u
	OV_dma_init_struct.DMA_DestAddr = OV_binary_ADDR;      //目的地址：存放图像的数组
	OV_dma_init_struct.DMA_DestAddrOffset = 1;       //目的地址偏移：每次读入增加1
	OV_dma_init_struct.DMA_AutoDisableReq = TRUE;    //自动禁用请求
	OV_dma_init_struct.DMA_MajorCompleteIntEnable = TRUE;
	OV_dma_init_struct.DMA_Isr = OV_DMA_isr;
	//初始化DMA
	LPLD_DMA_Init(OV_dma_init_struct);
	LPLD_DMA_EnableIrq(OV_dma_init_struct);
}

GPIO_InitTypeDef OV_pta_init;
//GPIO_InitTypeDef OV_ptb_init;
GPIO_InitTypeDef OV_pte_init;
GPIO_InitTypeDef OV_ptd_init;
void OV_gpio_init(void)
{
	//OV数据口初始化：PTD8~PTD15
	OV_ptd_init.GPIO_PTx = PTD;
	OV_ptd_init.GPIO_Dir = DIR_INPUT;
	OV_ptd_init.GPIO_Pins = GPIO_Pin8_15; 
	OV_ptd_init.GPIO_PinControl = IRQC_DIS | INPUT_PULL_DIS;  
	LPLD_GPIO_Init(OV_ptd_init); 
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
	OV_pte_init.GPIO_PinControl = IRQC_DMARI | INPUT_PULL_DOWN; 
	LPLD_GPIO_Init(OV_pte_init); 
}



void OV_porta_Visr(void)
{
	if(LPLD_GPIO_IsPinxExt(PORTA, GPIO_Pin5))
	{
		//检测到场开始信号，加载目的地址
		LPLD_DMA_LoadDstAddr(DMA_CH0, OV_binary_ADDR);
		LPLD_DMA_EnableReq(DMA_CH0);
		disable_irq(PORTA_IRQn);
		//printf("porta_Visr");
	}
}

void OV_DMA_isr(void)
{
	Is_DispPhoto = 1;//可以显示图像
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
	LPLD_UART_PutChar(UART4, 0x01); //上位机命令字
	LPLD_UART_PutChar(UART4, 0xFE);
	for(row=0;row<=CAMERA_H-1;row++)
	{
		for(col=0;col<=CAMERA_W-1;col++)
		{
			LPLD_UART_PutChar(UART4, OV_binary_data(row,col));
		}
	}
	LPLD_UART_PutChar(UART4, 0xFE);
	LPLD_UART_PutChar(UART4, 0x01);
	LPLD_GPIO_ClearIntFlag(PORTA);//清PORTA中断标志
	enable_irq(PORTA_IRQn);//使能PORTA中断
}
