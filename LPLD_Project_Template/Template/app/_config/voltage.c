#include "voltage.h"

myadc Adc0_0P1={&adc0_0P1_init, ADC0, &adc0_isr, DAD1, TRIGGER_PIT0, PIT0, \
				1000000, 0		//采样周期(us) 电压(mv)
				};
/*myadc Adc1_1P1={&adc1_1P1_init, ADC1, &adc1_isr, DAD1, TRIGGER_PIT1, PIT1, \
				1000000		//采样周期(us)
				};
*/
myadc*	Battery = &Adc0_0P1;
GPIO_InitTypeDef gpio_init_struct9;
ADC_InitTypeDef adc_init_struct1;
#pragma optimize=size
void adc0_0P1_init(void)
{	
	Battery_pdb_init();
	
	adc_init_struct1.ADC_Adcx = Battery->ADC_Adcx;
	adc_init_struct1.ADC_DiffMode = ADC_SE;        //单端采集
	adc_init_struct1.ADC_BitMode = SE_12BIT;       //单端12位精度
	adc_init_struct1.ADC_SampleTimeCfg = SAMTIME_SHORT;    //短采样时间
	adc_init_struct1.ADC_HwAvgSel = HW_4AVG;       //4次硬件平均
	adc_init_struct1.ADC_PgaGain = PGA_1GAIN;      //1倍增益
	adc_init_struct1.ADC_MuxSel = MUX_ADXXA;		//A输入通道
	adc_init_struct1.ADC_CalEnable = TRUE; //使能初始化校验
	adc_init_struct1.ADC_HwTrgCfg = HW_TRGA;   //禁用硬件触发转换
	adc_init_struct1.ADC_DmaEnable = FALSE;			//禁用DMA
	adc_init_struct1.ADC_Isr = Battery->isr;  //设置ADC中断函数

	LPLD_ADC_Init(adc_init_struct1);   //初始化ADC0
	LPLD_ADC_Chn_Enable(Battery->ADC_Adcx, Battery->chn);	//使能ADC0的DAD1输入引脚复用功能
	LPLD_ADC_EnableConversion(Battery->ADC_Adcx, Battery->chn, 0, TRUE);	//使能ADC0的DAD1输入通道及其转换完成中断
	LPLD_ADC_EnableIrq(adc_init_struct1);	//使能ADC0中断
	
	Battery_pit_init();
	gpio_init_struct9.GPIO_PTx = PTA;
	gpio_init_struct9.GPIO_Pins = GPIO_Pin25;
	gpio_init_struct9.GPIO_Dir = DIR_OUTPUT;
//	gpio_init_struct9.GPIO_Output = OUTPUT_L;
	gpio_init_struct9.GPIO_PinControl = IRQC_DIS;
	LPLD_GPIO_Init(gpio_init_struct9);
#if defined(DEBUG_PRINT)
	printf("Battery OK!\n");
#endif
}

PDB_InitTypeDef pdb_init_struct1;
#pragma optimize=size
void Battery_pdb_init(void)
{
	pdb_init_struct1.PDB_CounterPeriodUs = 10;   //PDB计数器周期设置
	pdb_init_struct1.PDB_LoadModeSel = LOADMODE_0; //加载模式设置
	pdb_init_struct1.PDB_DelayS = 10;    //中断延时时间2秒
	pdb_init_struct1.PDB_ContinuousModeEnable = FALSE;     //禁用连续工作模式
	pdb_init_struct1.PDB_TriggerInputSourceSel = Battery->PDB_TriggerInputSourceSel;     //配置触发源为PIT0
	LPLD_PDB_Init(pdb_init_struct1);
	LPLD_PDB_AdcTriggerCfg(Battery->ADC_Adcx, PRETRIG_EN_A, 0);
}

PIT_InitTypeDef pit_init_struct1;
#pragma optimize=size
void Battery_pit_init(void)
{
	pit_init_struct1.PIT_Pitx = Battery->PIT_Pitx;     //选择PIT0
	pit_init_struct1.PIT_PeriodUs = Battery->PIT_PeriodUs;  //PIT0计数周期500ms
	LPLD_PIT_Init(pit_init_struct1);  
}

#pragma optimize=speed
void adc0_isr(void)
{
	//uint8 ab = LPLD_ADC_GetSC1nCOCO(ADC0);//判断转换完成的是A组还是B组
	Battery->Vol = (uint16)(1000*4*LPLD_ADC_GetResult(ADC0, 0)*3.3/4095);//获取采样结果
#if defined(DEBUG_PRINT)
//	printf("ADC0_R[A]=%d mV.\n", Battery->Vol);
//        if(Battery->Vol<=6800){
//          PTA25_O = 1;
//        }
//        else{
//          PTA25_O = 0;
//        }
#endif
	LPLD_ADC_EnableConversion(ADC0, DAD1, 0, TRUE);
}
