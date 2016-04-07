#include "ADCConfig.h"
#include "interrupt.h"
#include "time.h"

ADC_InitTypeDef adc_init_struct;
ADCstatus adc_init(myadc *adc, uint32 PIT_PeriodUs)
{
	if( pdb_init(adc->PDB_TriggerInputSourceSel, adc->ADC_Adcx)==0 )
	{
		return error_init_pdb;
	}
	
	adc_init_struct.ADC_Adcx = adc->ADC_Adcx;
	adc_init_struct.ADC_DiffMode = ADC_SE;        //单端采集
	adc_init_struct.ADC_BitMode = SE_12BIT;       //单端12位精度
	adc_init_struct.ADC_SampleTimeCfg = SAMTIME_SHORT;    //短采样时间
	adc_init_struct.ADC_HwAvgSel = HW_4AVG;       //4次硬件平均
	adc_init_struct.ADC_PgaGain = PGA_1GAIN;      //1倍增益
	adc_init_struct.ADC_MuxSel = MUX_ADXXA;		//A输入通道
	adc_init_struct.ADC_CalEnable = TRUE; //使能初始化校验
	adc_init_struct.ADC_HwTrgCfg = HW_TRGA;   //禁用硬件触发转换
	adc_init_struct.ADC_DmaEnable = FALSE;			//禁用DMA
	adc_init_struct.ADC_Isr = adc->isr;  //设置ADC中断函数

	if( LPLD_ADC_Init(adc_init_struct)==0 || \
		LPLD_ADC_Chn_Enable(adc->ADC_Adcx, adc->chn)==0 || \
		LPLD_ADC_EnableIrq(adc_init_struct)==0 )	
	{
		return error_init_adc;
	}
	LPLD_ADC_EnableConversion(adc->ADC_Adcx, adc->chn, 0, TRUE);
	
	if( pit_init(adc->PIT_Pitx, PIT_PeriodUs)==0 )
	{
		return error_init_pit;
	}
	else
	{
		return OK;
	}
}
