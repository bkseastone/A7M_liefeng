- Pin configuration

|Function   |Pins                                                             |
|:--------- |:--------------------------------------------------------------- |
|编码器		|pwm2_B18 pwm3_D0                                                 |
|舵机		|pwm0_C1                                                          |
|电机		|Pwm1_A8 Pwm1_A9                                                  |
|摄像头		|PTD8~PTD15(Y0~Y7) PTB3(H) PTA5(V) PTE6(PCLK) PTB0(SCL) PTB1(SDA) |
|电量采集	|Adc0_0P1(23号) Adc1_1P1(25号)                                    |
|SD卡		|PTE0~PTE5 (SDHC.D1/SDHC.D0/SDHC.CLK/SDHC.CMD/SDHC.D3/SDHC.D2)    |
|串口		|PTE24(Rx) PTE25(Tx)                                              |
|拨码开关	|PTB4~PTB11                                                       |
|LED		|PTD4~PTD7                                                        |
|EEROM(IIC)	|PTC10 PTC11                                                      |
|独立按键	|PTC4~PTC7                                                        |
|*OLED(SPI)*|用龙邱核心板自己提供的接口                                       |

- About MCU

Device		  |Programflash(KB)	  |FlexNVM(KB)	  |FlexRAM(KB)	  |SRAM(KB)
--------------|-------------------|---------------|---------------|--------
MK60FX512VLQ15|512 		    	  |512    		  |16 		      |128

- SRAM configuration

|Address                         |Function                          |Size   |
|:------------------------------ |:-------------------------------- |:----- |
|0x1FFF0410 ~ 0x20000000         |readwrite, block CodeRelocateRam  |62.98KB|
|0x20000000 ~ 0x20008F72         |block CSTACK, block HEAP          |32.86KB|
|0x20008F7A ~ 0x20010000         |binary-format picture             |28.13KB|

