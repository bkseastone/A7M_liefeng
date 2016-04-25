- Pin configuration

|Function   |Pins                                                             |
|:--------- |:--------------------------------------------------------------- |
|编码器		|pwm2_A10 pwm2_A11                                            |
|舵机		|pwm0_A4                                                      |
|电机		|pwm1_A8 pwm1_A9 EN_A7                                        |
|摄像头		|D0~D7(Y7~Y0) E28(HREF）E27(VSYNC）A6(PCLK) B0(SCL) B1(SDA)   |
|电量采集	|adc0_0P1(pin 23) adc1_1P1(pin 25)                            |
|SD卡		|E0~E5 (SDHC.D1/SDHC.D0/SDHC.CLK/SDHC.CMD/SDHC.D3/SDHC.D2)    |
|蓝牙		|E24(Rx) E25(Tx)                                              |
|串口		|A14(Rx) A15(Tx)                                              |
|蜂鸣器		|A25                                                          |
|拨码开关	|PTB4~PTB11                                                   |
|LED		|D8~D11                                                       |
|EEROM/MPU6050	|C10 C11 C8                                                   |
|独立按键	|C3 C4 B23 B19                                                |
|扩展           |D12 D13 B2 B3 ADC0-DP3                                       |
|*OLED(SPI)*|*用龙邱核心板自己提供的接口C16--C19*                             |

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

