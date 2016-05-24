/*
     ___   _____       ___  ___        _       _   _____   _____   _____   __   _   _____
    /   | |___  |     /   |/   |      | |     | | | ____| |  ___| | ____| |  \ | | /  ___|
   / /| |     / /    / /|   /| |      | |     | | | |__   | |__   | |__   |   \| | | |
  / / | |    / /    / / |__/ | |      | |     | | |  __|  |  __|  |  __|  | |\   | | |  _
 / /  | |   / /    / /       | |      | |___  | | | |___  | |     | |___  | | \  | | |_| |
/_/   |_|  /_/    /_/        |_|      |_____| |_| |_____| |_|     |_____| |_|  \_| \_____/

*
*/
#include "main.h"
//extern myic				*Encoder_Feedback;
extern myic					*Encoder_Test;
//extern MotorTypeDef		*MotorF;
extern MotorTypeDef			*MotorB;
extern OvTypeDef			*Ov7725;
extern ServomotorTypeDef	*ServoMotor;
//extern myadc				*Battery;
extern SystemTypeDef		*Sys;
extern BluetoothTypeDef 	*Bluetooth;
extern ServoPIDTypeDef		*Servo_PID;

void dosomething(void)
{
	Bluetooth->Command.PID_D = (uint32)(Servo_PID->Td*1000);
	Bluetooth->Command.PID_I = (uint32)(Servo_PID->Ti*1000);
	Bluetooth->Command.PID_P = (uint32)(Servo_PID->Kp*1000);
}
void main (void)
{
	Sys->init();
	Bluetooth->init();
	Ov7725->init();
//	ServoMotor->init();
//	MotorB->init();
//	Encoder_Test->init();
//	dosomething();
	while(1)
	{
		if(Sys->PULSE == 1)
		{
			Sys->PULSE = 0;
			Sys->RunTime += (Sys->PeriodUs)/1000000;
			if(Ov7725->Is_DispPhoto){
				Ov7725->calculate();
				#if defined(DEBUG_PRINT)
					/*�ַ�*/
//					printf("\n");
//					printf("*******************************\n");
//					printf("escape_position = %d\n", Ov7725->pic.escape_position);
//					printf("L: %d--->%d\n", Ov7725->pic.start_L, Ov7725->pic.end_L);
//					printf("R: %d--->%d\n", Ov7725->pic.start_R, Ov7725->pic.end_R);
//					printf("L: (%d,%d)#####K=%f\n", Ov7725->calparam.datum_pointY_L, Ov7725->calparam.datum_pointX_L, Ov7725->calparam.border_slope_L);
//					printf("R: (%d,%d)#####K=%f\n", Ov7725->calparam.datum_pointY_R, Ov7725->calparam.datum_pointX_R, Ov7725->calparam.border_slope_R);
//					printf("last_point_L = %d#####last_point_R = %d\n", (Ov7725->calparam.last_point_L), (Ov7725->calparam.last_point_R));
//					printf("*******************************\n");
//					printf("Location_bias = %d\nDeflection = %d\n", (Ov7725->pos.location_bias), (Ov7725->pos.deflection));
//					printf("\n");
					/*ͼƬ*/
//					Ov7725->display();
				#endif
				Ov7725->restart();
			}
//			Servo_PID->nowe = (0 - (Ov7725->gain*Ov7725->pos.deflection + 0.2*Ov7725->pos.location_bias));
//			if(Ov7725->GOODSTATUS){ //������ʻʱ��
//				ServoMotor->Deflection = Servo_PID->cal();
//			}
//			else if(Ov7725->mode == 1){ //Σ��ת��ʱ��
//				ServoMotor->Deflection = (Ov7725->pic.exit_R-Ov7725->pic.exit_L)*90;
//				//���PID���ε�����
//			}
//			else{ //�޷�����ʱ��
//				continue;
//			}


			ServoMotor->Deflection = (int32)(0 - (Ov7725->gain*Ov7725->pos.deflection + 0.2*Ov7725->pos.location_bias));
			LPLD_FTM_PWM_ChangeDuty(ServoMotor->FTM_Ftmx, ServoMotor->chn, angle_to_period(ServoMotor->Deflection));
			if(Bluetooth->Is_OK){
				Servo_PID->Td = (float)(Bluetooth->Command.PID_D)/1000.0;
				Servo_PID->Ti = (float)(Bluetooth->Command.PID_I)/1000.0;
				Servo_PID->Kp = (float)(Bluetooth->Command.PID_P)/1000.0;
			}
		}
	}
}

/**@��ʼ������
	init_gpio_led_on();
	init_gpio_monitorC4();//�����ж�
	Ov7725->init(); //����ͷ
	MotorB->init(); //���
	Encoder_Test->init(); //������
	Battery->init(); //���
	ServoMotor->init(); //���
	LPLD_FTM_PWM_ChangeDuty(ServoMotor->FTM_Ftmx, ServoMotor->chn, angle_to_period(15)); //���
*/

/**@��DMA
	char num[2] = {5,2};

	Bluetooth->DMA_SourceAddr = (uint32)(&num);
	Bluetooth->DMA_ByteCnt = 2;
	Bluetooth->init();
	Bluetooth->transmit();
	while(1){
	}
*/