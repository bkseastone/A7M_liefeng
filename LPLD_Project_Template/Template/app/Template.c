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
#include "math.h"
extern int16                RectifyX[60][81];
extern int16                RectifyY[60][81];
extern int16                sudu;
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
extern motorPIDTypeDef		*motor_PID;
extern WeizhiPIDTypeDef	*Weizhi_PID;

//extern int16                    a;   //�����������



void dosomething(void)
{
	Bluetooth->Command.PID_D = (uint32)(Servo_PID->Td*1000);
	Bluetooth->Command.PID_I = (uint32)(Servo_PID->Ti*1000);
	Bluetooth->Command.PID_P = (uint32)(Servo_PID->Kp*1000);
}
void main (void)
{
//        int i;
//        Battery->init();
//	Bluetooth->init();
	Ov7725->init();
	ServoMotor->init();
	MotorB->init();
	Encoder_Test->init();
//	dosomething();
	Sys->init();

////      ���Բ��������ŵĵ���
//        a=InvSqrt(0.023)*1000;
//        printf("L: %d\n", a);

	while(1)
	{
////              LPLD_GPIO_Output_b(PTA, 25, 1);
		if(Ov7725->Is_DispPhoto){
//			Sys->timer.start();
			Ov7725->calculate();
//			Sys->timer.end();
//			printf("calculate waste %d\n", (int32)(Sys->timer.runtime));
//			#if defined(DEBUG_PRINT)
			/*�ַ�*/
//////				printf("\n");
//////				printf("*******************************\n");
//////				printf("escape_position = %d\n", Ov7725->pic.escape_position);
//////				printf("L: (%d,%d)#####K=%f\n", Ov7725->calparam.datum_pointY_L, Ov7725->calparam.datum_pointX_L, Ov7725->calparam.border_slope_L);
//////				printf("R: (%d,%d)#####K=%f\n", Ov7725->calparam.datum_pointY_R, Ov7725->calparam.datum_pointX_R, Ov7725->calparam.border_slope_R);
//////				printf("last_point_L = %d#####last_point_R = %d\n", (Ov7725->calparam.last_point_L), (Ov7725->calparam.last_point_R));
//////				printf("start(%d,%d) end(%d,%d)\n", Ov7725->pic.border_pos_R[Ov7725->pic.end_R], Ov7725->pic.end_R, Ov7725->pic.border_pos_R[Ov7725->pic.start_R], Ov7725->pic.start_R);
//////
//////                      ���Է��任��
//                        printf("start\n");
//                        for (i=0;i<=59;i++)
//                        {
//////                        RectifyZ[i][1]=RectifyX[i][Ov7725->pic.border_pos_L[i]];
//////                        RectifyZ[i][2]=RectifyY[i][Ov7725->pic.border_pos_L[i]];
//////                        RectifyY[i][1]=RectifyX[i][Ov7725->pic.border_pos_R[i]];
//////                        RectifyY[i][2]=RectifyY[i][Ov7725->pic.border_pos_R[i]];
//                        printf("%d %d %d %d %d %d %d\n",RectifyX[i][Ov7725->pic.border_pos_L[i]],RectifyY[i][Ov7725->pic.border_pos_L[i]],RectifyX[i][Ov7725->pic.border_pos_R[i]],RectifyY[i][Ov7725->pic.border_pos_R[i]],Ov7725->pic.border_pos_L[i],Ov7725->pic.border_pos_R[i],i);
//////                        printf("%d %d %d %d\n",RectifyX[i][Ov7725->pic.border_pos_L[i]],RectifyY[i][Ov7725->pic.border_pos_L[i]],RectifyX[i][Ov7725->pic.border_pos_R[i]],RectifyY[i][Ov7725->pic.border_pos_R[i]]);
//                        LPLD_LPTMR_DelayMs(100);
//                        printf("%d %d\n",Ov7725->pic.border_pos_L[i],Ov7725->pic.border_pos_R[i]);
//                        }
//////                       ���Խ�����matlab�鿴
//////				printf("*******************************\n");
//				printf("L: %d--->%d\n", Ov7725->pic.start_L, Ov7725->pic.end_L);
//				printf("R: %d--->%d\n", Ov7725->pic.start_R, Ov7725->pic.end_R);
//				printf("L: %d	R: %d\n", Ov7725->pic.exit_L, Ov7725->pic.exit_R);
//				printf("Location_bias = %d\nDeflection = %d\n", (Ov7725->pos.location_bias), (Ov7725->pos.deflection));
//				printf("GOODSTATUS %d\n mode %d\n", Ov7725->GOODSTATUS, Ov7725->mode);
				/*����*/
//				printf("Ov7725-->deflection = %d\n", Ov7725->pos.deflection);
                        throttle_control();
//              printf("distance %d\n", Ov7725->distance);
//              printf("%d,%d\n",MotorB->Velosity,MotorB->Target_Velosity);
//                OV_delay();
//				printf("\n");
				/*ͼƬ*/
//				Ov7725->display();
//			#endif
			Ov7725->restart();
		}
//               printf("%d,%d\n",sudu,MotorB->Velosity);
//               OV_delay();
//		LPLD_LPTMR_DelayMs(300);
////		Servo_PID->nowe = (0 - (((Ov7725->pos.deflection>0)?1:(-1))*GAIN_deflection*Ov7725->pos.deflection*Ov7725->pos.deflection + \
////				(((Ov7725->pos.location_bias)>0)?1:(-1))*GAIN_location_bias*Ov7725->pos.location_bias*Ov7725->pos.location_bias))+Ov7725->pos.deflection;
////                ServoMotor->Deflection = Servo_PID->cal();
////                printf("PID���: %d �� %d\n", Servo_PID->out,Servo_PID->nowe);
////////                OV_delay();
//////                LPLD_FTM_PWM_ChangeDuty(ServoMotor->FTM_Ftmx, ServoMotor->chn, angle_to_period(Servo_PID->nowe));
//////
//		if((Ov7725->GOODSTATUS == 1)&&(Ov7725->mode == 0)){ //ֱ��
//			Weizhi_PID->nowe = (int32)(0 - (((Ov7725->pos.deflection>0)?1:(-1))*GAIN_deflection*Ov7725->pos.deflection*Ov7725->pos.deflection + \
//				(((Ov7725->pos.location_bias)>0)?1:(-1))*GAIN_location_bias*Ov7725->pos.location_bias*Ov7725->pos.location_bias));
//		}
//		else if((Ov7725->GOODSTATUS == 0)&&(Ov7725->mode == 1)){ //���
//			Weizhi_PID->nowe = (int32)(0 - (Ov7725->pos.deflection + \
//				(((Ov7725->pos.location_bias)>0)?1:(-1))*GAIN_location_bias*Ov7725->pos.location_bias*Ov7725->pos.location_bias));
//		}
//		else if((Ov7725->GOODSTATUS == 1)&&(Ov7725->mode == 0)){ //������
//			//����
//			Weizhi_PID->nowe = (int32)(0 - (((Ov7725->pos.deflection>0)?1:(-1))*GAIN_deflection*Ov7725->pos.deflection*Ov7725->pos.deflection + \
//				(((Ov7725->pos.location_bias)>0)?1:(-1))*GAIN_location_bias*Ov7725->pos.location_bias*Ov7725->pos.location_bias));
//		}
//		else{ //�޷�����ʱ��(�����߶�������)
//			continue;
//		}
//		printf("ServoMotor-->deflection = %d\n", ServoMotor->Deflection);
//		OV_delay();
//////
////////		ServoMotor->Deflection = (0 - (((Ov7725->pos.deflection>0)?1:(-1))*GAIN_deflection*Ov7725->pos.deflection*Ov7725->pos.deflection + \
////////				(((Ov7725->pos.location_bias)>0)?1:(-1))*GAIN_location_bias*Ov7725->pos.location_bias*Ov7725->pos.location_bias));
////////                printf("����Ƕ�: %d  ���ƫ��: %d\n", ServoMotor->Deflection,Ov7725->pos.deflection);
//
//		LPLD_FTM_PWM_ChangeDuty(ServoMotor->FTM_Ftmx, ServoMotor->chn, angle_to_period(Weizhi_PID->cal()));
//
		
//		LPLD_FTM_PWM_ChangeDuty(MotorB->FTM_Ftmx, MotorB->chn, MotorB->Duty);
                
//
//		if(Bluetooth->Is_OK){
//			Servo_PID->Td = (float)(Bluetooth->Command.PID_D)/1000.0;
//			Servo_PID->Ti = (float)(Bluetooth->Command.PID_I)/1000.0;
//			Servo_PID->Kp = (float)(Bluetooth->Command.PID_P)/1000.0;
//		}
//		while(Bluetooth->Command.STOP)
//		{	;	}
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