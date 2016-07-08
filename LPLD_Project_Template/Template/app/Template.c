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
extern myadc				*Battery;
extern SystemTypeDef		*Sys;
extern BluetoothTypeDef 	*Bluetooth;
extern ServoPIDTypeDef		*Servo_PID;
extern motorPIDTypeDef		*motor_PID;
extern WeizhiPIDTypeDef		*Weizhi_PID;
extern int                  Sflag;
extern OV_pictureTypeDef_SRAM	OV_pictures_SRAM @(OV_binary_ADDR+2);
void main (void)
{
//	Battery->init();
//	Bluetooth->init();
	Ov7725->init();
//	LPLD_LPTMR_DelayMs(1000);//调试用
	ServoMotor->init();
	MotorB->init();
	Encoder_Test->init();
	Sys->init();
	init_photocellB2B3();
	while(1)
	{
		if(Ov7725->Is_DispPhoto){
			/*运行时长测试*/
//			Sys->timer.start();
			Ov7725->calculate();
//			Sys->timer.end();
//			printf("calculate waste %d\n", (int32)(Sys->timer.runtime));

			Weizhi_PID->nowe = 0 - (Ov7725->pos.deflection + Ov7725->pos.location_bias);
			ServoMotor->Deflection = Weizhi_PID->cal();
			LPLD_FTM_PWM_ChangeDuty(ServoMotor->FTM_Ftmx, ServoMotor->chn, angle_to_period(ServoMotor->Deflection));
//			#if defined(DEBUG_PRINT)
//				printf("L(H): %d--->%d\n", Ov7725->pic.start_L, Ov7725->pic.end_L);
//				printf("R(H): %d--->%d\n", Ov7725->pic.start_R, Ov7725->pic.end_R);
//				printf("L: %d	R: %d\n", Ov7725->pic.exit_L, Ov7725->pic.exit_R);
//				printf("Location_bias = %d\nDeflection = %d\n", (Ov7725->pos.location_bias), (Ov7725->pos.deflection));
//				printf("ServoMotor-->deflection = %d\n", ServoMotor->Deflection);
//				printf("GOODSTATUS %d\t mode %d\t Sflag %d\n", Ov7725->GOODSTATUS, Ov7725->mode, Sflag);
//				printf("distance %d\n", Ov7725->distance);
//				printf("PV %d, SV %d\n",MotorB->Velosity,MotorB->Target_Velosity);
//				printf("\n");
//				OV_delay();
//				LPLD_LPTMR_DelayMs(300);
			/*图片*/
//				Ov7725->display();
//			#endif
			Ov7725->restart();
		}

//		while(Bluetooth->Command.STOP)
//		{	;	}
	}
}
