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
//extern myic				*Encoder_Feedback;
extern myic					*Encoder_Test;
extern MotorTypeDef			*MotorF;
extern MotorTypeDef			*MotorB;
extern OvTypeDef			*Ov7725;
extern ServomotorTypeDef	*ServoMotor;
extern myadc				*Battery;
extern SystemTypeDef		*Sys;
//extern BluetoothTypeDef 	*Bluetooth;
//extern ServoPIDTypeDef		*Servo_PID;
extern motorPIDTypeDef		*motor_PID;
extern WeizhiPIDTypeDef		*Weizhi_PID;
extern int                  Sflag;
extern OV_pictureTypeDef_SRAM	OV_pictures_SRAM @(OV_binary_ADDR+2);
extern uint16 V_MODE1_NORMAL;
extern uint16 V_MODE1_SHIFT;
extern uint16 Velosity_s;
extern uint16 VELOCITY_MAX;
extern float SERVO_PID_KP_C;
void plan_select(void);
//#undefine					DEBUG_PRINT
#pragma optimize=none
void main (void)
{
//	Battery->init();
//	Bluetooth->init();
	Ov7725->init();
//	LPLD_LPTMR_DelayMs(1000);//调试用
	ServoMotor->init();
	MotorB->init();
	MotorF->init();
	Encoder_Test->init();
	Sys->init();
	init_photocellB2B3();
	init_PlanSelection();
	init_bell();
	plan_select();
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
			#if defined(DEBUG_PRINT)
//				printf("L(H): %d--->%d\n", Ov7725->pic.start_L, Ov7725->pic.end_L);
//				printf("R(H): %d--->%d\n", Ov7725->pic.start_R, Ov7725->pic.end_R);
//				printf("L: %d	R: %d\n", Ov7725->pic.exit_L, Ov7725->pic.exit_R);
//				printf("Location_bias = %d\nDeflection = %d\n", (Ov7725->pos.location_bias), (Ov7725->pos.deflection));
//				printf("ServoMotor-->deflection = %d\n", ServoMotor->Deflection);
//				printf("GOODSTATUS %d\t mode %d\t Sflag %d\n", Ov7725->GOODSTATUS, Ov7725->mode, Sflag);
//				printf("distance %d\n", Ov7725->distance);
//				printf("QvLv %d\n", (int)(Ov7725->QuLv*1000));
//				printf("PV %d, SV %d\n",MotorB->Velosity,MotorB->Target_Velosity);
//				printf("\n");
//				LPLD_LPTMR_DelayMs(300);
//				OV_delay();
			/*图片*/
//				Ov7725->display();
			#endif
			Ov7725->restart();
		}

//		while(Bluetooth->Command.STOP)
//		{	;	}
	}
}

void plan_select(void)
{
	//s弯是否加入 //3
	if(PTBn_I(8)==1){ //4
		SERVO_PID_KP_C = 0.7;
	}
	else{
		SERVO_PID_KP_C = 0.8;
	}
	if(PTBn_I(7)==1){ //5
		VELOCITY_MAX = 900;
	}
	else{
		VELOCITY_MAX = 1100;
	}
	if(PTBn_I(6)==1){ //6
		Velosity_s = 700;
	}
	else{
		Velosity_s = 550;
	}
	if(PTBn_I(5)==1){ //7
		Ov7725->SHIFT = 0;
	}
	else{
		Ov7725->SHIFT = 1;
		V_MODE1_SHIFT = 550;
	}
	if(PTBn_I(4)==1){ //8
		V_MODE1_NORMAL = 750;
	}
	else{
		V_MODE1_NORMAL = 800;
	}
	if(PTBn_I(10)==0){ //2
		VELOCITY_MAX = 1100;
		V_MODE1_NORMAL = 741;
		V_MODE1_SHIFT = 500;
	}
	Ov7725->CNT = 0;
}
