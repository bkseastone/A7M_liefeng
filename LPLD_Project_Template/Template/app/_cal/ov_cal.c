#include "ov_cal.h"
#include "OV7725_eagle.h"
#include "math.h"
#include "rectify.h"
#include "servoPID.h"
#include "motor.h"
extern MotorTypeDef			*MotorB;

extern int16                RectifyX[60][81];
extern int16                RectifyY[60][81];
extern OvTypeDef			*Ov7725;
extern OV_pictureTypeDef_SRAM OV_pictures_SRAM @(OV_binary_ADDR+2);
extern OV_pictureTypeDef OV_pictures @OV_binary_BONDADDR(0, 16);
extern WeizhiPIDTypeDef		*Weizhi_PID;
int tmpL_location_bias;
int tmpR_location_bias;
int tmpL_location_bias2;
int tmpR_location_bias2;
int tmpL_location_bias3;
int tmpR_location_bias3;
uint8                Sflag=0;
uint8				 Sflag_MARK = 0;
//权重
#define CNST_3			2.0f	//曲率变化灵敏度(只能看到一边时)
#define CNST_4			1.0f	//(MotorB->Velosity/150)	//曲率变化灵敏度(两边都能看到时)(弯道时)(取弯道外侧曲率)
#define CNST_5			1		//车身角度权重
#define	CNST6			1		//底部偏移权重(加权平均用)
#define	CNST7			1		//远部偏移权重(加权平均用)
#define	CNST8			0.75	//弯道偏移权重(加权平均用)
//阈值
#define	MIN_deflection	0		//角度最小动作量
#define	BIAS_POSION		0		//偏内侧行驶量
//#define	POS_BORDER_MARK	20		//若扫到距边沿20列了,则认为另一边没有识别的必要性
//道路识别(主)
#define	POS_DIS_		35		//实际位置(加权平均用) 21(不压线) 25(压线) 29  35 36 37
#define	POS_PRE_		65		//弯道模式中提前打用(依照距离)
#define	POS3_DIS_		(45-(MotorB->Velosity/100))		//弯道位置(加权平均用)
#define	POS_curve_S		0.0f	//小s弯偏移量矫正
//道路识别(辅)
#define	THRESHOLD		100		//前沿距离判定弯道(cm)
#define	THRESHOLD_S		40		//十字弯
//PID参数
#define	SERVO_PID_KP_S	0.4		//直道
#define	SERVO_PID_KP_C_s 0.35		//小s弯
#define	SERVO_PID_KP_C	0.7		//弯道 0.7

#pragma optimize=speed
void ov7725_cal(void)
{
    int row;
	uint16 tmp =0;
	float tmp_deflection;
	int tmpL_location_bias;
	int tmpR_location_bias;
	int tmpL_location_bias2;
	int tmpR_location_bias2;
	Ov7725->LOCK = 1;
	Ov7725->mode = 0;
	ov7725_get_border();//沿缘线寻黑边
	ov7725_get_2slope();//逐差法求两边沿线斜率
	//待定的二次关系(为方便调试，暂时用Servo_PID->Kp代替)
	Ov7725->gain = 0.8;
	//车身距左右边界虚拟距离
	if(Ov7725->pic.border_pos_L[CAMERA_H-1]==80){
		tmpL_location_bias = 0;
	}
	else{
		tmpL_location_bias = 1+Ov7725->pic.border_pos_L[CAMERA_H-1];
	}
	tmpR_location_bias = 80-Ov7725->pic.border_pos_R[CAMERA_H-1];
	if(Ov7725->pic.border_pos_L[POS_DIS_]==80){
		tmpL_location_bias2 = 0;
	}
	else{
		tmpL_location_bias2 = 1+Ov7725->pic.border_pos_L[POS_DIS_];
	}
	tmpR_location_bias2 = 80-Ov7725->pic.border_pos_R[POS_DIS_];
	if(Ov7725->pic.border_pos_L[POS3_DIS_]==80){
		tmpL_location_bias3 = 0;
	}
	else{
		tmpL_location_bias3 = 1+Ov7725->pic.border_pos_L[POS3_DIS_];
	}
	tmpR_location_bias3 = 80-Ov7725->pic.border_pos_R[POS3_DIS_];
	Ov7725->pos.location_bias = (int)(CNST6*(tmpR_location_bias-tmpL_location_bias));
	//十字弯识别
	ov7725_identify_cross();
	//车头距前边界真实距离
    for(row=CAMERA_H-1;row>=0;row--)
	{
		if(OV_pictures.pic1_data[row][40] > 0){
			  Ov7725->distance = RectifyY[row][40];
			  break;
		}
	}
	if(row<0){
		Ov7725->distance = RectifyY[0][40];
	}
	throttle_control();
	//曲率
	if(Ov7725->mode != 2){
		if(Ov7725->pic.end_L < Ov7725->pic.end_R){
			tmp_deflection = -CurvatureABC(RectifyX[Ov7725->pic.start_L][Ov7725->pic.border_pos_L[Ov7725->pic.start_L]], \
					RectifyY[Ov7725->pic.start_L][Ov7725->pic.border_pos_L[Ov7725->pic.start_L]], \
					RectifyX[((Ov7725->pic.start_L+Ov7725->pic.end_L)/2+Ov7725->pic.start_L)/2][Ov7725->pic.border_pos_L[((Ov7725->pic.start_L+Ov7725->pic.end_L)/2+Ov7725->pic.start_L)/2]], \
					RectifyY[((Ov7725->pic.start_L+Ov7725->pic.end_L)/2+Ov7725->pic.start_L)/2][Ov7725->pic.border_pos_L[((Ov7725->pic.start_L+Ov7725->pic.end_L)/2+Ov7725->pic.start_L)/2]], \
					RectifyX[(Ov7725->pic.start_L+Ov7725->pic.end_L)/2][Ov7725->pic.border_pos_L[(Ov7725->pic.start_L+Ov7725->pic.end_L)/2]], \
					RectifyY[(Ov7725->pic.start_L+Ov7725->pic.end_L)/2][Ov7725->pic.border_pos_L[(Ov7725->pic.start_L+Ov7725->pic.end_L)/2]]);
//			printf("QvLv %d\n",(int)(tmp_deflection*1000));
		}
		else{
			tmp_deflection = CurvatureABC(RectifyX[Ov7725->pic.start_R][Ov7725->pic.border_pos_R[Ov7725->pic.start_R]], \
					RectifyY[Ov7725->pic.start_R][Ov7725->pic.border_pos_R[Ov7725->pic.start_R]], \
					RectifyX[((Ov7725->pic.start_R+Ov7725->pic.end_R)/2+Ov7725->pic.start_R)/2][Ov7725->pic.border_pos_R[((Ov7725->pic.start_R+Ov7725->pic.end_R)/2+Ov7725->pic.start_R)/2]], \
					RectifyY[((Ov7725->pic.start_R+Ov7725->pic.end_R)/2+Ov7725->pic.start_R)/2][Ov7725->pic.border_pos_R[((Ov7725->pic.start_R+Ov7725->pic.end_R)/2+Ov7725->pic.start_R)/2]], \
					RectifyX[(Ov7725->pic.start_R+Ov7725->pic.end_R)/2][Ov7725->pic.border_pos_R[(Ov7725->pic.start_R+Ov7725->pic.end_R)/2]], \
					RectifyY[(Ov7725->pic.start_R+Ov7725->pic.end_R)/2][Ov7725->pic.border_pos_R[(Ov7725->pic.start_R+Ov7725->pic.end_R)/2]]);
//			printf("QvLv %d\n",(int)(tmp_deflection*1000));
		}
		if(fabsf(tmp_deflection)<0.7){
			Ov7725->QuLv = tmp_deflection;
			Ov7725->mode = 1;
		}
		else{
			if(fabsf((Ov7725->QuLv+tmp_deflection)/2)>=0.7){
				Ov7725->QuLv = tmp_deflection;
				Ov7725->mode = 0;
			}
			else{
				Ov7725->QuLv = Ov7725->QuLv+0.02;
				Ov7725->mode = 1;
			}
		}
	}
	if((Ov7725->distance) <= 70){
		Ov7725->mode = 1;
	}
/*	调试小S弯用
	Ov7725->GOODSTATUS = 1;
	Ov7725->distance = 200;
*/
	if(Sflag==0){
		ov7725_Spanduan();
	}
	if(Sflag>=1){
		Weizhi_PID->Kd = 5;
		Ov7725->GOODSTATUS = 1;
		Ov7725->distance = 200;//go on
		tmp = 0;
		for(row=0;row<=9;row++){
			tmp += (!OV_pictures_SRAM.pic1_data[13][row]);
		}
		if(tmp==0){
			Sflag_MARK ++;
		}
		if(Sflag_MARK >= 10){
			Sflag_MARK = 0;
			Sflag = 0;
		}
		MotorB->Target_Velosity=700;
		Weizhi_PID->Kp = SERVO_PID_KP_C_s;
		Ov7725->pos.location_bias = (int)((CNST6*(tmpR_location_bias - tmpL_location_bias) + CNST7*((float)tmpR_location_bias2 - tmpL_location_bias2))/2 + POS_curve_S);
//		Ov7725->pos.deflection = 0;
		Ov7725->LOCK = 0;
		return;
	}
	//十字
	if(Ov7725->mode ==2){
		Weizhi_PID->Kd = 0;
		Ov7725->LOCK = 0;
		return;
	}
	//直道
	if(Ov7725->mode == 0){
		Weizhi_PID->Kd = 0;
		//斜率
		if((Ov7725->pic.start_R - Ov7725->pic.end_R)>(Ov7725->pic.start_L - Ov7725->pic.end_L)){
			tmp_deflection = (((float)(RectifyX[Ov7725->pic.start_R][Ov7725->pic.border_pos_R[Ov7725->pic.start_R]])- \
				(float)(RectifyX[(Ov7725->pic.end_R+Ov7725->pic.start_R)/2][(Ov7725->pic.border_pos_R[Ov7725->pic.start_R]+Ov7725->pic.border_pos_R[Ov7725->pic.end_R])/2]))/ \
				((float)(RectifyY[(Ov7725->pic.end_R+Ov7725->pic.start_R)/2][(Ov7725->pic.border_pos_R[Ov7725->pic.start_R]+Ov7725->pic.border_pos_R[Ov7725->pic.end_R])/2])- \
				(float)(RectifyY[Ov7725->pic.start_R][Ov7725->pic.border_pos_R[Ov7725->pic.start_R]])));
		}
		else{
			tmp_deflection = (((float)(RectifyX[Ov7725->pic.start_L][Ov7725->pic.border_pos_L[Ov7725->pic.start_L]])- \
				(float)(RectifyX[(Ov7725->pic.end_L+Ov7725->pic.start_L)/2][(Ov7725->pic.border_pos_L[Ov7725->pic.start_L]+Ov7725->pic.border_pos_L[Ov7725->pic.end_L])/2]))/ \
				((float)(RectifyY[(Ov7725->pic.end_L+Ov7725->pic.start_L)/2][(Ov7725->pic.border_pos_L[Ov7725->pic.start_L]+Ov7725->pic.border_pos_L[Ov7725->pic.end_L])/2])- \
				(float)(RectifyY[Ov7725->pic.start_L][Ov7725->pic.border_pos_L[Ov7725->pic.start_L]])));
		}
		tmp_deflection = (CNST_5*180*atan(tmp_deflection)/9.4248);//(int)(((fabsf(Ov7725->pos.deflection-tmp_deflection))<80)?tmp_deflection:Ov7725->pos.deflection);
		tmp_deflection = ((fabsf(tmp_deflection)<MIN_deflection)?0:tmp_deflection);
		Ov7725->pos.deflection = (int)tmp_deflection;
		Weizhi_PID->Kp = SERVO_PID_KP_S;
		Ov7725->LOCK = 0;
		return;
	}
	//弯道
	if(Ov7725->mode == 1){
		Weizhi_PID->Kd = ((float)(MotorB->Velosity)/80.0f);
		//紧急情况
		if(Ov7725->distance <= 40){
			if(Ov7725->pic.exit_L){
				Ov7725->pos.location_bias = (int)(CNST8*(tmpR_location_bias3 - tmpL_location_bias3));
				Ov7725->pos.location_bias -= BIAS_POSION;
				tmp_deflection = CNST_3*(-2.37-Ov7725->QuLv);
				tmp_deflection -= (1.5*(POS_PRE_-50) +CNST_4*10);
				if((Ov7725->pic.start_L < 30)||(Ov7725->pic.end_L>55)){
					tmp_deflection = -tmp_deflection;
				}
			}
			else if(Ov7725->pic.exit_R){
				Ov7725->pos.location_bias = (int)(CNST8*(tmpR_location_bias3 - tmpL_location_bias3));
				Ov7725->pos.location_bias += BIAS_POSION;
				tmp_deflection = CNST_3*(2.37-Ov7725->QuLv);
				tmp_deflection += (1.5*(POS_PRE_-50) +CNST_4*10);
				if((Ov7725->pic.start_R < 30)||(Ov7725->pic.end_R>55)){
					tmp_deflection = -tmp_deflection;
				}
			}
//			if((Ov7725->pic.exit_L) || (Ov7725->pic.exit_R)){
//				throttle_control();
				tmp_deflection = ((fabsf(tmp_deflection)<MIN_deflection)?0:tmp_deflection);
				Ov7725->pos.deflection = (int)tmp_deflection;
				Weizhi_PID->Kp = SERVO_PID_KP_C;
				Ov7725->LOCK = 0;
//			}
		}
		//正常弯道情况
		else if(Ov7725->distance <= THRESHOLD){
//			if(((Ov7725->pic.exit_L==1)&&(Ov7725->pic.end_L<=4))||((Ov7725->pic.exit_R==1)&&(Ov7725->pic.end_R<=4))){
//				Ov7725->mode = 0;//go on //采用直线策略(斜率求车身偏向)
//			}
		//		else if((Ov7725->pic.start_R - Ov7725->pic.end_R)>(Ov7725->pic.start_L - Ov7725->pic.end_L)){
		//			printf("RR_LONG = %d\n",(RectifyY[Ov7725->pic.end_R][Ov7725->pic.border_pos_R[Ov7725->pic.end_R]]-RectifyY[Ov7725->pic.start_R][Ov7725->pic.border_pos_R[Ov7725->pic.start_R]]));
		//		}
		//		else if((Ov7725->pic.start_R - Ov7725->pic.end_R)<(Ov7725->pic.start_L - Ov7725->pic.end_L)){
		//			printf("LL_LONG = %d\n",(RectifyY[Ov7725->pic.end_L][Ov7725->pic.border_pos_L[Ov7725->pic.end_L]]-RectifyY[Ov7725->pic.start_L][Ov7725->pic.border_pos_L[Ov7725->pic.start_L]]));
		//		}
//			else{
				Ov7725->mode = 1;
				if((Ov7725->pic.end_L < Ov7725->pic.end_R)){
					Ov7725->pos.location_bias = (int)(CNST8*(tmpR_location_bias3 - tmpL_location_bias3));
					Ov7725->pos.location_bias -= BIAS_POSION;
					tmp_deflection = CNST_3*(-2.37-Ov7725->QuLv);
					if(Ov7725->distance <= POS_PRE_-10){
						tmp_deflection -= (1.5*(POS_PRE_-10 - Ov7725->distance) +CNST_4*10);
					}
					else if(Ov7725->distance <= POS_PRE_){
						tmp_deflection -= CNST_4*(POS_PRE_ - Ov7725->distance);
					}
					if((Ov7725->pic.start_L < 30)||(Ov7725->pic.end_L>55)){
						tmp_deflection = -tmp_deflection;
					}
				}
				else if((Ov7725->pic.end_R < Ov7725->pic.end_L)){
					Ov7725->pos.location_bias = (int)(CNST8*(tmpR_location_bias3 - tmpL_location_bias3));
					Ov7725->pos.location_bias += BIAS_POSION;
					tmp_deflection = CNST_3*(2.37-Ov7725->QuLv);
					if(Ov7725->distance <= POS_PRE_-10){
						tmp_deflection += (1.5*(POS_PRE_-10 - Ov7725->distance) +CNST_4*10);
					}
					else if(Ov7725->distance <= POS_PRE_){
						tmp_deflection += CNST_4*(POS_PRE_ - Ov7725->distance);
					}
					if((Ov7725->pic.start_R < 30)||(Ov7725->pic.end_R>55)){
						tmp_deflection = -tmp_deflection;
					}
				}
				//				throttle_control();
				//				tmp_deflection = ((fabsf(tmp_deflection)<MIN_deflection)?0:tmp_deflection);
				Ov7725->pos.deflection = (int)tmp_deflection;
				Weizhi_PID->Kp = SERVO_PID_KP_C;
				Ov7725->LOCK = 0;
				return;
//			}
		}
//		else if(0){
//			//障碍检测
//		}
	}
}
#pragma optimize=speed
void ov7725_identify_cross(void)
{
	int col;
	int row;
	uint32	tmp;
	int flag;
	flag = 0;
//	float tmp_deflection;
	for(row=30;row<45;row++){
		tmp=0;
		for(col=0;col<=9;col++){
			tmp += 0x01 & (OV_pictures_SRAM.pic1_data[row][col]>0);
		}
		if(tmp==0){
			flag = 1;
			break;
		}
	}
	if(flag==1){
		Ov7725->mode = 2;

//		Ov7725->pic.escape_position = 30;
//		for(row=CAMERA_H-1;row>Ov7725->pic.escape_position;row--){
//			Ov7725->pic.border_pos_L[row] = 80;
//			Ov7725->pic.border_pos_R[row] = 80;
//		}
//		ov7725_get_border();
//		ov7725_get_2slope();
	}
	else{
		Ov7725->mode = 0;
	}
//	else{


	/*
		tmp=0;
		flag=0;
		if(Ov7725->pic.end_L > Ov7725->pic.start_R){
				flag = -1;
		}
		else if((Ov7725->pic.start_L >= Ov7725->pic.end_R)&&(Ov7725->pic.end_L <= Ov7725->pic.end_R)){
				flag = -1;
		}
		else if(Ov7725->pic.end_R >= Ov7725->pic.start_L){
				flag = 1;
		}
		else if((Ov7725->pic.end_R <= Ov7725->pic.end_L)&&(Ov7725->pic.start_R >= Ov7725->pic.end_L)){
				flag = 1;
		}
//                for(col=0;col<=9;col++){
//                        tmp += 0x1 & (OV_pictures_SRAM.pic1_data[59][col]>0);
//                }
//                if(tmp==0){
//                        flag=0;
//                }
//                else{
//                        tmp=0;
//                }
		if(Ov7725->distance<=100){
				flag = 0;
		}
		if(flag==-1){
			for(col=0;col<=9;col++){
				tmp += (uint32)(OV_pictures_SRAM.pic1_data[Ov7725->pic.end_L-15][col]);
			}
			if(tmp == 0){
				Ov7725->mode = 2; //交叉路口模式(底端有黑边)
				Ov7725->pic.escape_position = Ov7725->pic.end_L-15;
				for(row=CAMERA_H-1;row>Ov7725->pic.escape_position;row--){
					Ov7725->pic.border_pos_L[row] = 80;
					Ov7725->pic.border_pos_R[row] = 80;
				}
				if(Ov7725->pic.end_L>=40){
					ov7725_get_border();
					ov7725_get_2slope();
				}
			}
			else{
				Ov7725->mode = 0;
			}
		}
		else if(flag==1){
			for(col=0;col<=9;col++){
				tmp += (uint32)(OV_pictures_SRAM.pic1_data[Ov7725->pic.end_R-15][col]);
			}
			if(tmp == 0){
				Ov7725->mode = 2;
				Ov7725->pic.escape_position = Ov7725->pic.end_R-15;
				for(row=CAMERA_H-1;row>Ov7725->pic.escape_position;row--){
					Ov7725->pic.border_pos_L[row] = 80;
					Ov7725->pic.border_pos_R[row] = 80;
				}
				if(Ov7725->pic.end_R>=40){
					ov7725_get_border();
					ov7725_get_2slope();
				}
			}
			else{
				Ov7725->mode = 0;
			}
		}
*/
//	}
//	//  实际斜率
//	if(Ov7725->mode == 2){
//		if((Ov7725->pic.start_R - Ov7725->pic.end_R)>(Ov7725->pic.start_L - Ov7725->pic.end_L)){
//			tmp_deflection = (((float)(RectifyX[Ov7725->pic.start_R][Ov7725->pic.border_pos_R[Ov7725->pic.start_R]])- \
//				(float)(RectifyX[(Ov7725->pic.end_R+Ov7725->pic.start_R)/2][(Ov7725->pic.border_pos_R[Ov7725->pic.start_R]+Ov7725->pic.border_pos_R[Ov7725->pic.end_R])/2]))/ \
//				((float)(RectifyY[(Ov7725->pic.end_R+Ov7725->pic.start_R)/2][(Ov7725->pic.border_pos_R[Ov7725->pic.start_R]+Ov7725->pic.border_pos_R[Ov7725->pic.end_R])/2])- \
//				(float)(RectifyY[Ov7725->pic.start_R][Ov7725->pic.border_pos_R[Ov7725->pic.start_R]])));
//		}
//		else{
//			tmp_deflection = (((float)(RectifyX[Ov7725->pic.start_L][Ov7725->pic.border_pos_L[Ov7725->pic.start_L]])- \
//				(float)(RectifyX[(Ov7725->pic.end_L+Ov7725->pic.start_L)/2][(Ov7725->pic.border_pos_L[Ov7725->pic.start_L]+Ov7725->pic.border_pos_L[Ov7725->pic.end_L])/2]))/ \
//				((float)(RectifyY[(Ov7725->pic.end_L+Ov7725->pic.start_L)/2][(Ov7725->pic.border_pos_L[Ov7725->pic.start_L]+Ov7725->pic.border_pos_L[Ov7725->pic.end_L])/2])- \
//				(float)(RectifyY[Ov7725->pic.start_L][Ov7725->pic.border_pos_L[Ov7725->pic.start_L]])));
//		}
//		tmp_deflection = (CNST_5*180*atan(tmp_deflection)/9.4248);
//		tmp_deflection = ((fabsf(tmp_deflection)<MIN_deflection)?0:tmp_deflection);
//		Ov7725->pos.deflection = (int)tmp_deflection;
//	}
}
#pragma optimize=speed
void ov7725_get_2slope(void)
{
	int row;
	char start_L_OK = 0, start_R_OK = 0, end_L_OK = 0, end_R_OK = 0;
//	int32 x_sum_L = 0, y_sum_L = 0, x_sum_R = 0, y_sum_R = 0;
	Ov7725->pic.start_L = 59;
	Ov7725->pic.end_L = 59;
	Ov7725->pic.start_R = 59;
	Ov7725->pic.end_R = 59;

	for(row=CAMERA_H-1;row>=0;row--)
	{
//		if((start_L_OK==0) && (end_L_OK==0) && (row<=(CAMERA_H/2-1))){
//			start_L_OK = 1;
//			end_L_OK = 1;
//			Ov7725->pic.exit_L = 0;
//		}
//		if((start_R_OK==0) && (end_R_OK==0) && (row<=(CAMERA_H/2-1))){
//			start_R_OK = 1;
//			end_R_OK = 1;
//			Ov7725->pic.exit_R = 0;
//		}
		if((start_L_OK==0) && ((Ov7725->pic.border_pos_L[row])!=80)){
			Ov7725->pic.start_L = row;
			start_L_OK = 1;

		}
		if((end_L_OK==0) && ((Ov7725->pic.border_pos_L[CAMERA_H-1-row])!=80)){
			Ov7725->pic.end_L = CAMERA_H-1-row;
			end_L_OK = 1;

		}
		if((start_R_OK==0) && ((Ov7725->pic.border_pos_R[row])!=80)){
			Ov7725->pic.start_R = row;
			start_R_OK = 1;
		}
		if((end_R_OK==0) && ((Ov7725->pic.border_pos_R[CAMERA_H-1-row])!=80)){
			Ov7725->pic.end_R = CAMERA_H-1-row;
			end_R_OK = 1;
		}
		if(start_L_OK&&end_L_OK&&start_R_OK&&end_R_OK)
			break;
	}
        Ov7725->pic.exit_R = (Ov7725->pic.start_R==Ov7725->pic.end_R)?0:1;
        Ov7725->pic.exit_L = (Ov7725->pic.start_L==Ov7725->pic.end_L)?0:1;
	if((Ov7725->pic.exit_L==0) || (Ov7725->pic.exit_R==0)){
		if(Ov7725->pic.exit_L==0){
			Ov7725->pic.start_L = Ov7725->pic.start_R+1;
			Ov7725->pic.end_L = Ov7725->pic.start_R+1;
		}
		else if(Ov7725->pic.exit_R==0){
			Ov7725->pic.start_R = Ov7725->pic.start_L+1;
			Ov7725->pic.end_R = Ov7725->pic.start_L+1;
		}
		Ov7725->GOODSTATUS = 0;
		if(Ov7725->mode != 2){
			Ov7725->mode = Ov7725->pic.exit_L ^ Ov7725->pic.exit_R;
		}
		return;
	}
	else if(Ov7725->mode != 2){
		Ov7725->GOODSTATUS = 1;
	}
}

#define HorizonShiftMAX		5	//相邻两行边缘点最大横向偏移量
#define CNST_1				2	//自一边缘线向另一边缘线搜索的起始偏移量
#pragma optimize=speed
void ov7725_get_border(void)
{
	int row, col;
	int lie_L, lie_R;
	char Is_L_OK = 0, Is_R_OK = 0;
	char Isall_L_OK = 0,Isall_R_OK = 0;
	int _i = 0;
	Ov7725->GOODSTATUS = 1;
	if(Ov7725->mode != 2){
		Ov7725->pic.escape_position = CAMERA_H-1;
	}
	for(row=Ov7725->pic.escape_position;row>=Ov7725->pic.escape_position;row--)
	{
		Is_L_OK = 0;
		Is_R_OK = 0;
		for(col=0;col<=((CAMERA_W/16)-1);col++)
		{
			if((Is_L_OK==0) && ((OV_pictures_SRAM.pic1_data[row][5-1-col])>0)){
				Is_L_OK = 1;
				lie_L = 5-1-col;
			};
			if((Is_R_OK==0) && ((OV_pictures_SRAM.pic1_data[row][5+col])>0)){
				Is_R_OK = 1;
				lie_R = 5+col;
			};
			if(Is_L_OK && Is_R_OK)
				break;
		}
		if(Is_L_OK){
			for(_i=7;_i>=0;_i--)
			{
				if((OV_pictures.pic1_data[row][_i+lie_L*8])>0){
					Ov7725->pic.border_pos_L[row] = _i+lie_L*8;
					break;
				}
			}
		}
		else{
			Ov7725->pic.border_pos_L[row] = 80; //80意味木有
		}
		if(Is_R_OK){
			for(_i=0;_i<=7;_i++)
			{
				if((OV_pictures.pic1_data[row][_i+lie_R*8])>0){
					Ov7725->pic.border_pos_R[row] = _i+lie_R*8;
					break;
				}
			}
		}
		else{
			Ov7725->pic.border_pos_R[row] = 80;
		}
		//若第一行39列和40列都为黑边的处理方案
		if(row==59){
			if((Ov7725->pic.border_pos_L[row]+1)==(Ov7725->pic.border_pos_R[row])){
				Ov7725->GOODSTATUS = 0;
				Ov7725->mode = 1;
				Ov7725->pic.exit_L = (OV_pictures.pic1_data[59][0]>0)&0x01;
				Ov7725->pic.exit_R = (OV_pictures.pic1_data[59][CAMERA_W-1]>0)&0x01;
				Ov7725->pic.border_pos_L[row] = (Ov7725->pic.exit_L==1)?Ov7725->pic.border_pos_R[row]:80;
				Ov7725->pic.border_pos_R[row] = (Ov7725->pic.exit_R==1)?Ov7725->pic.border_pos_R[row]:80;
				if(Ov7725->pic.exit_L != Ov7725->pic.exit_R){
					break;
				}
			}
		}

		if((((Ov7725->pic.border_pos_L[Ov7725->pic.escape_position]) != 80)&&    \
			((Ov7725->pic.border_pos_R[Ov7725->pic.escape_position]) != 80))  || \
			(((Ov7725->pic.border_pos_L[Ov7725->pic.escape_position]) == 80)&&   \
			((Ov7725->pic.border_pos_R[Ov7725->pic.escape_position]) == 80)))
		{
			Ov7725->pic.escape_position--;
		}
		else if(row!=CAMERA_H-1){
			if((Ov7725->pic.border_pos_L[Ov7725->pic.escape_position]) == 80){
				Isall_L_OK = 1;
			}
			else if((Ov7725->pic.border_pos_R[Ov7725->pic.escape_position]) == 80){
				Isall_R_OK = 1;
			}
		}
		if(Ov7725->pic.escape_position<0){
			Ov7725->pic.escape_position = -1;
			return;
		}
	}
	_i = 1;
	if(((Ov7725->pic.border_pos_L[Ov7725->pic.escape_position]) == 80) && \
		((Ov7725->pic.border_pos_R[Ov7725->pic.escape_position]) != 80))
	{
		for(row=Ov7725->pic.escape_position-1;row>=0;row--)
		{
			lie_R = Ov7725->pic.border_pos_R[row+1];
			if(_i>=HorizonShiftMAX){
				Isall_R_OK = 1;
			}
//			if(lie_R < POS_BORDER_MARK){
//				Isall_L_OK = 1;
//			}
			if(Isall_R_OK == 0){
				if((OV_pictures.pic1_data[row][lie_R])>0){
					if(((lie_R+_i)>=CAMERA_W)||(OV_pictures.pic1_data[row][lie_R+_i])>0){
						if(((lie_R-_i)<0)||(OV_pictures.pic1_data[row][lie_R-_i])>0){
							if(((lie_R+_i)>=CAMERA_W)||(OV_pictures.pic1_data[row+1][lie_R+_i])>0){
								if(((lie_R-_i)<0)||(OV_pictures.pic1_data[row+1][lie_R-_i])>0){
									_i++;
									row++;
								}
								else{
									Ov7725->pic.border_pos_R[row] = lie_R-_i;
									Ov7725->pic.border_pos_R[row+1] = lie_R-(_i-1);
									_i = 1;
								}
							}
							else{
								Ov7725->pic.border_pos_R[row] = lie_R+_i;
								Ov7725->pic.border_pos_R[row+1] = lie_R+(_i-1);
								_i = 1;
							}
						}
						else{
							Ov7725->pic.border_pos_R[row] = lie_R-(_i-1);
							Ov7725->pic.border_pos_R[row+1] = lie_R-(_i-1);
							_i = 1;
						}
					}
					else{
						Ov7725->pic.border_pos_R[row] = lie_R+(_i-1);
						Ov7725->pic.border_pos_R[row+1] = lie_R+(_i-2);
						_i = 1;
					}
				}
				else if(((lie_R+_i)<CAMERA_W)&&(OV_pictures.pic1_data[row][lie_R+_i])>0){
					Ov7725->pic.border_pos_R[row] = lie_R+_i;
					_i = 1;
				}
				else if(((lie_R-_i)>=0)&&(OV_pictures.pic1_data[row][lie_R-_i])>0){
					Ov7725->pic.border_pos_R[row] = lie_R-_i;
					_i = 1;
				}
				else if(((lie_R+_i)<CAMERA_W)&&(OV_pictures.pic1_data[row+1][lie_R+_i])>0){
					_i++;
					row++;
				}
				else if(((lie_R-_i)>=0)&&(OV_pictures.pic1_data[row+1][lie_R-_i])>0){
					_i++;
					row++;
				}
				else{
					Ov7725->pic.border_pos_R[row] = 80;
					Isall_R_OK = 1;
				}
			}
			else{
				Ov7725->pic.border_pos_R[row] = 80;
			}
		}
		if(Ov7725->GOODSTATUS){
			for(row=Ov7725->pic.escape_position-1;row>=0;row--)
			{
				if(OV_pictures.pic1_data[row][0]>0){
					//Ov7725->pic.border_pos_L[row] = 0;
					break;
				}
				else{
					Ov7725->pic.border_pos_L[row] = 80;
				}
			}
			if(row<20){
				Isall_L_OK = 1;
				Ov7725->pic.border_pos_L[row] = 80;
				Ov7725->pic.escape_position = row;
			}
			else{
				Ov7725->pic.escape_position = row;
				for(_i=0;_i<=40;_i++){
					if(OV_pictures.pic1_data[row][_i]==0){
						Ov7725->pic.border_pos_L[row] = _i-1;
						break;
					}
				}
			}
            _i = 1;
			for(row=Ov7725->pic.escape_position-1;row>=0;row--)
			{
				lie_L = Ov7725->pic.border_pos_L[row+1];
				if(_i>=HorizonShiftMAX){
					Isall_L_OK = 1;
				}
				if(Isall_L_OK == 0){
					if((OV_pictures.pic1_data[row][lie_L])>0){
						if(((lie_L+_i)>=CAMERA_W)||((OV_pictures.pic1_data[row][lie_L+_i])>0)){
							if(((lie_L-_i)<0)||((OV_pictures.pic1_data[row][lie_L-_i])>0)){
								if(((lie_L+_i)>=CAMERA_W)||((OV_pictures.pic1_data[row+1][lie_L+_i])>0)){
									if(((lie_L-_i)<0)||((OV_pictures.pic1_data[row+1][lie_L-_i])>0)){
										_i++;
										row++;
									}
									else{
										Ov7725->pic.border_pos_L[row] = lie_L-_i;
										Ov7725->pic.border_pos_L[row+1] = lie_L-(_i-1);
										_i = 1;
									}
								}
								else{
									Ov7725->pic.border_pos_L[row] = lie_L+_i;
									Ov7725->pic.border_pos_L[row+1] = lie_L+(_i-1);
									_i = 1;
								}
							}
							else{
								Ov7725->pic.border_pos_L[row] = lie_L-(_i-1);
								Ov7725->pic.border_pos_L[row+1] = lie_L-(_i-2);
								_i = 1;
							}
						}
						else{
							Ov7725->pic.border_pos_L[row] = lie_L+(_i-1);
							Ov7725->pic.border_pos_L[row+1] = lie_L+(_i-1);
							_i = 1;
						}
					}
					else if(((lie_L+_i)<CAMERA_W)&&(OV_pictures.pic1_data[row][lie_L+_i])>0){
						Ov7725->pic.border_pos_L[row] = lie_L+_i;
						_i = 1;
					}
					else if(((lie_L-_i)>=0)&&(OV_pictures.pic1_data[row][lie_L-_i])>0){
						Ov7725->pic.border_pos_L[row] = lie_L-_i;
						_i = 1;
					}
					else if(((lie_L+_i)<CAMERA_W)&&(OV_pictures.pic1_data[row+1][lie_L+_i])>0){
						_i++;
						row++;
					}
					else if(((lie_L-_i)>=0)&&(OV_pictures.pic1_data[row+1][lie_L-_i])>0){
						_i++;
						row++;
					}
					else{
						Ov7725->pic.border_pos_L[row] = 80;
						Isall_L_OK = 1;
					}
				}
				else{
					Ov7725->pic.border_pos_L[row] = 80;
				}
				if((fabsf(Ov7725->pic.border_pos_L[row]-Ov7725->pic.border_pos_R[row]))<=5){
					Ov7725->pic.border_pos_L[row] = 80;
					Isall_L_OK = 1;
				}
			}
		}
	}
	else if(((Ov7725->pic.border_pos_R[Ov7725->pic.escape_position]) == 80) && \
			((Ov7725->pic.border_pos_L[Ov7725->pic.escape_position]) != 80))
	{
		for(row=Ov7725->pic.escape_position-1;row>=0;row--)
		{
			lie_L = Ov7725->pic.border_pos_L[row+1];
			if(_i>=HorizonShiftMAX){
				Isall_L_OK = 1;
			}
//			if(lie_L > 80-POS_BORDER_MARK){
//				Isall_R_OK = 1;
//			}
			if(Isall_L_OK == 0){
				if((OV_pictures.pic1_data[row][lie_L])>0){
					if(((lie_L+_i)>=CAMERA_W)||(OV_pictures.pic1_data[row][lie_L+_i])>0){
						if(((lie_L-_i)<0)||(OV_pictures.pic1_data[row][lie_L-_i])>0){
							if(((lie_L+_i)>=CAMERA_W)||(OV_pictures.pic1_data[row+1][lie_L+_i])>0){
								if(((lie_L-_i)<0)||(OV_pictures.pic1_data[row+1][lie_L-_i])>0){
									_i++;
									row++;
								}
								else{
									Ov7725->pic.border_pos_L[row] = lie_L-_i;
									Ov7725->pic.border_pos_L[row+1] = lie_L-(_i-1);
									_i = 1;
								}
							}
							else{
								Ov7725->pic.border_pos_L[row] = lie_L+_i;
								Ov7725->pic.border_pos_L[row+1] = lie_L+(_i-1);
								_i = 1;
							}
						}
						else{
							Ov7725->pic.border_pos_L[row] = lie_L-(_i-1);
							Ov7725->pic.border_pos_L[row+1] = lie_L-(_i-2);
							_i = 1;
						}
					}
					else{
						Ov7725->pic.border_pos_L[row] = lie_L+(_i-1);
						Ov7725->pic.border_pos_L[row+1] = lie_L+(_i-1);
						_i = 1;
					}
				}
				else if(((lie_L+_i)<CAMERA_W)&&(OV_pictures.pic1_data[row][lie_L+_i])>0){
					Ov7725->pic.border_pos_L[row] = lie_L+_i;
					_i = 1;
				}
				else if(((lie_L-_i)>=0)&&(OV_pictures.pic1_data[row][lie_L-_i])>0){
					Ov7725->pic.border_pos_L[row] = lie_L-_i;
					_i = 1;
				}
				else if(((lie_L+_i)<CAMERA_W)&&(OV_pictures.pic1_data[row+1][lie_L+_i])>0){
					_i++;
					row++;
				}
				else if(((lie_L-_i)>=0)&&(OV_pictures.pic1_data[row+1][lie_L-_i])>0){
					_i++;
					row++;
				}
				else{
					Ov7725->pic.border_pos_L[row] = 80;
					Isall_L_OK = 1;
				}
			}
			else{
				Ov7725->pic.border_pos_L[row] = 80;
			}
		}

		if(Ov7725->GOODSTATUS){
			for(row=Ov7725->pic.escape_position-1;row>=0;row--)
			{
				if(OV_pictures.pic1_data[row][79]>0){
					//Ov7725->pic.border_pos_R[row] = 79;
					break;
				}
				else{
					Ov7725->pic.border_pos_R[row] = 80;
				}
			}
			if(row<20){
				Isall_R_OK = 1;
				Ov7725->pic.escape_position = row;
				Ov7725->pic.border_pos_R[row] = 80;
			}
			else{
				Ov7725->pic.escape_position = row;
				for(_i=79;_i>=40;_i--){
					if(OV_pictures.pic1_data[row][_i]==0){
						Ov7725->pic.border_pos_R[row] = _i+1;
						break;
					}
				}
			}
			_i = 1;
			for(row=Ov7725->pic.escape_position-1;row>=0;row--)
			{
				lie_R = Ov7725->pic.border_pos_R[row+1];
				if(_i>=HorizonShiftMAX){
					Isall_R_OK = 1;
				}
				if(Isall_R_OK == 0){
					if((OV_pictures.pic1_data[row][lie_R])>0){
						if(((lie_R+_i)>=CAMERA_W)||((OV_pictures.pic1_data[row][lie_R+_i])>0)){
							if(((lie_R-_i)<0)||((OV_pictures.pic1_data[row][lie_R-_i])>0)){
								if(((lie_R+_i)>=CAMERA_W)||((OV_pictures.pic1_data[row+1][lie_R+_i])>0)){
									if(((lie_R-_i)<0)||((OV_pictures.pic1_data[row+1][lie_R-_i])>0)){
										_i++;
										row++;
									}
									else{
										Ov7725->pic.border_pos_R[row] = lie_R-_i;
										Ov7725->pic.border_pos_R[row+1] = lie_R-(_i-1);
										_i = 1;
									}
								}
								else{
									Ov7725->pic.border_pos_R[row] = lie_R+_i;
									Ov7725->pic.border_pos_R[row+1] = lie_R+(_i-1);
									_i = 1;
								}
							}
							else{
								Ov7725->pic.border_pos_R[row] = lie_R-(_i-1);
								Ov7725->pic.border_pos_R[row+1] = lie_R-(_i-1);
								_i = 1;
							}
						}
						else{
							Ov7725->pic.border_pos_R[row] = lie_R+(_i-1);
							Ov7725->pic.border_pos_R[row+1] = lie_R+(_i-2);
							_i = 1;
						}
					}
					else if(((lie_R+_i)<CAMERA_W)&&(OV_pictures.pic1_data[row][lie_R+_i])>0){
						Ov7725->pic.border_pos_R[row] = lie_R+_i;
						_i = 1;
					}
					else if(((lie_R-_i)>=0)&&(OV_pictures.pic1_data[row][lie_R-_i])>0){
						Ov7725->pic.border_pos_R[row] = lie_R-_i;
						_i = 1;
					}
					else if(((lie_R+_i)<CAMERA_W)&&(OV_pictures.pic1_data[row+1][lie_R+_i])>0){
						_i++;
						row++;
					}
					else if(((lie_R-_i)>=0)&&(OV_pictures.pic1_data[row+1][lie_R-_i])>0){
						_i++;
						row++;
					}
					else{
						Ov7725->pic.border_pos_R[row] = 80;
						Isall_R_OK = 1;
					}
				}
				else{
					Ov7725->pic.border_pos_R[row] = 80;
				}
				if((fabsf(Ov7725->pic.border_pos_R[row]-Ov7725->pic.border_pos_L[row]))<=5){
					Ov7725->pic.border_pos_R[row] = 80;
					Isall_R_OK = 1;
				}
			}
		}
	}
}

void ov7725_Spanduan(void)
{
	char                 zouxiang[CAMERA_H-1];
	int16                bianyuan[CAMERA_H];
	int                  zouxiangjilu[CAMERA_H+1]={0};
	int16                Zuobiao[CAMERA_H+1][2];
	int16                Zuobiao2[CAMERA_H+1][2];
	int jishu=0,ZYflag=0;
	int jilu=0;
	int jizouxiang=0;
	int jishu2=0;
	if((Ov7725->pic.start_R - Ov7725->pic.end_R)>(Ov7725->pic.start_L - Ov7725->pic.end_L))
		for(jishu=0;jishu<=59;jishu++)
		{
			bianyuan[jishu]=RectifyX[jishu][Ov7725->pic.border_pos_R[jishu]];
			ZYflag=1;
		}
	else
		for (jishu=0;jishu<=59;jishu++)
		{
			bianyuan[jishu]=RectifyX[jishu][Ov7725->pic.border_pos_L[jishu]];
			ZYflag=2;
		}
	for(jishu=0;jishu<=58;jishu++)
	{
		if(bianyuan[jishu+1]>bianyuan[jishu])
			zouxiang[jishu]=1;
		else if(bianyuan[jishu+1]<bianyuan[jishu])
			zouxiang[jishu]=2;
		else if(bianyuan[jishu+1]==bianyuan[jishu])
			zouxiang[jishu]=3;
		else
			zouxiang[jishu]=25;
	}
	for(jishu=0;jishu<=58;jishu++)
	{
		if (jilu==0){
			if( bianyuan[jishu]!=300 ){
				zouxiangjilu[jilu]=jishu;
				jilu++;
				jizouxiang=zouxiang[jishu];
			}
		}
		else{
			if( bianyuan[jishu]!=300 && zouxiang[jishu]!=jizouxiang && zouxiang[jishu]!=3){
				if(jishu!=3){
					zouxiangjilu[jilu]=jishu;
					jilu++;
					jizouxiang=zouxiang[jishu];
				}
			}
		}
	}
	for(jishu=0;jishu<=60;jishu++){
		if(zouxiangjilu[jishu]!=0||zouxiangjilu[jishu+1]!=0)
		{
			if(ZYflag==1)
			{
				Zuobiao[jishu][0]=RectifyX[zouxiangjilu[jishu]][Ov7725->pic.border_pos_R[zouxiangjilu[jishu]]];
				Zuobiao[jishu][1]=RectifyY[zouxiangjilu[jishu]][Ov7725->pic.border_pos_R[zouxiangjilu[jishu]]];
			}
			else if(ZYflag==2)
			{
				Zuobiao[jishu][0]=RectifyX[zouxiangjilu[jishu]][Ov7725->pic.border_pos_L[zouxiangjilu[jishu]]];
				Zuobiao[jishu][1]=RectifyY[zouxiangjilu[jishu]][Ov7725->pic.border_pos_L[zouxiangjilu[jishu]]];
			}
		}
	}


	for (jishu=0;jishu<=59;jishu++){
		if (Zuobiao[jishu][1]<=200 && fabsf(Zuobiao[jishu+1][0]-Zuobiao[jishu][0])>=7){
			Zuobiao2[jishu2][0]=Zuobiao[jishu][0];
			Zuobiao2[jishu2][1]=Zuobiao[jishu][1];
			jishu2++;
		}
	}
	for (jishu=0;jishu<=59;jishu++){
	//     printf("%d\n",Zuobiao2[jishu+1][1]-Zuobiao2[jishu][1]);
		if ((Zuobiao2[jishu][1]-Zuobiao2[jishu+1][1])>=55&&(Zuobiao2[jishu][1]-Zuobiao2[jishu+1][1])<=90&&jishu+1!=1&&Zuobiao2[jishu+1][1]>=32){
			Sflag=1;
			break;
		}
		if (Zuobiao2[jishu+2][1]==0)
			break;
	}
	for (jishu=0;jishu<=60;jishu++)
	{
	  zouxiangjilu[jishu]=0;
	  bianyuan[jishu]=0;
	  Zuobiao2[jishu][0]=0;
	  Zuobiao2[jishu][1]=0;
	  Zuobiao[jishu][0]=0;
	  Zuobiao[jishu][1]=0;
	}
}