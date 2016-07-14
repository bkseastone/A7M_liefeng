#include "ov_cal.h"
#include "OV7725_eagle.h"
#include "math.h"
#include "rectify.h"
#include "servoPID.h"
#include "motor.h"
#include "SUP_check.h"

extern MotorTypeDef			*MotorB;
extern int16                RectifyX[60][81];
extern int16                RectifyY[60][81];
extern OvTypeDef			*Ov7725;
extern OV_pictureTypeDef_SRAM OV_pictures_SRAM @(OV_binary_ADDR+2);
extern OV_pictureTypeDef OV_pictures @OV_binary_BONDADDR(0, 16);
extern WeizhiPIDTypeDef		*Weizhi_PID;
extern PhotocellTypeDef		*StartEndLine;
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
#define	CNST6			1		//底部偏移权重(s加权平均用)
#define	CNST7			1		//远部偏移权重(s加权平均用)
#define	CNST8			0.75	//弯道偏移权重(加权平均用)
#define	CNST9			1.5		//障碍中心点权重
//阈值
#define	MIN_deflection	0		//角度最小动作量
#define	BIAS_POSION		0		//偏内侧行驶量
//#define	POS_BORDER_MARK	20		//若扫到距边沿20列了,则认为另一边没有识别的必要性
//道路识别(主)
#define	POS1_DIS_		(CAMERA_H-1)	//基础位置偏差
#define	POS2_DIS_		35		//(S加权平均用) 21(不压线) 25(压线) 29  35 36 37
#define	POS3_DIS_		(45-(MotorB->Velosity/100))		//弯道位置(mode1加权平均用)
#define	POS_PRE_		65		//弯道模式中提前打用(依照距离)
#define	POS_curve_S		0.0f	//小s弯偏移量矫正
//道路识别(辅)
#define	THRESHOLD		100		//前沿距离判定弯道(cm)
#define	THRESHOLD_S		40		//十字弯
//PID参数
#define	SERVO_PID_KP_S	0.4		//直道
#define	SERVO_PID_KP_C_s 0.35		//小s弯
#define	SERVO_PID_KP_C	0.7		//弯道 0.7
#define	SERVO_PID_KP_B	0.7		//障碍
#define SHIFT_TINE		1
#pragma optimize=speed
void ov7725_cal(void)
{
    int row,col;
	unsigned char row_bar=0,col_bar_L=0,col_bar_R=0;
	int tmp1_bar,tmp2_bar;
	uint16 tmp =0;
	float tmp_deflection;
	int tmpL_location_bias;
	int tmpR_location_bias;
	int tmpL_location_bias2;
	int tmpR_location_bias2;
	Ov7725->LOCK = 1;
	StartEndLine->alert = 0;
	Ov7725->mode = 0;
//	if(fabsf(Weizhi_PID->e_dis)<=16){
//		Ov7725->CNT = 0;
//	}
//	else{
//		Ov7725->CNT = SHIFT_TINE;
//	}
	Ov7725->pic.exit_L = 1;
	Ov7725->pic.exit_R = 1;
	Ov7725->pos.deflection = 0;
	ov7725_get_border();//沿缘线寻黑边
	if(Ov7725->mode == 6){
		Ov7725->pos.location_bias = -40;
	}
	else if(Ov7725->mode == 9){
		Ov7725->pos.location_bias = 40;
	}
	ov7725_get_2slope();//提取边线信息
	//待定的二次关系(为方便调试，暂时用Servo_PID->Kp代替)
	//Ov7725->gain = 0.8;
	//车身距左右边界虚拟距离
	if(Ov7725->pic.border_pos_L[POS1_DIS_]==80){
		tmpL_location_bias = 0;
	}
	else{
		tmpL_location_bias = 1+Ov7725->pic.border_pos_L[POS1_DIS_];
	}
	tmpR_location_bias = 80-Ov7725->pic.border_pos_R[POS1_DIS_];
	if(Ov7725->pic.border_pos_L[POS2_DIS_]==80){
		tmpL_location_bias2 = 0;
	}
	else{
		tmpL_location_bias2 = 1+Ov7725->pic.border_pos_L[POS2_DIS_];
	}
	tmpR_location_bias2 = 80-Ov7725->pic.border_pos_R[POS2_DIS_];
	if(Ov7725->pic.border_pos_L[POS3_DIS_]==80){
		tmpL_location_bias3 = 0;
	}
	else{
		tmpL_location_bias3 = 1+Ov7725->pic.border_pos_L[POS3_DIS_];
	}
	tmpR_location_bias3 = 80-Ov7725->pic.border_pos_R[POS3_DIS_];
	Ov7725->pos.location_bias = (int)(tmpR_location_bias-tmpL_location_bias);
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
	//障碍判断
	if(Ov7725->mode != 2){
		row_bar = 0;
		if(Ov7725->pic.start_L<Ov7725->pic.start_R){
			tmp1_bar = Ov7725->pic.start_L;
		}
		else{
			tmp1_bar = Ov7725->pic.start_R;
		}
		if(Ov7725->pic.end_L>Ov7725->pic.end_R){
			tmp2_bar = Ov7725->pic.end_L;
		}
		else{
			tmp2_bar = Ov7725->pic.end_R;
		}
		tmp2_bar = ((tmp2_bar>=16)?tmp2_bar:16);//目的：滤掉误判为s的情况(逐步变大常数即可)
		for(row=tmp1_bar;row>=tmp2_bar;row--){
			for(col=Ov7725->pic.border_pos_L[row]+1;col<=Ov7725->pic.border_pos_R[row]-1;col++){//!!!bug!!!//会不会是引起误判s的原因：从左往右扫
				if(OV_pictures.pic1_data[row][col]>0){
					StartEndLine->alert = 1;
					row_bar++;
					break;
				}
			}
			if(row_bar>7){ // 滤掉终止线误判为障碍的情况
				row_bar = row+6;
				break;
			}
		}
		if(row_bar>7){
			Ov7725->mode = 3;
			Sflag = 0;
			Sflag_MARK = 0;
		}
		else{
			row_bar = 0;
		}
	}
	//曲率
	if((Ov7725->mode != 3)&&(Ov7725->mode != 2)){
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
//			if(fabsf((Ov7725->QuLv+tmp_deflection)/2)>=0.7){
				Ov7725->QuLv = tmp_deflection;
				Ov7725->mode = 0;
//			}
//			else{
//				Ov7725->QuLv = Ov7725->QuLv+0.02;
//				Ov7725->mode = 1;
//			}
		}
	}
	if((Ov7725->distance) <= 70){
		Ov7725->mode = 1;
	}
	if((Ov7725->distance>200)||(fabsf(tmp_deflection)>0.85)){ //此处参数待测bug
		StartEndLine->alert = 1;
	}
	else{
		StartEndLine->alert = 0;
	}
	throttle_control();
/*	调试小S弯用
	Ov7725->GOODSTATUS = 1;
	Ov7725->distance = 200;
*/
	if((Sflag==0)&&(Ov7725->mode != 3)){
		ov7725_Spanduan();
	}
	if(Sflag>=1){
		Weizhi_PID->Kd = 5;
		Ov7725->GOODSTATUS = 1;
		Ov7725->distance = 200;//go on
		StartEndLine->alert = 0;
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
		MotorB->Target_Velosity=700; //700
		Weizhi_PID->Kp = SERVO_PID_KP_C_s;
		Ov7725->pos.location_bias = (int)((CNST6*(tmpR_location_bias - tmpL_location_bias) + CNST7*((float)tmpR_location_bias2 - tmpL_location_bias2))/2 + POS_curve_S);
//		Ov7725->pos.deflection = 0;
		Ov7725->LOCK = 0;
		return;
	}
	//十字
	if(Ov7725->mode ==2){
		Weizhi_PID->Kd = 0;
//		Weizhi_PID->Kp = SERVO_PID_KP_S;
		StartEndLine->alert = 0;
		Ov7725->LOCK = 0;
		return;
	}
	//直道
	if((Ov7725->mode == 0)||(Ov7725->mode == 3)){
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
		col_bar_L = 0;
		col_bar_R = 0;
		if(row_bar>0){
			for(col=Ov7725->pic.border_pos_L[row_bar]+1;col<=Ov7725->pic.border_pos_R[row_bar]-1;col++){
				if(OV_pictures.pic1_data[row_bar][col]>0){
					col_bar_L = col;
					break;
				}
			}
			for(col=Ov7725->pic.border_pos_R[row_bar]-1;col>=Ov7725->pic.border_pos_L[row_bar]+1;col--){
				if(OV_pictures.pic1_data[row_bar][col]>0){
					col_bar_R = col;
					break;
				}
			}
			for(col=col_bar_L;col<=col_bar_R;col++){
				if(OV_pictures.pic1_data[row_bar][col]==0){
					col_bar_R = 0;
					Ov7725->mode = 0;
					Weizhi_PID->Kp = SERVO_PID_KP_S;
					Ov7725->LOCK = 0;
					return;
				}
			}
			if(col_bar_R==0){
				Ov7725->mode = 0;
				Weizhi_PID->Kp = SERVO_PID_KP_S;
				Ov7725->LOCK = 0;
				return;
			}
			if((col_bar_L-Ov7725->pic.border_pos_L[row_bar])<(Ov7725->pic.border_pos_R[row_bar]-col_bar_R)){
				Ov7725->pos.location_bias = (int)(CNST9*(float)((40-((col_bar_R+Ov7725->pic.border_pos_R[row_bar])/2))));
			}
			else{
				Ov7725->pos.location_bias = (int)(CNST9*(float)(40-((col_bar_L+Ov7725->pic.border_pos_L[row_bar])/2)));
			}
			Weizhi_PID->Kp = SERVO_PID_KP_B;
//			printf("%d, %d, %d\n",row_bar, col_bar_L, col_bar_R);
		}
		else{
			Weizhi_PID->Kp = SERVO_PID_KP_S;
		}
		Ov7725->LOCK = 0;
		return;
	}
	//弯道
	if(Ov7725->mode == 1){
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
		}
		//正常弯道情况
		else if(Ov7725->distance <= THRESHOLD){
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
		}
		tmp_deflection = ((fabsf(tmp_deflection)<MIN_deflection)?0:tmp_deflection);
		Ov7725->pos.deflection = (int)tmp_deflection;
		Weizhi_PID->Kd = ((float)(MotorB->Velosity)/80.0f);
		Weizhi_PID->Kp = SERVO_PID_KP_C;
		StartEndLine->alert = 0;
		Ov7725->LOCK = 0;
		return;
	}
}
#pragma optimize=speed
void ov7725_identify_cross(void)
{
	int col;
	int row;
	uint32	tmp;
	for(row=30;row<45;row++){
		tmp=0;
		for(col=0;col<=9;col++){
			tmp += 0x01 & (OV_pictures_SRAM.pic1_data[row][col]>0);
		}
		if(tmp==0){
			Ov7725->mode = 2;
			break;
		}
	}
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
			//!!!bug!!!//未考虑两边均不存在情况
			Ov7725->mode = Ov7725->pic.exit_L + Ov7725->pic.exit_R;
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
				for(col=0;col<=((CAMERA_W/2)-1);col++){
					if(OV_pictures.pic1_data[59][39-col]==0){
						Ov7725->pic.exit_L = 0;
						Ov7725->pic.exit_R = 1;
						break;
					}
					if(OV_pictures.pic1_data[59][40+col]==0){
						Ov7725->pic.exit_R = 0;
						Ov7725->pic.exit_L = 1;
						break;
					}
				}
				if(Ov7725->pic.exit_L == 0){
					Ov7725->pic.border_pos_L[row] = 80;
					Ov7725->mode = 9;
					return;
				}
				else if(Ov7725->pic.exit_R == 0){
					Ov7725->pic.border_pos_R[row] = 80;
					Ov7725->mode = 6;
					return;
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
			else if(Isall_L_OK!=1){
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
			else if(Isall_R_OK!=1){
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