#include "ov_cal.h"
#include "OV7725_eagle.h"
#include "math.h"
#include "rectify.h"
extern int16                RectifyX[60][81];
extern int16                RectifyY[60][81];
extern OvTypeDef			*Ov7725;
extern OV_pictureTypeDef_SRAM OV_pictures_SRAM @(OV_binary_ADDR+2);
extern OV_pictureTypeDef OV_pictures @OV_binary_BONDADDR(0, 16);

#define CNST_3		8.0f	//曲率变化灵敏度(只能看到一边时)
#define CNST_4		6.0f	//曲率变化灵敏度(两边都能看到时)(弯道时)
#define CNST_5		10
#define	THRESHOLD	70		//弯道动作距离(cm)
#pragma optimize=speed
void ov7725_cal(void)
{
    int row;
	float tmp_deflection;
	int tmp_location_bias;
	Ov7725->LOCK = 1;
	ov7725_get_border();//沿缘线寻黑边
	ov7725_get_2slope();//逐差法求两边沿线斜率
	//待定的二次关系(为方便调试，暂时用Servo_PID->Kp代替)
	Ov7725->gain = 0.8;
	//车身距左右边界距离
	if(Ov7725->pic.border_pos_R[CAMERA_H-1]==80){
		if(Ov7725->pic.border_pos_L[CAMERA_H-1]==80){
			tmp_location_bias = 0;
		}
		else{
			tmp_location_bias = 0 - Ov7725->pic.border_pos_L[CAMERA_H-1];
		}
	}
	else{
		if(Ov7725->pic.border_pos_L[CAMERA_H-1]==80){
			tmp_location_bias = 80-Ov7725->pic.border_pos_R[CAMERA_H-1];
		}
		else{
			tmp_location_bias = 80-Ov7725->pic.border_pos_R[CAMERA_H-1] - Ov7725->pic.border_pos_L[CAMERA_H-1];
		}
	}
	Ov7725->pos.location_bias = tmp_location_bias;
	//车头距前边界距离
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
	//只存在一条边线情况
	if(!Ov7725->GOODSTATUS){
		//求算一边曲率
		if(Ov7725->pic.exit_L){
			tmp_deflection = -CNST_3*CurvatureABC(RectifyX[Ov7725->pic.start_L][Ov7725->pic.border_pos_L[Ov7725->pic.start_L]], \
					RectifyY[Ov7725->pic.start_L][Ov7725->pic.border_pos_L[Ov7725->pic.start_L]], \
					RectifyX[Ov7725->pic.end_L][Ov7725->pic.border_pos_L[Ov7725->pic.end_L]], \
					RectifyY[Ov7725->pic.end_L][Ov7725->pic.border_pos_L[Ov7725->pic.end_L]], \
					RectifyX[(Ov7725->pic.start_L+Ov7725->pic.end_L)/2][Ov7725->pic.border_pos_L[(Ov7725->pic.start_L+Ov7725->pic.end_L)/2]], \
					RectifyY[(Ov7725->pic.start_L+Ov7725->pic.end_L)/2][Ov7725->pic.border_pos_L[(Ov7725->pic.start_L+Ov7725->pic.end_L)/2]]);
//			tmp_deflection = CNST_3*(((float)(Ov7725->pic.border_pos_L[Ov7725->pic.end_L])-(float)(Ov7725->pic.border_pos_L[Ov7725->pic.start_L]))/((float)(Ov7725->pic.end_L)-(float)(Ov7725->pic.start_L)));
//                        if(Ov7725->distance>50){
//                                        tmp_deflection += CNST_3*(Ov7725->distance - 55);
//                        }
                }
		else if(Ov7725->pic.exit_R){
			tmp_deflection = CNST_3*CurvatureABC(RectifyX[Ov7725->pic.start_R][Ov7725->pic.border_pos_R[Ov7725->pic.start_R]], \
					RectifyY[Ov7725->pic.start_R][Ov7725->pic.border_pos_R[Ov7725->pic.start_R]], \
					RectifyX[Ov7725->pic.end_R][Ov7725->pic.border_pos_R[Ov7725->pic.end_R]], \
					RectifyY[Ov7725->pic.end_R][Ov7725->pic.border_pos_R[Ov7725->pic.end_R]], \
					RectifyX[(Ov7725->pic.start_R+Ov7725->pic.end_R)/2][Ov7725->pic.border_pos_R[(Ov7725->pic.start_R+Ov7725->pic.end_R)/2]], \
					RectifyY[(Ov7725->pic.start_R+Ov7725->pic.end_R)/2][Ov7725->pic.border_pos_R[(Ov7725->pic.start_R+Ov7725->pic.end_R)/2]]);
//			tmp_deflection = CNST_3*(((float)(Ov7725->pic.border_pos_R[Ov7725->pic.start_R])-(float)(Ov7725->pic.border_pos_R[Ov7725->pic.end_R]))/((float)(Ov7725->pic.start_R)-(float)(Ov7725->pic.end_R)));
//                        if(Ov7725->distance>50){
//                                        tmp_deflection -= CNST_3*(Ov7725->distance - 55);
//                        }
                }
		if((Ov7725->pic.exit_L) || (Ov7725->pic.exit_R)){
			if(Ov7725->distance <= 40){ //若距边线大于40cm，则不采用此次计算
				Ov7725->pos.deflection = (int)(((fabsf(Ov7725->pos.deflection-tmp_deflection))<80)?tmp_deflection:Ov7725->pos.deflection);
				Ov7725->LOCK = 0;
				return;
			}
			else{
				Ov7725->pos.deflection = Ov7725->pos.deflection;
				Ov7725->LOCK = 0;
				return;
			}
		}
	}
	//两条边线都存在，弯道情况
	if(Ov7725->distance <= THRESHOLD){
		if((Ov7725->pic.end_L==0)||(Ov7725->pic.end_R==0)){
			;//go on //采用直线策略(焦点求车身偏向)
		}
		else{
			Ov7725->mode = 1;
			if(Ov7725->pic.end_L < Ov7725->pic.end_R){
				tmp_deflection = -CNST_4*CurvatureABC(RectifyX[Ov7725->pic.start_L][Ov7725->pic.border_pos_L[Ov7725->pic.start_L]], \
						RectifyY[Ov7725->pic.start_L][Ov7725->pic.border_pos_L[Ov7725->pic.start_L]], \
						RectifyX[Ov7725->pic.end_L][Ov7725->pic.border_pos_L[Ov7725->pic.end_L]], \
						RectifyY[Ov7725->pic.end_L][Ov7725->pic.border_pos_L[Ov7725->pic.end_L]], \
						RectifyX[(Ov7725->pic.start_L+Ov7725->pic.end_L)/2][Ov7725->pic.border_pos_L[(Ov7725->pic.start_L+Ov7725->pic.end_L)/2]], \
						RectifyY[(Ov7725->pic.start_L+Ov7725->pic.end_L)/2][Ov7725->pic.border_pos_L[(Ov7725->pic.start_L+Ov7725->pic.end_L)/2]]);
			}
			else{
				tmp_deflection = CNST_4*CurvatureABC(RectifyX[Ov7725->pic.start_R][Ov7725->pic.border_pos_R[Ov7725->pic.start_R]], \
						RectifyY[Ov7725->pic.start_R][Ov7725->pic.border_pos_R[Ov7725->pic.start_R]], \
						RectifyX[Ov7725->pic.end_R][Ov7725->pic.border_pos_R[Ov7725->pic.end_R]], \
						RectifyY[Ov7725->pic.end_R][Ov7725->pic.border_pos_R[Ov7725->pic.end_R]], \
						RectifyX[(Ov7725->pic.start_R+Ov7725->pic.end_R)/2][Ov7725->pic.border_pos_R[(Ov7725->pic.start_R+Ov7725->pic.end_R)/2]], \
						RectifyY[(Ov7725->pic.start_R+Ov7725->pic.end_R)/2][Ov7725->pic.border_pos_R[(Ov7725->pic.start_R+Ov7725->pic.end_R)/2]]);
			}
			Ov7725->pos.deflection = (int)(((fabsf(Ov7725->pos.deflection-tmp_deflection))<80)?tmp_deflection:Ov7725->pos.deflection);
			Ov7725->LOCK = 0;
			return;
		}
	}
	else{
		Ov7725->mode = 0;
	}
//  实际斜率
//	tmp_deflection = CNST_3*(((float)(RectifyY[Ov7725->pic.start_R][Ov7725->pic.border_pos_R[Ov7725->pic.start_R]])- \
//				(float)(RectifyY[Ov7725->pic.end_R][Ov7725->pic.border_pos_R[Ov7725->pic.end_R]]))/ \
//				((float)(RectifyX[Ov7725->pic.start_R][Ov7725->pic.border_pos_R[Ov7725->pic.start_R]])- \
//				(float)(RectifyX[Ov7725->pic.end_R][Ov7725->pic.border_pos_R[Ov7725->pic.end_R]])));
	tmp_deflection = 0;
//	tmp_deflection = 40 - (((float)Ov7725->pic.start_L-(float)Ov7725->pic.start_R+ \
//				(float)(Ov7725->calparam.border_slope_R)*Ov7725->pic.border_pos_R[Ov7725->pic.start_R]- \
//				(float)(Ov7725->calparam.border_slope_L)*Ov7725->pic.border_pos_L[Ov7725->pic.start_L])/ \
//				((float)(Ov7725->calparam.border_slope_R)-(float)(Ov7725->calparam.border_slope_L)));
	Ov7725->pos.deflection = (int)(((fabsf(Ov7725->pos.deflection-tmp_deflection))<80)?tmp_deflection:Ov7725->pos.deflection);
	Ov7725->LOCK = 0;
}

#pragma optimize=speed
void ov7725_get_2slope(void)
{
	int row;
	char start_L_OK = 0, start_R_OK = 0, end_L_OK = 0, end_R_OK = 0;
//	int32 x_sum_L = 0, y_sum_L = 0, x_sum_R = 0, y_sum_R = 0;
	Ov7725->pic.start_L = 0;
	Ov7725->pic.end_L = 0;
	Ov7725->pic.start_R = 0;
	Ov7725->pic.end_R = 0;

	for(row=CAMERA_H-1;row>=0;row--)
	{
		if((start_L_OK==0) && (end_L_OK==0) && (row<=(CAMERA_H/2-1))){
			start_L_OK = 1;
			end_L_OK = 1;
			Ov7725->pic.exit_L = 0;
		}
		if((start_R_OK==0) && (end_R_OK==0) && (row<=(CAMERA_H/2-1))){
			start_R_OK = 1;
			end_R_OK = 1;
			Ov7725->pic.exit_R = 0;
		}
		if((start_L_OK==0) && ((Ov7725->pic.border_pos_L[row])!=80)){
			Ov7725->pic.start_L = row;
			start_L_OK = 1;
			Ov7725->pic.exit_L = (Ov7725->pic.start_L==Ov7725->pic.end_L)?0:1;
		}
		if((end_L_OK==0) && ((Ov7725->pic.border_pos_L[CAMERA_H-1-row])!=80)){
			Ov7725->pic.end_L = CAMERA_H-1-row;
			end_L_OK = 1;
			Ov7725->pic.exit_L = (Ov7725->pic.start_L==Ov7725->pic.end_L)?0:1;
		}
		if((start_R_OK==0) && ((Ov7725->pic.border_pos_R[row])!=80)){
			Ov7725->pic.start_R = row;
			start_R_OK = 1;
			Ov7725->pic.exit_R = (Ov7725->pic.start_R==Ov7725->pic.end_R)?0:1;
		}
		if((end_R_OK==0) && ((Ov7725->pic.border_pos_R[CAMERA_H-1-row])!=80)){
			Ov7725->pic.end_R = CAMERA_H-1-row;
			end_R_OK = 1;
			Ov7725->pic.exit_R = (Ov7725->pic.start_R==Ov7725->pic.end_R)?0:1;
		}
		if(start_L_OK&&end_L_OK&&start_R_OK&&end_R_OK)
			break;
	}
	if((Ov7725->pic.exit_L==0) || (Ov7725->pic.exit_R==0)){
		Ov7725->GOODSTATUS = 0;
		Ov7725->mode = Ov7725->pic.exit_L ^ Ov7725->pic.exit_R;
		return;
	}
	else{
		Ov7725->GOODSTATUS = 1;
	}
//	for(row=Ov7725->pic.start_L;row>=((Ov7725->pic.start_L+Ov7725->pic.end_L)>>1);row--)
//	{
//		if(((Ov7725->pic.border_pos_L[Ov7725->pic.start_L-row+Ov7725->pic.end_L])!=80)&&((Ov7725->pic.border_pos_L[row])!=80)){
//			Ov7725->calparam.datum_pointY_L = row;
//			Ov7725->calparam.datum_pointX_L = Ov7725->pic.border_pos_L[row];
//			x_sum_L += Ov7725->pic.border_pos_L[Ov7725->pic.start_L-row+Ov7725->pic.end_L]-Ov7725->pic.border_pos_L[row];
//			y_sum_L += Ov7725->pic.start_L+Ov7725->pic.end_L-row*2;
//		}
//	}
//	for(row=Ov7725->pic.start_R;row>=((Ov7725->pic.end_R+Ov7725->pic.start_R)>>1);row--)
//	{
//		if((Ov7725->pic.border_pos_R[Ov7725->pic.start_R-row+Ov7725->pic.end_R])!=80){
//			if((Ov7725->pic.border_pos_R[row])!=80){
//				Ov7725->calparam.datum_pointY_R = row;
//				Ov7725->calparam.datum_pointX_R = Ov7725->pic.border_pos_R[row];
//				x_sum_R += Ov7725->pic.border_pos_R[Ov7725->pic.start_R-row+Ov7725->pic.end_R]-Ov7725->pic.border_pos_R[row];
//				y_sum_R += Ov7725->pic.start_R+Ov7725->pic.end_R-row*2;
//			}
//		}
//	}
	Ov7725->calparam.border_slope_L = (Ov7725->pic.end_L+2*(Ov7725->pic.start_L-Ov7725->pic.end_L)/3 - Ov7725->pic.start_L)/ \
				(Ov7725->pic.border_pos_L[Ov7725->pic.end_L+2*(Ov7725->pic.start_L-Ov7725->pic.end_L)/3] - \
				Ov7725->pic.border_pos_L[Ov7725->pic.start_L]);
	Ov7725->calparam.border_slope_R = (Ov7725->pic.end_R+2*(Ov7725->pic.start_R-Ov7725->pic.end_R)/3 - Ov7725->pic.start_R)/ \
				(Ov7725->pic.border_pos_R[Ov7725->pic.end_R+2*(Ov7725->pic.start_R-Ov7725->pic.end_R)/3] - \
				Ov7725->pic.border_pos_R[Ov7725->pic.start_R]);
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
	Ov7725->pic.escape_position = CAMERA_H-1;
	for(row=CAMERA_H-1;row>=Ov7725->pic.escape_position;row--)
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
		if((Ov7725->pic.border_pos_L[row]+1)==(Ov7725->pic.border_pos_R[row])){
			Ov7725->GOODSTATUS = 0;
			Ov7725->mode = 1;
			Ov7725->pic.exit_L = (OV_pictures.pic1_data[row][0]>0)&0x01;
			Ov7725->pic.exit_R = (OV_pictures.pic1_data[row][CAMERA_W-1]>0)&0x01;
			Ov7725->pic.border_pos_L[row] = (Ov7725->pic.exit_L)?Ov7725->pic.border_pos_R[row]:80;
			Ov7725->pic.border_pos_R[row] = (Ov7725->pic.exit_R)?Ov7725->pic.border_pos_R[row]:80;
			if(Ov7725->pic.exit_L != Ov7725->pic.exit_R){
				break;
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
		if(Ov7725->pic.escape_position>=80){
			Ov7725->pic.escape_position = Ov7725->pic.escape_position+1;
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
			for(row=CAMERA_H-1;row>=0;row--)
			{
				if(OV_pictures.pic1_data[row][0]>0){
					Ov7725->pic.border_pos_L[row] = 0;
					break;
				}
				else{
					Ov7725->pic.border_pos_L[row] = 80;
				}
			}
			Ov7725->pic.escape_position = row;
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
			}
		}
//		if(Ov7725->GOODSTATUS){
//			for(row=Ov7725->pic.escape_position-1;row>=0;row--)
//			{
//				if((Ov7725->pic.border_pos_R[row]+CNST_1<80)&&((Ov7725->pic.border_pos_R[29])<80)){
//					for(col=(Ov7725->pic.border_pos_R[row])-CNST_1;col>=0;col--)
//					{
//						if((OV_pictures.pic1_data[row][col])>0){
//							Ov7725->pic.border_pos_L[row] = col;
//							break;
//						}
//						else if(col == 0){
//							Ov7725->pic.border_pos_L[row] = 80;
//						}
//					}
//				}
//				else{
//					Ov7725->pic.border_pos_L[row] = 80;
//				}
//			}
//
//		}
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
			for(row=CAMERA_H-1;row>=0;row--)
			{
				if(OV_pictures.pic1_data[row][0]>0){
					Ov7725->pic.border_pos_L[row] = 0;
					break;
				}
				else{
					Ov7725->pic.border_pos_L[row] = 80;
				}
			}
			Ov7725->pic.escape_position = row;
			for(row=Ov7725->pic.escape_position-1;row>=0;row--)
			{
				lie_R = Ov7725->pic.border_pos_R[row+1];
				if(_i>=HorizonShiftMAX){
					Isall_R_OK = 1;
				}
				if(Isall_R_OK == 0){
					if((OV_pictures.pic1_data[row][lie_R])>0){
						if(((lie_L+_i)>=CAMERA_W)||((OV_pictures.pic1_data[row][lie_R+_i])>0)){
							if(((lie_L-_i)<0)||((OV_pictures.pic1_data[row][lie_R-_i])>0)){
								if(((lie_L+_i)>=CAMERA_W)||((OV_pictures.pic1_data[row+1][lie_R+_i])>0)){
									if(((lie_L-_i)<0)||((OV_pictures.pic1_data[row+1][lie_R-_i])>0)){
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
					else if(((lie_L+_i)<CAMERA_W)&&(OV_pictures.pic1_data[row][lie_R+_i])>0){
						Ov7725->pic.border_pos_R[row] = lie_R+_i;
						_i = 1;
					}
					else if(((lie_L-_i)>=0)&&(OV_pictures.pic1_data[row][lie_R-_i])>0){
						Ov7725->pic.border_pos_R[row] = lie_R-_i;
						_i = 1;
					}
					else if(((lie_L+_i)<CAMERA_W)&&(OV_pictures.pic1_data[row+1][lie_R+_i])>0){
						_i++;
						row++;
					}
					else if(((lie_L-_i)>=0)&&(OV_pictures.pic1_data[row+1][lie_R-_i])>0){
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
		}
//		if(Ov7725->GOODSTATUS){
//			for(row=Ov7725->pic.escape_position-1;row>=0;row--)
//			{
//				if((Ov7725->pic.border_pos_L[row]+CNST_1<80)&&((Ov7725->pic.border_pos_L[29])<80)){
//					for(col=(Ov7725->pic.border_pos_L[row])+CNST_1;col<=79;col++)
//					{
//						if((OV_pictures.pic1_data[row][col])>0){
//							Ov7725->pic.border_pos_R[row] = col;
//							break;
//						}
//						else if(col == 79){
//							Ov7725->pic.border_pos_R[row] = 80;
//						}
//					}
//				}
//				else{
//					Ov7725->pic.border_pos_R[row] = 80;
//				}
//			}
//		}
	}
}