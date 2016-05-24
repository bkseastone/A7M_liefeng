#include "ov_cal.h"
#include "OV7725_eagle.h"
#include "math.h"
extern OvTypeDef			*Ov7725;
extern OV_pictureTypeDef_SRAM OV_pictures_SRAM @(OV_binary_ADDR+2);
extern OV_pictureTypeDef OV_pictures @OV_binary_BONDADDR(0, 16);
#pragma optimize=speed
void ov7725_cal(void)
{
	int tnp_deflection;
	int tnp_location_bias;
	ov7725_get_border();//沿缘线寻黑边
	ov7725_get_slope();//逐差法求斜率
	if(!Ov7725->GOODSTATUS) return;
	Ov7725->calparam.last_point_L = Ov7725->pic.border_pos_L[CAMERA_H-1];
	Ov7725->calparam.last_point_R = Ov7725->pic.border_pos_R[CAMERA_H-1];
	if((Ov7725->calparam.last_point_L)==100)
		Ov7725->calparam.last_point_L = (int)((((int)(CAMERA_H-1-Ov7725->calparam.datum_pointY_L))/(Ov7725->calparam.border_slope_L)) + Ov7725->calparam.datum_pointX_L);
	if((Ov7725->calparam.last_point_R)==100)
		Ov7725->calparam.last_point_R = (int)((((int)(CAMERA_H-1-Ov7725->calparam.datum_pointY_R))/(Ov7725->calparam.border_slope_R)) + Ov7725->calparam.datum_pointX_R);
	tnp_deflection = 40 - (((int)Ov7725->calparam.datum_pointY_L-(int)Ov7725->calparam.datum_pointY_R+ \
				(int)(Ov7725->calparam.border_slope_R)*Ov7725->calparam.datum_pointX_R- \
				(int)(Ov7725->calparam.border_slope_L)*Ov7725->calparam.datum_pointX_L)/ \
				((int)(Ov7725->calparam.border_slope_R)-(int)(Ov7725->calparam.border_slope_L)));
	Ov7725->pos.deflection = ((fabsf(Ov7725->pos.deflection-tnp_deflection))<80)?tnp_deflection:Ov7725->pos.deflection;
	tnp_location_bias = 40 - (int)(((Ov7725->calparam.last_point_L)+(Ov7725->calparam.last_point_R))>>1) - (Ov7725->pos.deflection);
	tnp_location_bias = (fabsf(tnp_location_bias)<=40)?tnp_location_bias:40;
	Ov7725->pos.location_bias = ((fabsf(Ov7725->pos.location_bias-tnp_location_bias))<80)?tnp_location_bias:Ov7725->pos.location_bias;
	//待定的二次关系
	Ov7725->gain = 0.6;
}

#pragma optimize=speed
void ov7725_get_slope(void)
{
	int row;
	char start_L_OK = 0, start_R_OK = 0, end_L_OK = 0, end_R_OK = 0;
	int32 x_sum_L = 0, y_sum_L = 0, x_sum_R = 0, y_sum_R = 0;
	Ov7725->pic.start_L = 0;
	Ov7725->pic.end_L = 0;
	Ov7725->pic.start_R = 0;
	Ov7725->pic.end_R = 0;
	if(Ov7725->GOODSTATUS){
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
			if((start_L_OK==0) && ((Ov7725->pic.border_pos_L[row])!=100)){
				Ov7725->pic.start_L = row;
				start_L_OK = 1;
				Ov7725->pic.exit_L = (Ov7725->pic.start_L==Ov7725->pic.end_L)?0:1;
			}
			if((end_L_OK==0) && ((Ov7725->pic.border_pos_L[CAMERA_H-1-row])!=100)){
				Ov7725->pic.end_L = CAMERA_H-1-row;
				end_L_OK = 1;
				Ov7725->pic.exit_L = (Ov7725->pic.start_L==Ov7725->pic.end_L)?0:1;
			}
			if((start_R_OK==0) && ((Ov7725->pic.border_pos_R[row])!=100)){
				Ov7725->pic.start_R = row;
				start_R_OK = 1;
				Ov7725->pic.exit_R = (Ov7725->pic.start_R==Ov7725->pic.end_R)?0:1;
			}
			if((end_R_OK==0) && ((Ov7725->pic.border_pos_R[CAMERA_H-1-row])!=100)){
				Ov7725->pic.end_R = CAMERA_H-1-row;
				end_R_OK = 1;
				Ov7725->pic.exit_R = (Ov7725->pic.start_R==Ov7725->pic.end_R)?0:1;
			}
			if(start_L_OK&&end_L_OK&&start_R_OK&&end_R_OK)
				break;
		}

	}
	if((Ov7725->pic.exit_L==0) || (Ov7725->pic.exit_R==0)){
		Ov7725->GOODSTATUS = 0;
		Ov7725->mode = Ov7725->pic.exit_L || Ov7725->pic.exit_R;
		return;
	}
	for(row=Ov7725->pic.start_L;row>=((Ov7725->pic.start_L+Ov7725->pic.end_L)>>1);row--)
	{
		if(((Ov7725->pic.border_pos_L[Ov7725->pic.start_L-row+Ov7725->pic.end_L])!=100)&&((Ov7725->pic.border_pos_L[row])!=100)){
			Ov7725->calparam.datum_pointY_L = row;
			Ov7725->calparam.datum_pointX_L = Ov7725->pic.border_pos_L[row];
			x_sum_L += Ov7725->pic.border_pos_L[Ov7725->pic.start_L-row+Ov7725->pic.end_L]-Ov7725->pic.border_pos_L[row];
			y_sum_L += Ov7725->pic.start_L+Ov7725->pic.end_L-row*2;
		}
	}
	for(row=Ov7725->pic.start_R;row>=((Ov7725->pic.end_R+Ov7725->pic.start_R)>>1);row--)
	{
		if((Ov7725->pic.border_pos_R[Ov7725->pic.start_R-row+Ov7725->pic.end_R])!=100){
			if((Ov7725->pic.border_pos_R[row])!=100){
				Ov7725->calparam.datum_pointY_R = row;
				Ov7725->calparam.datum_pointX_R = Ov7725->pic.border_pos_R[row];
				x_sum_R += Ov7725->pic.border_pos_R[Ov7725->pic.start_R-row+Ov7725->pic.end_R]-Ov7725->pic.border_pos_R[row];
				y_sum_R += Ov7725->pic.start_R+Ov7725->pic.end_R-row*2;
			}
		}
	}
	Ov7725->calparam.border_slope_L = ((float)(y_sum_L))/((float)(x_sum_L));
	Ov7725->calparam.border_slope_R = ((float)(y_sum_R))/((float)(x_sum_R));
}

#define HorizonShiftMAX	10	//相邻两行边缘点最大横向偏移量
#define CNST_1				2	//自一边缘线向另一边缘线搜索的起始偏移量
#pragma optimize=speed
void ov7725_get_border(void)
{
	int row, col;
	int lie_L, lie_R;
	char Is_L_OK = 0, Is_R_OK = 0;
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
			Ov7725->pic.border_pos_L[row] = 100; //100意味木有
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
			Ov7725->pic.border_pos_R[row] = 100;
		}
		//若第一行39列和40列都为黑边的处理方案
//		if((Ov7725->pic.border_pos_L[row]+1)==(Ov7725->pic.border_pos_R[row])){
//			Ov7725->GOODSTATUS = 0;
//			Ov7725->pic.exit_L = OV_pictures.pic1_data[row][];
//		}
		if((((Ov7725->pic.border_pos_L[Ov7725->pic.escape_position]) != 100)&&    \
			((Ov7725->pic.border_pos_R[Ov7725->pic.escape_position]) != 100))  || \
			(((Ov7725->pic.border_pos_L[Ov7725->pic.escape_position]) == 100)&&   \
			((Ov7725->pic.border_pos_R[Ov7725->pic.escape_position]) == 100)))
		{
			Ov7725->pic.escape_position--;
		}
		if(Ov7725->pic.escape_position>=80){
			Ov7725->pic.escape_position = Ov7725->pic.escape_position+1;
			return;
		}
	}
	Is_R_OK = 0;
	Is_L_OK = 0;
	_i = 1;
	if(((Ov7725->pic.border_pos_L[Ov7725->pic.escape_position]) == 100) && \
		((Ov7725->pic.border_pos_R[Ov7725->pic.escape_position]) != 100))
	{
		for(row=Ov7725->pic.escape_position-1;row>=0;row--)
		{
			lie_R = Ov7725->pic.border_pos_R[row+1];
			lie_R = ((lie_R+_i)>=CAMERA_W)?(CAMERA_W-1-_i):lie_R;
			lie_R = ((lie_R-_i)<0)?(0-_i):lie_R;
			if(_i>=HorizonShiftMAX){
				Is_R_OK = 1;
			}
			if(Is_R_OK == 0){
				if((OV_pictures.pic1_data[row][lie_R])>0){
					if((OV_pictures.pic1_data[row][lie_R+_i])>0){
						if((OV_pictures.pic1_data[row][lie_R-_i])>0){
							if((OV_pictures.pic1_data[row+1][lie_R+_i])>0){
								if((OV_pictures.pic1_data[row+1][lie_R-_i])>0){
									_i++;
									row++;
								}
								else{
									Ov7725->pic.border_pos_R[row] = lie_R-_i;
									_i = 1;
								}
							}
							else{
								Ov7725->pic.border_pos_R[row] = lie_R+_i;
								_i = 1;
							}
						}
						else{
							Ov7725->pic.border_pos_R[row] = lie_R;
							_i = 1;
						}
					}
					else{
						Ov7725->pic.border_pos_R[row] = lie_R;
						_i = 1;
					}
				}
				else if((OV_pictures.pic1_data[row][lie_R+_i])>0){
					Ov7725->pic.border_pos_R[row] = lie_R+_i;
					_i = 1;
				}
				else if((OV_pictures.pic1_data[row][lie_R-_i])>0){
					Ov7725->pic.border_pos_R[row] = lie_R-_i;
					_i = 1;
				}
				else if((OV_pictures.pic1_data[row+1][lie_R+_i])>0){
					_i++;
					row++;
				}
				else if((OV_pictures.pic1_data[row+1][lie_R-_i])>0){
					_i++;
					row++;
				}
				else{
					Ov7725->pic.border_pos_R[row] = 100;
					Is_R_OK = 1;
				}
			}
			else{
				Ov7725->pic.border_pos_R[row] = 100;
			}
		}
		for(row=Ov7725->pic.escape_position-1;row>=0;row--)
		{
			for(col=(Ov7725->pic.border_pos_R[row])-CNST_1;col>=0;col--)
			{
				if(col != (100-CNST_1)){
					if((OV_pictures.pic1_data[row][col])>0){
						Ov7725->pic.border_pos_L[row] = col;
						break;
					}
					else if(col == 0){
						Ov7725->pic.border_pos_L[row] = 100;
					}
				}
				else{
					Ov7725->pic.border_pos_L[row] = 100;
				}
			}
		}
	}
	else if(((Ov7725->pic.border_pos_R[Ov7725->pic.escape_position]) == 100) && \
			((Ov7725->pic.border_pos_L[Ov7725->pic.escape_position]) != 100))
	{
		for(row=Ov7725->pic.escape_position-1;row>=0;row--)
		{
			lie_L = Ov7725->pic.border_pos_L[row+1];
			lie_L = ((lie_L+_i)>=CAMERA_W)?(CAMERA_H-1-_i):lie_L;
			lie_L = ((lie_L-_i)<0)?(0-_i):lie_L;
			if(_i>=HorizonShiftMAX){
				Is_L_OK = 1;
			}
			if(Is_L_OK == 0){
				if((OV_pictures.pic1_data[row][lie_L])>0){
					if((OV_pictures.pic1_data[row][lie_L+_i])>0){
						if((OV_pictures.pic1_data[row][lie_L-_i])>0){
							if((OV_pictures.pic1_data[row+1][lie_L+_i])>0){
								if((OV_pictures.pic1_data[row+1][lie_L-_i])>0){
									_i++;
									row++;
								}
								else{
									Ov7725->pic.border_pos_L[row] = lie_L-_i;
									_i = 1;
								}
							}
							else{
								Ov7725->pic.border_pos_L[row] = lie_L+_i;
								_i = 1;
							}
						}
						else{
							Ov7725->pic.border_pos_L[row] = lie_L;
							_i = 1;
						}
					}
					else{
						Ov7725->pic.border_pos_L[row] = lie_L;
						_i = 1;
					}
				}
				else if((OV_pictures.pic1_data[row][lie_L+_i])>0){
					Ov7725->pic.border_pos_L[row] = lie_L+_i;
					_i = 1;
				}
				else if((OV_pictures.pic1_data[row][lie_L-_i])>0){
					Ov7725->pic.border_pos_L[row] = lie_L-_i;
					_i = 1;
				}
				else if((OV_pictures.pic1_data[row+1][lie_L+_i])>0){
					_i++;
					row++;
				}
				else if((OV_pictures.pic1_data[row+1][lie_L-_i])>0){
					_i++;
					row++;
				}
				else{
					Ov7725->pic.border_pos_L[row] = 100;
					Is_L_OK = 1;
				}
			}
			else{
				Ov7725->pic.border_pos_L[row] = 100;
			}
		}
		for(row=Ov7725->pic.escape_position-1;row>=0;row--)
		{
			for(col=(Ov7725->pic.border_pos_L[row])+CNST_1;col<=79;col++)
			{
				if(col != (100+CNST_1)){
					if((OV_pictures.pic1_data[row][col])>0){
						Ov7725->pic.border_pos_R[row] = col;
						break;
					}
					else if(col == 79){
						Ov7725->pic.border_pos_R[row] = 100;
					}
				}
				else{
					Ov7725->pic.border_pos_R[row] = 100;
				}
			}
		}
	}
}