#include "PID.h"

PIDTypeDef Motor_PID_struct = {PID_KP, PID_KI, PID_KD, 
						PID_NOWE, PID_LASTE, PID_PASTE, 
						PID_OUT}; // 电机PID
PIDTypeDef		*Motor_PID = &Motor_PID_struct;
int32 PID_cal(PIDTypeDef* PID)
{
	PID->out = (int32)(PID->kp*(PID->nowe-PID->laste + PID->ki*PID->nowe + PID->kd*(PID->nowe-2*PID->laste+PID->paste)));
	PID->paste = PID->laste;
	PID->laste = PID->nowe;
	return PID->out;
}
