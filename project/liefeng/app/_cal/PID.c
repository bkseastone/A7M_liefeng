#include "PID.h"

int32 PID_cal(PIDTypeDef* PID)
{
	PID->out = (int32)(PID->kp*(PID->nowe-PID->laste + PID->ki*PID->nowe + PID->kd*(PID->nowe-2*PID->laste+PID->paste)));
	PID->paste = PID->laste;
	PID->laste = PID->nowe;
	return PID->out;
}
