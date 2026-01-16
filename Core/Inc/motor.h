#ifndef _MOTOR_H
#define _MOTOR_H

#include "stm32f1xx_hal.h"
void Load(int moto1,int moto2);
void Limit(int *motoA,int *motoB);
void Stop(float current_pitch);
int abs(int p);
extern int MOTO1,MOTO2;
#endif
