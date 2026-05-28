#ifndef SERVO_PWM_H
#define SERVO_PWM_H

#include "stm32l4xx.h"

void PWM_Servo_Init(void);
void Mettre_A_Jour_Servomoteur(float angle_lacet);

#endif
