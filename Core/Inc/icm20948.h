#ifndef ICM20948_H
#define ICM20948_H

#include "main.h"

// L'adresse I2C (0x69) décalée d'un bit vers la gauche pour la fonction HAL
#define ICM_I2C_ADDR 0xD2

void ICM_Init(void);
void ICM_ReadAngles(float *roulis, float *tangage, float *lacet);

#endif




