#ifndef BMP280_H
#define BMP280_H

#include "main.h"

// L'adresse I2C (0x76) décalée d'un bit vers la gauche pour le HAL
#define BMP280_I2C_ADDR 0xEC

void BMP280_Init(void);
float BMP280_ReadTemperature(void);
float BMP280_ReadPressure(void);

#endif
