#include "bmp280.h"

extern I2C_HandleTypeDef hi2c1;

/* --------------------------------------------------------------------------
 * FP1 – Variables de calibration d'usine (BMP280)
 * -------------------------------------------------------------------------- */
uint16_t dig_T1; int16_t dig_T2, dig_T3;
uint16_t dig_P1; int16_t dig_P2, dig_P3, dig_P4, dig_P5,
                          dig_P6, dig_P7, dig_P8, dig_P9;
int32_t t_fine;

/* --------------------------------------------------------------------------
 * FP1 – Initialisation et lecture des coefficients de calibration
 * -------------------------------------------------------------------------- */
void BMP280_Init(void) {
    uint8_t calib[24];
    HAL_I2C_Mem_Read(&hi2c1, BMP280_I2C_ADDR, 0x88, 1, calib, 24, 100);

    dig_T1 = (calib[1]  << 8) | calib[0];  dig_T2 = (calib[3]  << 8) | calib[2];
    dig_T3 = (calib[5]  << 8) | calib[4];  dig_P1 = (calib[7]  << 8) | calib[6];
    dig_P2 = (calib[9]  << 8) | calib[8];  dig_P3 = (calib[11] << 8) | calib[10];
    dig_P4 = (calib[13] << 8) | calib[12]; dig_P5 = (calib[15] << 8) | calib[14];
    dig_P6 = (calib[17] << 8) | calib[16]; dig_P7 = (calib[19] << 8) | calib[18];
    dig_P8 = (calib[21] << 8) | calib[20]; dig_P9 = (calib[23] << 8) | calib[22];

    uint8_t ctrl_meas = 0x27;
    HAL_I2C_Mem_Write(&hi2c1, BMP280_I2C_ADDR, 0xF4, 1, &ctrl_meas, 1, 100);
}

/* --------------------------------------------------------------------------
 * FP1 – Algorithme de compensation Bosch (température + pression)
 * -------------------------------------------------------------------------- */
float BMP280_ReadTemperature(void) {
    uint8_t data[3];
    HAL_I2C_Mem_Read(&hi2c1, BMP280_I2C_ADDR, 0xFA, 1, data, 3, 100);
    int32_t adc_T = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);

    int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) *
                      ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) *
                    ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    return ((t_fine * 5 + 128) >> 8) / 100.0f;
}

float BMP280_ReadPressure(void) {
    BMP280_ReadTemperature();
    uint8_t data[3];
    HAL_I2C_Mem_Read(&hi2c1, BMP280_I2C_ADDR, 0xF7, 1, data, 3, 100);
    int32_t adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);

    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;
    if (var1 == 0) return 0;
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
    return (float)p / 25600.0f;
}
