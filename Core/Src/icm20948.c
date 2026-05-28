#include "icm20948.h"
#include <math.h>

extern I2C_HandleTypeDef hi2c1;

/* --------------------------------------------------------------------------
 * FP1/FP4 – Variables d'état du gyroscope
 * -------------------------------------------------------------------------- */
float lacet_cumule  = 0.0f;
float erreur_gyro_z = 0.0f;

/* --------------------------------------------------------------------------
 * FP1 – Calibration au démarrage (offset gyroscopique)
 * -------------------------------------------------------------------------- */
void ICM_Init(void) {
    uint8_t d = 0x01;
    HAL_I2C_Mem_Write(&hi2c1, ICM_I2C_ADDR, 0x06, 1, &d, 1, 100);
    HAL_Delay(200);

    int32_t somme_gz = 0;
    uint8_t data[2];
    for (int i = 0; i < 200; i++) {
        HAL_I2C_Mem_Read(&hi2c1, ICM_I2C_ADDR, 0x37, 1, data, 2, 100);
        somme_gz += (int16_t)((data[0] << 8) | data[1]);
        HAL_Delay(5);
    }
    erreur_gyro_z = (float)somme_gz / 200.0f;
    lacet_cumule  = 0.0f;
}

/* --------------------------------------------------------------------------
 * FP1/FP4 – Lecture des angles (accéléromètre + intégration gyroscope Z)
 * -------------------------------------------------------------------------- */
void ICM_ReadAngles(float *roulis, float *tangage, float *lacet) {
    uint8_t data[6];

    /* Roulis et tangage via accéléromètre */
    HAL_I2C_Mem_Read(&hi2c1, ICM_I2C_ADDR, 0x2D, 1, data, 6, 100);
    int16_t ax = (data[0] << 8) | data[1];
    int16_t ay = (data[2] << 8) | data[3];
    int16_t az = (data[4] << 8) | data[5];
    *roulis  = atan2f(ay, az)                        * 180.0f / 3.14159265f;
    *tangage = atan2f(-ax, sqrtf(ay*ay + az*az))     * 180.0f / 3.14159265f;

    /* Lacet via intégration gyroscope Z */
    HAL_I2C_Mem_Read(&hi2c1, ICM_I2C_ADDR, 0x37, 1, data, 2, 100);
    int16_t gz = (data[0] << 8) | data[1];

    float vitesse_z = (gz - erreur_gyro_z) / 131.0f;
    if (vitesse_z > -0.1f && vitesse_z < 0.1f) vitesse_z = 0.0f;

    static uint32_t last_time = 0;
    uint32_t now = HAL_GetTick();
    float dt = (now - last_time) / 1000.0f;
    if (dt > 1.0f || dt <= 0.0f) dt = 0.2f;
    last_time = now;

    lacet_cumule += vitesse_z * dt;
    if (lacet_cumule >  45.0f) lacet_cumule =  45.0f;
    if (lacet_cumule < -45.0f) lacet_cumule = -45.0f;

    *lacet = lacet_cumule;
}
