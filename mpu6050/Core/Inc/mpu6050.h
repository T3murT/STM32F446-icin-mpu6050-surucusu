/*
 * mpu6050.h
 *
 *  Created on: Jun 29, 2025
 *      Author: Temur
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_


#include "stm32f4xx_hal.h"
#include <math.h>
#include <stdint.h>

// Kalman filtre yapısı
typedef struct {
    float Q_angle;
    float Q_bias;
    float R_measure;

    float angle;
    float bias;
    float rate;

    float P[2][2];
} Kalman_t;

// MPU6050 register adresleri
#define MPU6050_ADDR         (0x68 << 1)
#define MPU6050_GYRO_CONFIG  0x1B
#define MPU6050_ACCE_CONFIG  0x1C
#define MPU6050_ACCE_MESURE  0x3B
#define MPU6050_GYRO_MESURE  0x43
#define MPU6050_POWER_1      0x6B
#define MPU6050_CONFIG       0x1A

extern I2C_HandleTypeDef hi2c1;

// MPU6050 başlatma ve kalibrasyon
void TT_Init_MPU6050(void);
void TT_Get_Gyro_Cal_Values(void);
void TT_Kalman_Init(void);

// Açı değerlerini alma
float TT_Get_Pitch_Angle(void);
float TT_Get_Roll_Angle(void);
float TT_Get_Yaw_Angle(void);

// Kalman filtre fonksiyonları
void Kalman_init(Kalman_t *Kalman);
void Kalman_setAngle(Kalman_t *Kalman, float angle);
float Kalman_getAngle(Kalman_t *Kalman, float newAngle, float newRate, float dt);



#endif /* INC_MPU6050_H_ */
