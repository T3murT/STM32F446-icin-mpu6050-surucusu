/*
 * mpu6050.c
 *
 *  Created on: Jun 29, 2025
 *      Author: Temur
 */


#include "mpu6050.h"


void Kalman_init(Kalman_t *Kalman) {
    Kalman->Q_angle = 0.05f;
    Kalman->Q_bias = 0.001f;
    Kalman->R_measure = 0.5f;
    Kalman->angle = 0.0f;
    Kalman->bias = 0.0f;
    Kalman->P[0][0] = Kalman->P[0][1] = 0.0f;
    Kalman->P[1][0] = Kalman->P[1][1] = 0.0f;
}

float Kalman_getAngle(Kalman_t *Kalman, float newAngle, float newRate, float dt) {
    Kalman->rate = newRate - Kalman->bias;
    Kalman->angle += dt * Kalman->rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    float S = Kalman->P[0][0] + Kalman->R_measure;
    float K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    float y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias  += K[1] * y;

    float P00_temp = Kalman->P[0][0];
    float P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
}

void Kalman_setAngle(Kalman_t *Kalman, float angle) {
    Kalman->angle = angle;
}


static uint8_t Gyro_Buffer[6], Acce_Buffer[6];
static int16_t Gyro_Raw_X, Gyro_Raw_Y, Gyro_Raw_Z;
static int16_t Acce_Raw_X, Acce_Raw_Y, Acce_Raw_Z;

static float Gyro_Cal_X = 0, Gyro_Cal_Y = 0, Gyro_Cal_Z = 0;
static float kalman_pitch = 0, kalman_roll = 0, angle_yaw_gyro = 0;

static Kalman_t kalman_filter_pitch, kalman_filter_roll;

static uint32_t last_tick = 0;

void TT_Init_MPU6050(void) {
    uint8_t data;

    data = 0x00; HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_POWER_1, 1, &data, 1, 100);
    data = 0x04; HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_CONFIG, 1, &data, 1, 100);
    data = 0x10; HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_ACCE_CONFIG, 1, &data, 1, 100);
    data = 0x08; HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_GYRO_CONFIG, 1, &data, 1, 100);
}

void TT_Get_Gyro_Cal_Values(void) {
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    int16_t gx, gy, gz;
    uint8_t buf[6];

    for (int i = 0; i < 2000; i++) {
        HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_GYRO_MESURE, 1, buf, 6, HAL_MAX_DELAY);
        gx = (buf[0] << 8) | buf[1];
        gy = (buf[2] << 8) | buf[3];
        gz = (buf[4] << 8) | buf[5];
        sum_x += gx; sum_y += gy; sum_z += gz;
        HAL_Delay(3);
    }

    Gyro_Cal_X = sum_x / 2000.0f;
    Gyro_Cal_Y = sum_y / 2000.0f;
    Gyro_Cal_Z = sum_z / 2000.0f;
}

void TT_Kalman_Init(void) {
    Kalman_init(&kalman_filter_pitch);
    Kalman_init(&kalman_filter_roll);

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_ACCE_MESURE, 1, Acce_Buffer, 6, HAL_MAX_DELAY);
    Acce_Raw_X = (Acce_Buffer[0] << 8) | Acce_Buffer[1];
    Acce_Raw_Y = (Acce_Buffer[2] << 8) | Acce_Buffer[3];
    Acce_Raw_Z = (Acce_Buffer[4] << 8) | Acce_Buffer[5];

    float roll  = atan2f((float)Acce_Raw_Y, (float)Acce_Raw_Z) * (180.0f / M_PI);
    float pitch = atanf(-(float)Acce_Raw_X / sqrtf(Acce_Raw_Y * Acce_Raw_Y + Acce_Raw_Z * Acce_Raw_Z)) * (180.0f / M_PI);

    Kalman_setAngle(&kalman_filter_pitch, pitch);
    Kalman_setAngle(&kalman_filter_roll, roll);

    last_tick = HAL_GetTick();
}

static void TT_Process_All_Values(void) {
    float dt = (HAL_GetTick() - last_tick) / 1000.0f;
    if (dt <= 0) dt = 0.001f;
    last_tick = HAL_GetTick();

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_GYRO_MESURE, 1, Gyro_Buffer, 6, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_ACCE_MESURE, 1, Acce_Buffer, 6, HAL_MAX_DELAY);

    Gyro_Raw_X = (Gyro_Buffer[0] << 8) | Gyro_Buffer[1];
    Gyro_Raw_Y = (Gyro_Buffer[2] << 8) | Gyro_Buffer[3];
    Gyro_Raw_Z = (Gyro_Buffer[4] << 8) | Gyro_Buffer[5];

    Acce_Raw_X = (Acce_Buffer[0] << 8) | Acce_Buffer[1];
    Acce_Raw_Y = (Acce_Buffer[2] << 8) | Acce_Buffer[3];
    Acce_Raw_Z = (Acce_Buffer[4] << 8) | Acce_Buffer[5];

    float gyro_pitch = ((float)Gyro_Raw_X - Gyro_Cal_X) / 65.5f;
    float gyro_roll  = ((float)Gyro_Raw_Y - Gyro_Cal_Y) / 65.5f;
    float gyro_yaw   = ((float)Gyro_Raw_Z - Gyro_Cal_Z) / 65.5f;

    float acc_roll  = atan2f((float)Acce_Raw_Y, (float)Acce_Raw_Z) * (180.0f / M_PI);
    float acc_pitch = atanf(-(float)Acce_Raw_X / sqrtf(Acce_Raw_Y * Acce_Raw_Y + Acce_Raw_Z * Acce_Raw_Z)) * (180.0f / M_PI);

    kalman_pitch = Kalman_getAngle(&kalman_filter_pitch, acc_pitch, gyro_pitch, dt);
    kalman_roll  = Kalman_getAngle(&kalman_filter_roll, acc_roll, gyro_roll, dt);

    angle_yaw_gyro += gyro_yaw * dt;
}

float TT_Get_Pitch_Angle(void) {
    TT_Process_All_Values();
    return kalman_pitch;
}

float TT_Get_Roll_Angle(void) {
    return kalman_roll;
}

float TT_Get_Yaw_Angle(void) {
    return angle_yaw_gyro;
}
