#include "IMU.h"
#include <stdio.h>
#include <stdint.h>
#include <tm_stm32_mpu6050.h>
#include <math.h>
#include <float.h>
#include "debug.h"

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

#ifndef DESIRED_ACCEL_PRECISION
#define DESIRED_ACCEL_PRECISION TM_MPU6050_Accelerometer_4G
#endif

#ifndef DESIRED_GYRO_PRECISION
#define DESIRED_GYRO_PRECISION TM_MPU6050_Gyroscope_250s
#endif


/* Variable Declarations */
TM_MPU6050_t IMU_datastruct;

/* Private Function Prototypes */

/**
* @brief Returns acceleration for a passed raw accel
*
*
* @param raw_accel The raw accel val in the form of IMU_datastruct.Accelerometer_X
* @return The converted accel in m/s^2
*/
float get_acc(int16_t raw_accel);
/**
* @brief Returns angular accel in rads/sec for a passed raw gyro val
*
*
* @param raw_accel The raw accel val in the form of IMU_datastruct.Gyroscope_X
* @return The converted accel in rads/sec
*/
float get_gyro(int16_t raw_gyro);

/* Function Definitions */

void IMU_init(void) {
    TM_MPU6050_Result_t result = TM_MPU6050_Init(&IMU_datastruct, TM_MPU6050_Device_0, DESIRED_ACCEL_PRECISION, DESIRED_GYRO_PRECISION);
    if (result != TM_MPU6050_Result_Ok) {
        DEBUG("IMU Init Failed: result = %d", (int)result);
    }

}

/**
* @brief Internally captures a new set of IMU measurements
*/
void IMU_read(void) {
    TM_MPU6050_Result_t result = TM_MPU6050_ReadAll(&IMU_datastruct);
    if (result != TM_MPU6050_Result_Ok) {
        DEBUG("IMU Read Failed: result = %d", (int)result);
    }
}

float get_acc(int16_t raw_accel) {
    int8_t min_mapping;
    int8_t max_mapping;

    
    if (DESIRED_ACCEL_PRECISION == TM_MPU6050_Accelerometer_4G) {
        min_mapping = -4;
        max_mapping = 4;
    }
    else {
        printf("IMU accelerometer setting is not handled.");
    }
    
    
    // Convert our int16 input into a ms^-2 output
    return (min_mapping + (((float)(raw_accel - INT16_MIN) / (INT16_MAX - INT16_MIN)) * (max_mapping - min_mapping) )) * 9.81f;
}

float get_accZ(void) {
    static float prevZAcc = FLT_MIN;
    float convertedAcc = get_acc(IMU_datastruct.Accelerometer_Z);    
    
    if (convertedAcc == prevZAcc) {
        IMU_init();
        ;
    }
    else {
        prevZAcc = convertedAcc;
    }

    return convertedAcc;
}

float get_accY(void) {
    return get_acc(IMU_datastruct.Accelerometer_Y);
}

float get_gyro(int16_t raw_gyro) {
    int8_t min_mapping;
    int8_t max_mapping;

    if (DESIRED_GYRO_PRECISION == TM_MPU6050_Gyroscope_250s) {
        // Vals come as deg/sec but we wants rads/sec
        min_mapping = -250*(M_PI/180.0f);
        max_mapping = 250*(M_PI/180.0f);
    }
    else {
        printf("IMU gyro setting is not handled.");
    }
    
    
    // Convert our int16 input into a rads/sec output
    return (min_mapping + (((float)(raw_gyro - INT16_MIN) / (INT16_MAX - INT16_MIN)) * (max_mapping - min_mapping) ));
}

float get_gyroX(void) {
    return get_gyro(IMU_datastruct.Gyroscope_X);
}

double get_acc_angle(void) {
    return -atan2(get_accZ(), get_accY());
}

