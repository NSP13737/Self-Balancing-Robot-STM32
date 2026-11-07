#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

// motor.h
typedef enum {
    MOTOR_LEFT,
    MOTOR_RIGHT
} Motor;

typedef enum {
    MOTOR_DIR_FWD,
    MOTOR_DIR_BWD,
    MOTOR_DIR_NODIR
} MotorDirection;

void motor_set_direction(Motor motor, MotorDirection direction);
void motor_set_dc(Motor motor, int percent_dc);
int motor_get_dc(Motor motor);
float motor_get_voltage(Motor motor);
void setLeftMotorDC(int percent_dc);
void setRightMotorDC(int percent_dc);

void left_motor_PWM_init(void);
void left_motor_direction_init(void);
void left_motor_fwd_en(void);
void left_motor_bwd_en(void);
void left_motor_no_dir(void);
void left_motor_encoder_init(void);
void EXTI4_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
int32_t left_motor_encoder_getValue(void);
void motor_velocity_set_counts_per_wheel_rev(float counts_per_wheel_rev);
void motor_velocity_init(void);
float left_motor_velocity_rad_s(void);
float right_motor_velocity_rad_s(void);

void right_motor_PWM_init(void);
void right_motor_direction_init(void);
void right_motor_fwd_en(void);
void right_motor_bwd_en(void);
void right_motor_no_dir(void);
void right_motor_encoder_init(void);
void EXTI15_10_IRQHandler(void);
int32_t right_motor_encoder_getValue(void);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#endif
