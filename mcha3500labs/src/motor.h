#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>


void left_motor_PWM_init(void);
void left_motor_encoder_init(void);
void EXTI4_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
int32_t left_motor_encoder_getValue(void);

void right_motor_PWM_init(void);
void right_motor_encoder_init(void);
void EXTI15_10_IRQHandler(void);
int32_t right_motor_encoder_getValue(void);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#endif
