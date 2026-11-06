#include "motor.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include <stdio.h>

static uint8_t _left_motor_is_init = 0;
static uint8_t _right_motor_is_init = 0;

static TIM_HandleTypeDef htim3;

//TODO: configure for left motor
void left_motor_PWM_init(void) {
    // uses pa6 and tim_channel_1
    if (_left_motor_is_init) {
        return;
    } 
    _left_motor_is_init = 1;

    __TIM3_CLK_ENABLE();
    __GPIOA_CLK_ENABLE();

    // configure pins in AF mode
    GPIO_InitTypeDef gpio_config;
    gpio_config.Pin = GPIO_PIN_6;
    gpio_config.Mode = GPIO_MODE_AF_PP;
    gpio_config.Pull = GPIO_NOPULL;
    gpio_config.Speed = GPIO_SPEED_FREQ_LOW;
    gpio_config.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA,&gpio_config);

    // Set up timer in PWM mode
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = TIM_CLOCKPRESCALER_DIV1;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    // \frac{\left(180\cdot10^{6}\right)}{10\cdot10^{3}}-1
    htim3.Init.Period = 9999;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim3);

    // config timer more
    TIM_OC_InitTypeDef tim3_PWM;
    tim3_PWM.OCMode = TIM_OCMODE_PWM1;
    tim3_PWM.Pulse = 0;
    tim3_PWM.OCPolarity = TIM_OCPOLARITY_HIGH;
    tim3_PWM.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &tim3_PWM, TIM_CHANNEL_1);

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2500);

    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);

    left_motor_direction_init();
    left_motor_no_dir();
}

//TODO: configure for right motor
void right_motor_PWM_init(void) {
    // uses pa7 and tim_channel_2
    if (_right_motor_is_init) {
        return;
    } 
    _right_motor_is_init = 1;

    __TIM3_CLK_ENABLE();
    __GPIOA_CLK_ENABLE();

    // configure pins in AF mode
    GPIO_InitTypeDef gpio_config;
    gpio_config.Pin = GPIO_PIN_7;
    gpio_config.Mode = GPIO_MODE_AF_PP;
    gpio_config.Pull = GPIO_NOPULL;
    gpio_config.Speed = GPIO_SPEED_FREQ_LOW;
    gpio_config.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA,&gpio_config);

    // Set up timer in PWM mode
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = TIM_CLOCKPRESCALER_DIV1;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    // \frac{\left(180\cdot10^{6}\right)}{10\cdot10^{3}}-1
    htim3.Init.Period = 9999;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim3);

    // config timer more
    TIM_OC_InitTypeDef tim3_PWM;
    tim3_PWM.OCMode = TIM_OCMODE_PWM1;
    tim3_PWM.Pulse = 0;
    tim3_PWM.OCPolarity = TIM_OCPOLARITY_HIGH;
    tim3_PWM.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &tim3_PWM, TIM_CHANNEL_2);

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2500);

    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);

    right_motor_direction_init();
    right_motor_no_dir();
}

void left_motor_direction_init(void) {
    // Set up PC7 and PB6 (IN1 / IN2) to be outputs
    __GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef gpio_config;
    gpio_config.Pin = GPIO_PIN_7;
    gpio_config.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_config.Pull = GPIO_NOPULL;
    gpio_config.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC,&gpio_config);

    __GPIOB_CLK_ENABLE();

    gpio_config.Pin = GPIO_PIN_6;
    gpio_config.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_config.Pull = GPIO_NOPULL;
    gpio_config.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB,&gpio_config);
}

void right_motor_direction_init(void) {
    // Set up PC8 and PC9 (IN3 / IN4) to be outputs
    __GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef gpio_config;
    gpio_config.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    gpio_config.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_config.Pull = GPIO_NOPULL;
    gpio_config.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC,&gpio_config);
}

void motor_set_direction(Motor motor, MotorDirection direction)
{
    if (motor == MOTOR_LEFT) {
        switch (direction) {
        case MOTOR_DIR_FWD:   left_motor_fwd_en(); break;
        case MOTOR_DIR_BWD:   left_motor_bwd_en(); break;
        case MOTOR_DIR_NODIR: left_motor_no_dir(); break;
        }
    } else if (motor == MOTOR_RIGHT) {
        switch (direction) {
        case MOTOR_DIR_FWD:   right_motor_fwd_en(); break;
        case MOTOR_DIR_BWD:   right_motor_bwd_en(); break;
        case MOTOR_DIR_NODIR: right_motor_no_dir(); break;
        }
    }
}

void left_motor_fwd_en(void) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
}
void left_motor_bwd_en(void) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
}
void left_motor_no_dir(void) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
}

void right_motor_fwd_en(void) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
}
void right_motor_bwd_en(void) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
}
void right_motor_no_dir(void) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
}

void left_motor_encoder_init(void) {
    //config for pb4 and pb5
    __GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio_config;
    gpio_config.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    gpio_config.Mode = GPIO_MODE_IT_RISING_FALLING;
    gpio_config.Pull = GPIO_NOPULL; // motor encoder has built in pull up
    gpio_config.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB,&gpio_config);

    HAL_NVIC_SetPriority(EXTI4_IRQn, 0x0f, 0); // set to lowest priority
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0x0f, 0x0f);

    HAL_NVIC_EnableIRQ(EXTI4_IRQn);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void right_motor_encoder_init(void) {
    //config for pb10 and pa8
    GPIO_InitTypeDef gpio_config;

    //setup pb10 first
    __GPIOB_CLK_ENABLE();
    gpio_config.Pin = GPIO_PIN_10;
    gpio_config.Mode = GPIO_MODE_IT_RISING_FALLING;
    gpio_config.Pull = GPIO_NOPULL; // motor encoder has built in pull up
    gpio_config.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB,&gpio_config);

    //setup pa8 second
    __GPIOA_CLK_ENABLE();
    gpio_config.Pin = GPIO_PIN_8;
    gpio_config.Mode = GPIO_MODE_IT_RISING_FALLING;
    gpio_config.Pull = GPIO_NOPULL; // motor encoder has built in pull up
    gpio_config.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA,&gpio_config);


    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0x0f, 0);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0x0f, 0x0f); // set to lowest priority

    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

static volatile int32_t left_encoder_count = 0;
static volatile int32_t right_encoder_count = 0;

/* IRQs for left motor */
void EXTI4_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
}

void EXTI9_5_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
}

/* IRQs for right motor */
void EXTI15_10_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
}

int32_t left_motor_encoder_getValue(void) {
    return left_encoder_count;
}

int32_t right_motor_encoder_getValue(void) {
    return right_encoder_count;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_4) {
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) ==
            HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)) {
            left_encoder_count++;
        } else {
            left_encoder_count--;
        }
    } else if (GPIO_Pin == GPIO_PIN_5) {
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) ==
            HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)) {
            left_encoder_count--;
        } else {
            left_encoder_count++;
        }
    } 
    else if (GPIO_Pin == GPIO_PIN_10) {
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) ==
            HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)) {
            right_encoder_count--;
        } 
        else {
            right_encoder_count++;
        }
    } 
    else if (GPIO_Pin == GPIO_PIN_8) {
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) ==
            HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)) {
            right_encoder_count++;
        } 
        else {
            right_encoder_count--;
        }
    }
}
