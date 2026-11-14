#include "motor.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "batt_voltage.h"
#include "current_sens.h"
#include <stdio.h>
#include <math.h>

#define MOTOR_VELOCITY_SAMPLE_MS 5U
#define MOTOR_VELOCITY_WINDOW 3U
#define MOTOR_PI 3.14159265358979323846f

static uint8_t _left_motor_is_init = 0;
static uint8_t _right_motor_is_init = 0;

static TIM_HandleTypeDef htim3;
static volatile int left_motor_dc_percent = 0;
static volatile int right_motor_dc_percent = 0;
static volatile MotorDirection left_motor_direction = MOTOR_DIR_NODIR;
static volatile MotorDirection right_motor_direction = MOTOR_DIR_NODIR;

static uint32_t motor_dc_to_compare(int percent_dc);

/* SETUP FOR VELOCITY ESTIMATION */
static void motor_velocity_thread(void *arg);

#define ENCODER_COUNTS_PER_WHEEL_REV 1320.0f // 11 pulses per revolution * 4 (for quadrature) * 30 (gear ratio)
static volatile float left_velocity_rad_s = 0.0f;
static volatile float right_velocity_rad_s = 0.0f;
static int32_t left_count_samples[MOTOR_VELOCITY_WINDOW];
static int32_t right_count_samples[MOTOR_VELOCITY_WINDOW];
static uint32_t velocity_time_samples[MOTOR_VELOCITY_WINDOW];
static uint32_t velocity_sample_index = 0U;
static uint32_t velocity_sample_count = 0U;

static const osThreadAttr_t motor_velocity_thread_attr = {
    .name = "motor_velocity",
    .priority = osPriorityLow,
    .stack_size = 512U
};

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
    //calculates 10khz assuming 180mhz cpu (I think)
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

void motor_set_dc(Motor motor, int percent_dc) {
    if (motor == MOTOR_LEFT) {
        setLeftMotorDC(percent_dc);
    }
    if (motor == MOTOR_RIGHT) {
        setRightMotorDC(percent_dc);
    }
}

static uint32_t motor_dc_to_compare(int percent_dc) {
    if (percent_dc < 0) {
        percent_dc = 0;
    } else if (percent_dc > 100) {
        percent_dc = 100;
    }

    return ((uint32_t)(htim3.Init.Period + 1U) * (uint32_t)percent_dc) / 100U;
}

void setLeftMotorDC(int percent_dc) {
    if (percent_dc < 0) percent_dc = 0;
    if (percent_dc > 100) percent_dc = 100;
    left_motor_dc_percent = percent_dc;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, motor_dc_to_compare(percent_dc));
}

void setRightMotorDC(int percent_dc) {
    if (percent_dc < 0) percent_dc = 0;
    if (percent_dc > 100) percent_dc = 100;
    right_motor_dc_percent = percent_dc;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, motor_dc_to_compare(percent_dc));
}

int motor_get_dc(Motor motor)
{
    if (motor == MOTOR_LEFT) return left_motor_dc_percent;
    if (motor == MOTOR_RIGHT) return right_motor_dc_percent;
    return 0;
}

float motor_get_voltage(Motor motor)
{
    int dc = motor_get_dc(motor);
    MotorDirection direction;

    if (motor == MOTOR_LEFT) direction = left_motor_direction;
    else if (motor == MOTOR_RIGHT) direction = right_motor_direction;
    else return 0.0f;

    if (direction == MOTOR_DIR_NODIR) return 0.0f;

    float signed_dc = (direction == MOTOR_DIR_BWD) ? -(float)dc : (float)dc;
    float batt_v = batt_voltage_read();
    return (batt_v > 0) ? ((batt_v-get_driver_v_drop(motor))* signed_dc / 100.0f) : ((batt_v+get_driver_v_drop(motor))* signed_dc / 100.0f);
}

void voltage_motor_control(Motor motor, float desired_motor_voltage) {
    float battery_voltage = batt_voltage_read();
    if (desired_motor_voltage == 0.0f) {
        motor_set_dc(motor, 0);
        return;
    }

    // compensate for the motor driver's voltage drop before converting to PWM
    float voltage_magnitude = fabsf(desired_motor_voltage);
    float voltage_pre_driver = voltage_magnitude + get_driver_v_drop(motor);
    float duty_percent = (voltage_pre_driver / battery_voltage) * 100.0f;
    if (duty_percent > 100.0f) duty_percent = 100.0f;

    motor_set_direction(motor, (desired_motor_voltage >= 0.0f) ?
                        MOTOR_DIR_FWD : MOTOR_DIR_BWD);
    motor_set_dc(motor, (int)(duty_percent + 0.5f));
}

float get_driver_v_drop(Motor motor) {
    float (*get_current)(void) = (motor == MOTOR_LEFT) ? left_current_read : right_current_read;
    return 1.4f+(1.15f*fabs(get_current())); // We want this to be in abs val bc sign is handled by motor_get_voltage
}

/* MOTOR DIRECTION CONTROL */

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
        left_motor_direction = direction;
        switch (direction) {
        case MOTOR_DIR_FWD:   left_motor_fwd_en(); break;
        case MOTOR_DIR_BWD:   left_motor_bwd_en(); break;
        case MOTOR_DIR_NODIR: left_motor_no_dir(); break;
        }
    } else if (motor == MOTOR_RIGHT) {
        right_motor_direction = direction;
        switch (direction) {
        case MOTOR_DIR_FWD:   right_motor_fwd_en(); break;
        case MOTOR_DIR_BWD:   right_motor_bwd_en(); break;
        case MOTOR_DIR_NODIR: right_motor_no_dir(); break;
        }
    }
}

void left_motor_fwd_en(void) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
}
void left_motor_bwd_en(void) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
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

/* ENCODER */

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

float get_left_motor_velocity_rad_s(void)
{
    return left_velocity_rad_s;
}

float get_right_motor_velocity_rad_s(void)
{
    return right_velocity_rad_s;
}

void motor_velocity_init(void)
{
    static uint8_t initialized = 0U;
    if (!initialized) {
        initialized = 1U;
        osThreadNew(motor_velocity_thread, NULL, &motor_velocity_thread_attr);
    }
}

static void motor_velocity_thread(void *arg)
{
    (void)arg;
    int32_t previous_left_count = left_motor_encoder_getValue();
    int32_t previous_right_count = right_motor_encoder_getValue();
    uint32_t previous_time = HAL_GetTick();

    for (;;) {
        int32_t current_left_count;
        int32_t current_right_count;
        int32_t left_delta;
        int32_t right_delta;
        uint32_t current_time;
        uint32_t elapsed_ms;
        int64_t left_sum = 0;
        int64_t right_sum = 0;
        uint32_t elapsed_sum_ms = 0U;

        osDelay(MOTOR_VELOCITY_SAMPLE_MS);
        current_time = HAL_GetTick();
        current_left_count = left_motor_encoder_getValue();
        current_right_count = right_motor_encoder_getValue();
        left_delta = (int32_t)((uint32_t)current_left_count - (uint32_t)previous_left_count);
        right_delta = (int32_t)((uint32_t)current_right_count - (uint32_t)previous_right_count);
        elapsed_ms = current_time - previous_time;
        previous_left_count = current_left_count;
        previous_right_count = current_right_count;
        previous_time = current_time;

        if (elapsed_ms == 0U) {
            continue;
        }

        left_count_samples[velocity_sample_index] = left_delta;
        right_count_samples[velocity_sample_index] = right_delta;
        velocity_time_samples[velocity_sample_index] = elapsed_ms;
        velocity_sample_index = (velocity_sample_index + 1U) % MOTOR_VELOCITY_WINDOW;
        if (velocity_sample_count < MOTOR_VELOCITY_WINDOW) {
            velocity_sample_count++;
        }

        for (uint32_t i = 0U; i < velocity_sample_count; i++) {
            left_sum += left_count_samples[i];
            right_sum += right_count_samples[i];
            elapsed_sum_ms += velocity_time_samples[i];
        }

        if (ENCODER_COUNTS_PER_WHEEL_REV > 0.0f && elapsed_sum_ms > 0U) {
            float counts_to_radians = (2.0f * MOTOR_PI) / ENCODER_COUNTS_PER_WHEEL_REV;
            float elapsed_seconds = (float)elapsed_sum_ms / 1000.0f;
            left_velocity_rad_s = ((float)left_sum * counts_to_radians) / elapsed_seconds;
            right_velocity_rad_s = ((float)right_sum * counts_to_radians) / elapsed_seconds;
        } else {
            left_velocity_rad_s = 0.0f;
            right_velocity_rad_s = 0.0f;
        }
    }
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


