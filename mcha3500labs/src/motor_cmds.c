#include "motor.h"
#include "motor_cmds.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "data_logging.h"

static int parse_motor(const char *name, Motor *motor)
{
    if (strcmp(name, "left") == 0) {
        *motor = MOTOR_LEFT;
    } else if (strcmp(name, "right") == 0) {
        *motor = MOTOR_RIGHT;
    } else {
        printf("Motor must be left or right\n");
        return 0;
    }
    return 1;
}

static void run_motor_sine(Motor motor)
{
    const uint32_t duration_ms = 10000U;
    const uint32_t period_ms = 2000U;
    const uint32_t sample_ms = 20U;
    uint32_t start = HAL_GetTick();

    while ((uint32_t)(HAL_GetTick() - start) < duration_ms) {
        uint32_t elapsed = HAL_GetTick() - start;
        double phase = (2.0 * 3.14159265358979323846 * (elapsed % period_ms)) / period_ms;
        double sine_value = sin(phase);
        MotorDirection direction = (sine_value >= 0.0) ? MOTOR_DIR_FWD : MOTOR_DIR_BWD;
        int duty_cycle = (int)(75.0 * fabs(sine_value) + 0.5);
        motor_set_direction(motor, direction);
        motor_set_dc(motor, duty_cycle);
        osDelay(sample_ms);
    }

    motor_set_dc(motor, 0);
}

void motorDir(int argc, char *argv[])
{
    if (argc != 3) {
        printf("Usage: motorDir <left|right> <FWD|BWD|NODIR>\n");
        return;
    }

    Motor motor;
    if (strcmp(argv[1], "left") == 0) {
        motor = MOTOR_LEFT;
    } else if (strcmp(argv[1], "right") == 0) {
        motor = MOTOR_RIGHT;
    } else {
        printf("Motor must be left or right\n");
        return;
    }

    MotorDirection direction;
    if (strcmp(argv[2], "FWD") == 0) {
        direction = MOTOR_DIR_FWD;
    } else if (strcmp(argv[2], "BWD") == 0) {
        direction = MOTOR_DIR_BWD;
    } else if (strcmp(argv[2], "NODIR") == 0) {
        direction = MOTOR_DIR_NODIR;
    } else {
        printf("Direction must be FWD, BWD, or NODIR\n");
        return;
    }

    motor_set_direction(motor, direction);
}

void motor_set_DC(int argc, char *argv[]) {
    if (argc != 3) {
        printf("Usage: motorDir <left|right> <duty_cycle_percent>\n");
        return;
    }

    Motor motor;
    if (strcmp(argv[1], "left") == 0) {
        motor = MOTOR_LEFT;
    } else if (strcmp(argv[1], "right") == 0) {
        motor = MOTOR_RIGHT;
    } else {
        printf("Motor must be left or right\n");
        return;
    }

    int motor_dc_percent = atoi(argv[2]);
    if ((motor_dc_percent < 0) || (motor_dc_percent > 100)) {
        printf("Please make sure DC bewteen 0 and 100.\n");
        return;
    }
    motor_set_dc(motor, motor_dc_percent);
}

void motor_sine(int argc, char *argv[])
{
    if (argc != 2) {
        printf("Usage: motorSine <left|right>\n");
        return;
    }

    Motor motor;
    if (!parse_motor(argv[1], &motor)) {
        return;
    }

    run_motor_sine(motor);
}

void motor_sine_log(int argc, char *argv[])
{
    if (argc != 2) {
        printf("Usage: motorSineLog <left|right>\n");
        return;
    }

    Motor motor;
    if (!parse_motor(argv[1], &motor)) {
        return;
    }

    /* Start sampling, then run the 10-second sine wave while it logs. */
    motor_sine_logging_start(motor);
    run_motor_sine(motor);
}
