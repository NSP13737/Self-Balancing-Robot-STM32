#include "motor.h"
#include "motor_cmds.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "data_logging.h"
#include "control_allocation.h"
#include "current_sens.h"

#define SPEED_CONTROL_SAMPLE_MS 5U
#define SPEED_CONTROL_DURATION_MS 5000U

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
    const uint32_t duration_ms = 20000U;
    const uint32_t period_ms = 3000U;
    const uint32_t sample_ms = 5U;
    uint32_t start = HAL_GetTick();

    while ((uint32_t)(HAL_GetTick() - start) < duration_ms) {
        uint32_t elapsed = HAL_GetTick() - start;
        double phase = (2.0 * 3.14159265358979323846 * (elapsed % period_ms)) / period_ms;
        double sine_value = sin(phase);
        MotorDirection direction = (sine_value >= 0.0) ? MOTOR_DIR_FWD : MOTOR_DIR_BWD;
        int duty_cycle = (int)(90.0 * fabs(sine_value) + 0.5);
        motor_set_direction(motor, direction);
        motor_set_dc(motor, duty_cycle);
        osDelay(sample_ms);
    }

    motor_set_dc(motor, 0);
}

static void run_motor_dc_stages(Motor motor)
{
    static const int duty_cycles[] = {80, 80, 80, 80, 80};
    static const MotorDirection directions[] = {
        MOTOR_DIR_FWD, MOTOR_DIR_BWD
    };

    for (size_t direction_index = 0;
         direction_index < sizeof(directions) / sizeof(directions[0]);
         direction_index++) {
        for (size_t dc_index = 0;
             dc_index < sizeof(duty_cycles) / sizeof(duty_cycles[0]);
             dc_index++) {
            uint32_t stage_start = HAL_GetTick();
            motor_set_direction(motor, directions[direction_index]);
            motor_set_dc(motor, duty_cycles[dc_index]);
            while ((uint32_t)(HAL_GetTick() - stage_start) < 2000U) {
                osDelay(5U);
            }
        }
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

    /* Start sampling, then run the sine wave while it logs. */
    motor_sine_logging_start(motor);
    run_motor_sine(motor);
}

void motor_dc_log(int argc, char *argv[])
{
    if (argc != 2) {
        printf("Usage: motorDCLog <left|right>\n");
        return;
    }

    Motor motor;
    if (!parse_motor(argv[1], &motor)) {
        return;
    }

    motor_sine_logging_start(motor);
    run_motor_dc_stages(motor);
}


void setVel(int argc, char *argv[]) {
    if (argc != 3) {
        printf("Usage: setVel <left|right> <speed_rad_s>\n");
        return;
    }

    Motor motor;
    if (!parse_motor(argv[1], &motor)) {
        return;
    }

    char *end = NULL;
    float target_velocity = strtof(argv[2], &end);
    if (end == argv[2] || *end != '\0' || !isfinite(target_velocity)) {
        printf("Speed must be a finite number in rad/s\n");
        return;
    }

    uint32_t start = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - start) < SPEED_CONTROL_DURATION_MS) {
        float measured_velocity = (motor == MOTOR_LEFT) ?
            get_left_motor_velocity_rad_s() : get_right_motor_velocity_rad_s(); 
        printf("%f,%f,%d\n",
            measured_velocity,
            (motor == MOTOR_LEFT) ? left_current_read() : right_current_read(),
            motor_get_dc(motor) );

        float desired_voltage = controlAllocation(
            target_velocity, measured_velocity,
            SPEED_CONTROL_SAMPLE_MS / 1000.0f);
        voltage_motor_control(motor, desired_voltage);
        osDelay(SPEED_CONTROL_SAMPLE_MS);
    }

    /* Leave the selected motor stopped when the timed control run ends. */
    voltage_motor_control(motor, 0.0f);
}
