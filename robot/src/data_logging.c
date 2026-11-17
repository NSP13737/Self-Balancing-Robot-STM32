#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <inttypes.h> // For PRIxx and SCNxx macros
#include "stm32f4xx_hal.h" // to import UNUSED() macro
#include "current_sens.h"
#include "batt_voltage.h"
#include "extra_pin.h"
#include "cmsis_os2.h"
#include "data_logging.h"
#include "IMU.h"
#include "motor.h"
#include "state_estimator.h"
#include "control_allocation.h"
#include "controller.h"
#include "tuning_parameters.h"

#define TIMER_PERIOD_MS ((float)TUNING_SAMPLE_PERIOD_MS)
#define MOTOR_LOG_DURATION_SECONDS 3U
#define VELOCITY_LOG_DURATION_SECONDS 5U
#define MOTOR_SINE_LOG_DURATION_SECONDS 20U
#define STATE_ESTIMATE_LOG_DURATION_SECONDS 5U
#define CONTROL_LOOP_DURATION_SECONDS 5U
#define CONTROL_LOOP_SAMPLE_COUNT \
    (CONTROL_LOOP_DURATION_SECONDS * 1000U / (uint32_t)TIMER_PERIOD_MS)

uint16_t logCount;

/* Variable Declarations */
static void (*log_function)(void);
int _is_init = 0;
osTimerId_t osTim_dataLogging;
static int32_t (*encoder_read)(void);
static float (*current_read)(void);
static Motor sine_log_motor;

/* Compact samples so the full five-second capture fits in MCU SRAM. */
typedef struct {
    int32_t wheel_state_mrad;
    int16_t angle_state_10urad;
    int16_t momentum_state_milli;
    int32_t lqr_action_milli;
    int16_t allocated_voltage_milli;
    int16_t raw_imu_angle_10urad;
    int16_t raw_imu_velocity_milli;
    int16_t left_current_milli;
    int16_t right_current_milli;
    int16_t left_voltage_milli;
    int16_t right_voltage_milli;
    int16_t left_velocity_centi;
    int16_t right_velocity_centi;
    int16_t average_velocity_centi;
    int32_t left_encoder_mrad;
    int32_t right_encoder_mrad;
} ControlLoopSample_t;

static ControlLoopSample_t control_loop_samples[CONTROL_LOOP_SAMPLE_COUNT];
static uint16_t control_loop_sample_count;

/* Function Declarations */
static void log_pointer(void *argument); // The param (void *argument) is required by FreeRTOS for timer callback
static void encoder_log_data(void);
static void velocity_log_data(void);
static void motor_sine_log_data(void);
static void state_estimate_log_data(void);
static void imu_kf_log_data(void);
static void control_loop_log_data(void);
void logging_start(void *log_function_ptr);

static int16_t quantize_control_loop_value(float value, float scale)
{
    float scaled = value * scale;
    if (scaled >= 32767.0f) return INT16_MAX;
    if (scaled <= -32768.0f) return INT16_MIN;
    scaled += (scaled >= 0.0f) ? 0.5f : -0.5f;
    return (int16_t)scaled;
}

static int32_t quantize_wide_control_loop_value(float value, float scale)
{
    float scaled = value * scale;
    if (scaled >= 2147483520.0f) return INT32_MAX;
    if (scaled <= -2147483648.0f) return INT32_MIN;
    scaled += (scaled >= 0.0f) ? 0.5f : -0.5f;
    return (int32_t)scaled;
}

void logging_init(void) {
    if (_is_init) {
        return;
    }
    _is_init = 1;
    
    // We set the attributes for the timer to their default values
    osTimerAttr_t  dataLoggingTimerAttr = {
        .name = NULL,
        .attr_bits = 0,
        .cb_mem = NULL,
        .cb_size = 0,
    };

    osTim_dataLogging = osTimerNew(log_pointer, osTimerPeriodic, NULL, &dataLoggingTimerAttr);
}

void logging_start(void *log_function_ptr) {
    log_function = log_function_ptr;

    logCount = 0;
    if (osTim_dataLogging != NULL) {
        osTimerStart(osTim_dataLogging, (uint32_t)TIMER_PERIOD_MS);
    }
    else {
        //is there a good way to flag this somehow?
        return;
    }
}

static void log_pointer(void *argument) {
    UNUSED(argument);

    (*log_function)();
}

static void current_log_data(void) {
    float relative_time = ((float)TIMER_PERIOD_MS/1000) * logCount;
    printf("%f,%f", relative_time, current_read());
    logCount++;
    if (logCount >= (2 / ((float)TIMER_PERIOD_MS/1000))) {// call after 2 seconds
        printf("STOP\n");
        logging_stop();
    }
    printf("\n");
}

static void imu_log_data(void) {

    IMU_read();

    double acc_angle = get_acc_angle();
    float gyro_Y = get_gyroY();
    float encoderCount = right_motor_encoder_getValue();
    float relative_time = (TIMER_PERIOD_MS/1000) * logCount;
    printf("%f,%f,%f,%f", relative_time, acc_angle, gyro_Y, encoderCount);
    logCount++;
    if (logCount >= (5 / (TIMER_PERIOD_MS/1000))) {// call after 5 seconds
        logging_stop();
    }
    printf("\n");
}

static void encoder_log_data(void) {
    float relative_time = ((float)TIMER_PERIOD_MS / 1000.0f) * logCount;
    printf("%f,%" PRId32 "\n", relative_time, encoder_read());
    logCount++;
    if (logCount >= (MOTOR_LOG_DURATION_SECONDS * 1000U / TIMER_PERIOD_MS)) {
        logging_stop();
    }
}

static void velocity_log_data(void) {
    printf("Left: %f | Right: %f\n",
           get_left_motor_velocity_rad_s(), get_right_motor_velocity_rad_s());
    logCount++;
    if (logCount >= (VELOCITY_LOG_DURATION_SECONDS * 1000U / (uint32_t)TIMER_PERIOD_MS)) {
        logging_stop();
    }
}

static void motor_sine_log_data(void) {
    float time_seconds = ((float)logCount * TIMER_PERIOD_MS) / 1000.0f;
    float voltage = motor_get_voltage(sine_log_motor);
    float current = (sine_log_motor == MOTOR_LEFT) ? left_current_read() : right_current_read();
    float velocity = (sine_log_motor == MOTOR_LEFT) ?
                     get_left_motor_velocity_rad_s() : get_right_motor_velocity_rad_s();
    float pc3_pin_voltage = extra_pin_read();
    float torque = 0.3577f * (pc3_pin_voltage - 1.6337f);

    printf("%f,%f,%f,%f,%f\n", time_seconds, voltage, current, velocity, torque);
    logCount++;
    if (logCount >= (MOTOR_SINE_LOG_DURATION_SECONDS * 1000U / (uint32_t)TIMER_PERIOD_MS)) {
        logging_stop();
    }
}

static void state_estimate_log_data(void)
{
    float state0, state1, state2;
    float *state[3] = {&state0, &state1, &state2};
    float time_seconds = ((float)logCount * TIMER_PERIOD_MS) / 1000.0f;

    IMU_read();
    get_state_estimate(state);
    printf("%f,%f,%f,%f\n", time_seconds, state0, state1, state2);

    logCount++;
    if (logCount >= (STATE_ESTIMATE_LOG_DURATION_SECONDS * 1000U /
                     (uint32_t)TIMER_PERIOD_MS)) {
        logging_stop();
    }
}

void state_estimate_log(int argc, char *argv[])
{
    (void)argv;
    if (argc != 1) {
        printf("Usage: stateEstimateLog\n");
        return;
    }

    logging_start(&state_estimate_log_data);
}

static void imu_kf_log_data(void)
{
    float kalman_state[3];
    float imu_angle;
    float imu_ang_velo;
    float time_seconds = ((float)logCount * TIMER_PERIOD_MS) / 1000.0f;

    IMU_read();
    imu_angle = (float)get_acc_angle();
    imu_ang_velo = get_gyroY();
    kalman_update(imu_angle, imu_ang_velo, kalman_state);

    /* Kalman state is [angular velocity, angle, gyro bias]. */
    printf("%f,%f,%f,%f,%f\n", time_seconds, imu_angle, imu_ang_velo,
           kalman_state[1], kalman_state[0] + kalman_state[2]);

    logCount++;
    if (logCount >= (STATE_ESTIMATE_LOG_DURATION_SECONDS * 1000U /
                     (uint32_t)TIMER_PERIOD_MS)) {
        logging_stop();
    }
}

void imu_kf_log(int argc, char *argv[])
{
    (void)argv;
    if (argc != 1) {
        printf("Usage: imuKFLog\n");
        return;
    }

    kalman_init();
    logging_start(&imu_kf_log_data);
}

void read_gyro_velocity(int argc, char *argv[])
{
    (void)argv;
    if (argc != 1) {
        printf("Usage: gyroVel\n");
        return;
    }

    IMU_read();
    printf("%f\n", get_gyroY());
}

void read_imu_angle(int argc, char *argv[])
{
    if (argc != 2) {
        printf("Usage: imuAngle <raw|kalman>\n");
        return;
    }

    if (strcmp(argv[1], "raw") == 0) {
        IMU_read();
        printf("%f\n", (float)get_acc_angle());
    } else if (strcmp(argv[1], "kalman") == 0) {
        float wheel_state, angle_state, momentum_state;
        float *state[3] = {&wheel_state, &angle_state, &momentum_state};
        get_state_estimate(state);
        printf("%f\n", angle_state);
    } else {
        printf("Angle mode must be raw or kalman\n");
    }
}

void read_wheel_rotation(int argc, char *argv[])
{
    if (argc != 2) {
        printf("Usage: wheelRotation <left|right>\n");
        return;
    }

    if (strcmp(argv[1], "left") == 0) {
        printf("%f\n", get_left_motor_position_rad());
    } else if (strcmp(argv[1], "right") == 0) {
        printf("%f\n", get_right_motor_position_rad());
    } else {
        printf("Wheel must be left or right\n");
    }
}

void motor_sine_logging_start(Motor motor)
{
    sine_log_motor = motor;
    logging_start(&motor_sine_log_data);
}

void motor_velocity_log(int argc, char *argv[])
{
    (void)argv;
    if (argc != 1) {
        printf("Usage: motorVelocityLog\n");
        return;
    }

    logging_start(&velocity_log_data);
}

void current_log(int argc, char *argv[])
{
    if (argc != 2) {
        printf("Usage: currentLog <left|right>\n");
        return;
    }

    if (strcmp(argv[1], "left") == 0) {
        left_current_logging_start();
    } else if (strcmp(argv[1], "right") == 0) {
        right_current_logging_start();
    } else {
        printf("Motor must be left or right\n");
        return;
    }
}

void left_current_logging_start(void) {
    current_read = left_current_read;
    logging_start(&current_log_data);
}

void right_current_logging_start(void) {
    current_read = right_current_read;
    logging_start(&current_log_data);
}

void imu_logging_start(void) {

    logging_start(&imu_log_data);

}

void left_encoder_logging_start(void) {
    encoder_read = left_motor_encoder_getValue;
    logging_start(&encoder_log_data);
}

void right_encoder_logging_start(void) {
    encoder_read = right_motor_encoder_getValue;
    logging_start(&encoder_log_data);
}

void control_loop_log(int argc, char *argv[])
{
    (void)argv;
    if (argc != 1) {
        printf("Usage: controlLoopLog\n");
        return;
    }

    control_loop_sample_count = 0;
    logging_start(&control_loop_log_data);
}

static void control_loop_log_data(void) {
    /* Get state estimate */
    float wheel_state, angle_state, momentum_state;
    float *state[3] = {&wheel_state, &angle_state, &momentum_state};
    get_state_estimate(state);

    /* Collect IMU info. */
    // Do not call IMU_read here; these values must match the state estimate.
    // we can get the same IMU vals as the state estimate used
    float raw_imu_angle = get_acc_angle();
    float raw_imu_velo = get_gyroY();

    ctrl_set_x1(wheel_state);
    ctrl_set_x2(angle_state);
    ctrl_set_x3(momentum_state);
    ctrl_update();

    if ((angle_state < -ANGLE_CUTOFF) || (angle_state > ANGLE_CUTOFF)) {
        logging_stop();
    }

    float lqr_control_action = getControl();

    float left_wheel_vel = get_left_motor_velocity_rad_s();
    float right_wheel_vel = get_right_motor_velocity_rad_s();
    float avg_wheel_vel = (left_wheel_vel + right_wheel_vel) / 2.0f;

    float control_allocated_action = controlAllocation(
        lqr_control_action, avg_wheel_vel, TIMER_PERIOD_MS / 1000.0f);

    voltage_motor_control(MOTOR_LEFT, control_allocated_action);
    voltage_motor_control(MOTOR_RIGHT, control_allocated_action);

    // Collect other relevant outputs to log.
    float left_motor_current = left_current_read();
    float right_motor_current = right_current_read();
    float left_motor_voltage = motor_get_voltage(MOTOR_LEFT);
    float right_motor_voltage = motor_get_voltage(MOTOR_RIGHT);
    float left_motor_encoder = get_left_motor_position_rad();
    float right_motor_encoder = get_right_motor_position_rad();
    ControlLoopSample_t *sample = &control_loop_samples[control_loop_sample_count];
    sample->wheel_state_mrad = quantize_wide_control_loop_value(wheel_state, 1000.0f);
    sample->angle_state_10urad = quantize_control_loop_value(angle_state, 10000.0f);
    sample->momentum_state_milli = quantize_control_loop_value(momentum_state, 1000.0f);
    sample->lqr_action_milli = quantize_wide_control_loop_value(lqr_control_action, 1000.0f);
    sample->allocated_voltage_milli = quantize_control_loop_value(control_allocated_action, 1000.0f);
    sample->raw_imu_angle_10urad = quantize_control_loop_value(raw_imu_angle, 10000.0f);
    sample->raw_imu_velocity_milli = quantize_control_loop_value(raw_imu_velo, 1000.0f);
    sample->left_current_milli = quantize_control_loop_value(left_motor_current, 1000.0f);
    sample->right_current_milli = quantize_control_loop_value(right_motor_current, 1000.0f);
    sample->left_voltage_milli = quantize_control_loop_value(left_motor_voltage, 1000.0f);
    sample->right_voltage_milli = quantize_control_loop_value(right_motor_voltage, 1000.0f);
    sample->left_velocity_centi = quantize_control_loop_value(left_wheel_vel, 100.0f);
    sample->right_velocity_centi = quantize_control_loop_value(right_wheel_vel, 100.0f);
    sample->average_velocity_centi = quantize_control_loop_value(avg_wheel_vel, 100.0f);
    sample->left_encoder_mrad = quantize_wide_control_loop_value(left_motor_encoder, 1000.0f);
    sample->right_encoder_mrad = quantize_wide_control_loop_value(right_motor_encoder, 1000.0f);

    control_loop_sample_count++;
    logCount++;
    if (control_loop_sample_count >= CONTROL_LOOP_SAMPLE_COUNT) {
        logging_stop();
        setRightMotorDC(0.0f);
        setLeftMotorDC(0.0f);

        /* Sampling is complete; send the buffered capture over UART now. */
        for (uint16_t i = 0; i < control_loop_sample_count; ++i) {
            const ControlLoopSample_t *row = &control_loop_samples[i];
            float relative_time = ((float)TIMER_PERIOD_MS / 1000.0f) * i;
            printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,"
                   "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
                   relative_time,
                   row->wheel_state_mrad / 1000.0f,
                   row->angle_state_10urad / 10000.0f,
                   row->momentum_state_milli / 1000.0f,
                   row->lqr_action_milli / 1000.0f,
                   row->allocated_voltage_milli / 1000.0f,
                   row->raw_imu_angle_10urad / 10000.0f,
                   row->raw_imu_velocity_milli / 1000.0f,
                   row->left_current_milli / 1000.0f,
                   row->right_current_milli / 1000.0f,
                   row->left_voltage_milli / 1000.0f,
                   row->right_voltage_milli / 1000.0f,
                   row->left_velocity_centi / 100.0f,
                   row->right_velocity_centi / 100.0f,
                   row->average_velocity_centi / 100.0f,
                   row->left_encoder_mrad / 1000.0f,
                   row->right_encoder_mrad / 1000.0f,
                   TUNING_KP, TUNING_INTEGRATOR_SCALAR,
                   TUNING_CTRL_MK_0, TUNING_CTRL_MK_1,
                   TUNING_CTRL_MK_2, TUNING_CTRL_MK_3,
                   TUNING_CTRL_AZ_0, TUNING_CTRL_AZ_1,
                   TUNING_CTRL_AZ_2, TUNING_CTRL_AZ_3,
                   TUNING_Q_DIAG_0, TUNING_Q_DIAG_1,
                   TUNING_Q_DIAG_2, TUNING_Q_DIAG_3, TUNING_R);
        }
    }
}

void logging_stop(void) {
    osTimerStop(osTim_dataLogging);
}
