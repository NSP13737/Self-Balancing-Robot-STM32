#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <inttypes.h> // For PRIxx and SCNxx macros
#include "stm32f4xx_hal.h" // to import UNUSED() macro
#include "pendulum.h"
#include "cmsis_os2.h"
#include "data_logging.h"
#include "IMU.h"
#include "motor.h"

uint16_t logCount;

#define TIMER_PERIOD_MS 5.0f // 200 Hz with a 1 kHz RTOS tick
#define MOTOR_LOG_DURATION_SECONDS 3U
/* Variable Declarations */
static void (*log_function)(void);
int _is_init = 0;
osTimerId_t osTim_dataLogging;

/* Function Declarations */
static void log_pointer(void *argument); // The param (void *argument) is required by FreeRTOS for timer callback
static void pot_log_data(void);
static void encoder_log_data(void);
static int32_t (*encoder_read)(void);
void logging_start(void *log_function_ptr);

static void pot_log_data(void) {

    float voltage = pot_read_voltage();
    float relative_time = (TIMER_PERIOD_MS/1000) * logCount;
    printf("%f,%f", relative_time, voltage);
    logCount++;
    if (logCount >= (2 / (TIMER_PERIOD_MS/1000))) {// call after 2 seconds
        logging_stop();
    }
    printf("\n");
}

static void imu_log_data(void) {

    IMU_read();

    double acc_angle = get_acc_angle();
    float gyro_X = get_gyroX();
    float potVoltage = pot_read_voltage();
    float relative_time = (TIMER_PERIOD_MS/1000) * logCount;
    printf("%f,%f,%f,%f", relative_time, acc_angle, gyro_X, potVoltage);
    logCount++;
    if (logCount >= (5 / (TIMER_PERIOD_MS/1000))) {// call after 2 seconds
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

static void log_pointer(void *argument) {
    UNUSED(argument);

    (*log_function)();
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

void pot_logging_start(void) {

    logging_start(&pot_log_data);

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

void logging_stop(void) {
    osTimerStop(osTim_dataLogging);
}
