#include "dummy_task.h"

#include <stdint.h>

#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "uart.h"
#include "motor.h"
#include "IMU.h"

static void dummy_task_update(void *arg);

static osThreadId_t _dummyTaskThreadID;
static osThreadAttr_t _dummyTaskThreadAttr = 
{
    .name = "heartbeat",
    .priority = osPriorityIdle,
    .stack_size = 128
};

static uint8_t _is_running = 0;
static uint8_t _is_init = 0;

void dummy_task_init(void)
{
    if (!_is_init)
    {
        // CMSIS-RTOS API v2 Timer Documentation: https://www.keil.com/pack/doc/CMSIS/RTOS2/html/group__CMSIS__RTOS__TimerMgmt.html
        _dummyTaskThreadID = osThreadNew(dummy_task_update, NULL, &_dummyTaskThreadAttr);   // Create the thread in the OS scheduler. 
        // Note: The thread starts automatically when osThreadNew is called
        _is_running = 1;
        _is_init = 1;
    }
}

void dummy_task_start(void)
{
    if (!_is_running)
    {
        osThreadResume(_dummyTaskThreadID);
        _is_running = 1;
    }
}

void dummy_task_stop(void)
{
    if (_is_running)
    {
        osThreadSuspend(_dummyTaskThreadID);
        _is_running = 0;
    }
}

uint8_t dummy_task_is_running(void)
{
    return _is_running;
}

void dummy_task_update(void *arg)
{
    UNUSED(arg);
	
    while(1)
    {
        // TODO: Add print statements here
        
        /* TODO: Read IMU */
        // IMU_read();
        
        // /* TODO: Print Y accelleration */
        // // printf("Accel Y: %f\n", get_accY());
        // // printf("Accel Z: %f\n", get_accZ());
        // // printf("Gyro X: %f\n", get_gyroX());
        // printf("Accelerometer Angle: %f\n", get_acc_angle());
        
        /* TODO: Print angle */

        // Non-blocking delay to wait
        osDelay(1000);
    }
}

void dummy_task_deinit(void)
{
    _is_init = 0;
    _is_running = 0;
}
