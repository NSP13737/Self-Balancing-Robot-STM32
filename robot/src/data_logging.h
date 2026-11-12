#ifndef DATALOGGING_H
#define DATALOGGING_H

#include "motor.h"

void logging_init(void);
void current_log(int argc, char *argv[]);
void left_current_logging_start(void);
void right_current_logging_start(void);
void imu_logging_start(void);
void left_encoder_logging_start(void);
void right_encoder_logging_start(void);
void motor_velocity_log(int argc, char *argv[]);
void motor_sine_logging_start(Motor motor);
void logging_stop(void);

#endif
