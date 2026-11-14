#ifndef MOTOR_CMDS_H
#define MOTOR_CMDS_H

void motorDir(int, char *[]);
void motor_set_DC(int, char *[]);
void motor_sine(int, char *[]);
void motor_sine_log(int, char *[]);
void motor_dc_log(int, char *[]);
void setVel(int, char *[]);

#endif
