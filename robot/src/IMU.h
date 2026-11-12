#ifndef IMU_H
#define IMU_H

void IMU_init(void);
void IMU_read(void);
float get_accZ(void);
float get_accX(void);
float get_gyroY(void);
double get_acc_angle(void);

#endif