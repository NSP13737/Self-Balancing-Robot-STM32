#ifndef IMU_H
#define IMU_H

void IMU_init(void);
void IMU_read(void);
float get_accY(void);
float get_accZ(void);
float get_gyroX(void);
double get_acc_angle(void);

#endif