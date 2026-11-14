#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

void kalman_init(void);

/*
 * Correct with this tick's IMU readings, predict the next tick, and place
 * the posterior state in xp: [angular velocity, fused angle, gyro bias].
 */
void kalman_update(float acc_angle, float gyro_velocity, float xp[3]);

void get_state_estimate(float *stateMat[]);

float get_p_theta(float ang_velo, float velo);

#endif
