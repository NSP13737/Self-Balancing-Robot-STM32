#include "state_estimator.h"
#include "IMU.h"
#include "motor.h"
#include "tuning_parameters.h"
#include <stddef.h>
#include <math.h>

// Momentum calculation retained for the existing controller model.
#define A_STAR 2.8323f
#define B_STAR -0.9681f
#define SAMPLE_PERIOD TUNING_SAMPLE_PERIOD_S

static float get_p_theta(float ang_velo, float velo);

static const float process_noise[3][3] = {
    { 0.2611f,  0.0223f, -0.0280f },
    { 0.0223f,  0.0021f, -0.0024f },
    {-0.0280f, -0.0024f,  0.0030f }
};

// Using updated angle variance
static const float measurement_noise[2][2] = {
    {0.052f, 0.0f},
    {0.0f, 2.5772e-06f}
};

typedef struct {
    float xm[3];
    float Pm[3][3];
} KalmanFilterState_t;

static KalmanFilterState_t kalman_filter;
static float kalman_angular_velocity = 0.0f;

void kalman_init(void)
{
    size_t row;
    size_t col;

    for (row = 0; row < 3; ++row) {
        kalman_filter.xm[row] = 0.0f;
        for (col = 0; col < 3; ++col) {
            kalman_filter.Pm[row][col] = (row == col) ? 1.0f : 0.0f;
        }
    }
    kalman_angular_velocity = 0.0f;
}

float get_kalman_angular_velocity(void)
{
    return kalman_angular_velocity;
}

void kalman_update(float acc_angle, float gyro_velocity, float xp[3])
{
    float s[2][2];
    float s_inv[2][2];
    float gain[3][2];
    float err[2];
    float pp[3][3];
    float predicted_p[3][3];
    float ad[3][3] = {
        {1.0f, 0.0f, 0.0f},
        {SAMPLE_PERIOD, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    };
    float cp[2][3];
    float determinant;
    size_t i, j, k, m;

    // C = {{0, 1, 0}, {1, 0, 1}}; compute S = C Pm C' + R.
    s[0][0] = kalman_filter.Pm[1][1] + measurement_noise[0][0];
    s[0][1] = kalman_filter.Pm[1][0] + kalman_filter.Pm[1][2];
    s[1][0] = kalman_filter.Pm[0][1] + kalman_filter.Pm[2][1];
    s[1][1] = kalman_filter.Pm[0][0] + kalman_filter.Pm[0][2]
            + kalman_filter.Pm[2][0] + kalman_filter.Pm[2][2]
            + measurement_noise[1][1];

    determinant = s[0][0] * s[1][1] - s[0][1] * s[1][0];
    if (determinant == 0.0f) {
        // With the configured positive measurement noise this is not expected;
        // preserve the current estimate if the covariance is corrupted.
        for (i = 0; i < 3; ++i) xp[i] = kalman_filter.xm[i];
        return;
    }
    s_inv[0][0] =  s[1][1] / determinant;
    s_inv[0][1] = -s[0][1] / determinant;
    s_inv[1][0] = -s[1][0] / determinant;
    s_inv[1][1] =  s[0][0] / determinant;

    // K = Pm C' S^-1. Build Pm C' directly from the two observation rows.
    for (i = 0; i < 3; ++i) {
        cp[0][i] = kalman_filter.Pm[i][1];
        cp[1][i] = kalman_filter.Pm[i][0] + kalman_filter.Pm[i][2];
        for (j = 0; j < 2; ++j) {
            gain[i][j] = cp[0][i] * s_inv[0][j] + cp[1][i] * s_inv[1][j];
        }
    }

    err[0] = acc_angle - kalman_filter.xm[1];
    err[1] = gyro_velocity - (kalman_filter.xm[0] + kalman_filter.xm[2]);
    for (i = 0; i < 3; ++i) {
        xp[i] = kalman_filter.xm[i] + gain[i][0] * err[0] + gain[i][1] * err[1];
    }

    // Pp = Pm - K C Pm.
    for (i = 0; i < 3; ++i) {
        for (j = 0; j < 3; ++j) {
            pp[i][j] = kalman_filter.Pm[i][j]
                - gain[i][0] * kalman_filter.Pm[1][j]
                - gain[i][1] * (kalman_filter.Pm[0][j] + kalman_filter.Pm[2][j]);
        }
    }

    // Predict x and covariance for the next 5 ms tick: Ad*xp and Ad*Pp*Ad'.
    kalman_filter.xm[0] = xp[0];
    kalman_filter.xm[1] = SAMPLE_PERIOD * xp[0] + xp[1];
    kalman_filter.xm[2] = xp[2];
    for (i = 0; i < 3; ++i) {
        for (j = 0; j < 3; ++j) {
            predicted_p[i][j] = process_noise[i][j];
            for (k = 0; k < 3; ++k) {
                for (m = 0; m < 3; ++m) {
                    predicted_p[i][j] += ad[i][k] * pp[k][m] * ad[j][m];
                }
            }
        }
    }
    for (i = 0; i < 3; ++i) {
        for (j = 0; j < 3; ++j) kalman_filter.Pm[i][j] = predicted_p[i][j];
    }
}

static float get_p_theta(float ang_velo, float velo)
{
    return (ang_velo + (B_STAR * velo)) / A_STAR;
}

void get_state_estimate(float *stateMat[])
{
    float xp[3]; // Kalman posterior: angle at index 1, angular velocity at index 0, angular velo bias at index 2.

    // Average left and right wheel speeds.
    *stateMat[0] = (get_left_motor_position_rad()
                  + get_right_motor_position_rad()) / 2.0f;

    IMU_read();
    kalman_update((float)get_acc_angle(), get_gyroY(), xp);
    kalman_angular_velocity = xp[0];
    *stateMat[1] = xp[1];
    *stateMat[2] = get_p_theta(xp[1], xp[2]);

    /* USE THIS IF YOU WANT TO USE RAW IMU VALS */
    // float raw_angle = get_acc_angle();
    // float raw_gyro = get_gyroY();

    // *stateMat[1] = raw_angle;
    // *stateMat[2] = get_p_theta(raw_angle, raw_gyro);
}
