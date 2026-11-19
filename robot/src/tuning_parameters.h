#ifndef TUNING_PARAMETERS_H
#define TUNING_PARAMETERS_H

/* Control allocation gains. */
#define TUNING_KP 1.2f
//#define TUNING_INTEGRATOR_SCALAR 2.27272727273f
#define TUNING_INTEGRATOR_SCALAR 8.02918f

/* Shared control and sampling period. Keep integral/state-space gains tuned for this period. */
#define TUNING_SAMPLE_PERIOD_MS 5U
#define TUNING_SAMPLE_PERIOD_S (TUNING_SAMPLE_PERIOD_MS / 1000.0f)

/* Controller matrices: mK and Az are stored as row-major 1x4 arrays. */
// Old mK vals
// #define TUNING_CTRL_MK_0 4.492148056511271f
// #define TUNING_CTRL_MK_1 9.967726608934711f
// #define TUNING_CTRL_MK_2 22.355953248794705f
// #define TUNING_CTRL_MK_3 1.274592539852193f

//removing integral action
// #define TUNING_CTRL_MK_0 4.492148056511271f
// #define TUNING_CTRL_MK_1 9.967726608934711f
// #define TUNING_CTRL_MK_2 22.355953248794705f
// #define TUNING_CTRL_MK_3 0.0f

//removing the robot caring about the wheel position
// #define TUNING_CTRL_MK_0 0.0f
// #define TUNING_CTRL_MK_1 9.967726608934711f
// #define TUNING_CTRL_MK_2 22.355953248794705f
// #define TUNING_CTRL_MK_3 0.0f

//Tmp testing K vals
// #define TUNING_CTRL_MK_0 14.955f
// #define TUNING_CTRL_MK_1 28.948f
// #define TUNING_CTRL_MK_2 65.065f
// #define TUNING_CTRL_MK_3 3.0604f

// New derived K vals
// #define TUNING_CTRL_MK_0 8.8508f
// #define TUNING_CTRL_MK_1 18.209f
// #define TUNING_CTRL_MK_2 40.948f
// #define TUNING_CTRL_MK_3 1.7863f

//Testing other combos
#define TUNING_CTRL_MK_0 3.0f
#define TUNING_CTRL_MK_1 18.209f
#define TUNING_CTRL_MK_2 40.948f
#define TUNING_CTRL_MK_3 1.7863f

#define TUNING_CTRL_AZ_0 0.005f
#define TUNING_CTRL_AZ_1 0.0f
#define TUNING_CTRL_AZ_2 0.0f
#define TUNING_CTRL_AZ_3 1.0f

/* Logged for verification only; these values do not affect control. */
#define TUNING_Q_DIAG_0 10.0f
#define TUNING_Q_DIAG_1 1.0f
#define TUNING_Q_DIAG_2 1.0f
#define TUNING_Q_DIAG_3 1.0f
#define TUNING_R 0.3f

#endif
