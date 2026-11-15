#ifndef TUNING_PARAMETERS_H
#define TUNING_PARAMETERS_H

/* Control allocation gains. */
#define TUNING_KP 0.8f
#define TUNING_INTEGRATOR_SCALAR 2.27272727273f

/* Shared control and sampling period. Keep integral/state-space gains tuned for this period. */
#define TUNING_SAMPLE_PERIOD_MS 5U
#define TUNING_SAMPLE_PERIOD_S (TUNING_SAMPLE_PERIOD_MS / 1000.0f)

/* Controller matrices: mK and Az are stored as row-major 1x4 arrays. */
#define TUNING_CTRL_MK_0 4.492148056511271f
#define TUNING_CTRL_MK_1 9.967726608934711f
#define TUNING_CTRL_MK_2 22.355953248794705f
#define TUNING_CTRL_MK_3 1.274592539852193f

#define TUNING_CTRL_AZ_0 0.005f
#define TUNING_CTRL_AZ_1 0.0f
#define TUNING_CTRL_AZ_2 0.0f
#define TUNING_CTRL_AZ_3 1.0f

/* Logged for verification only; these values do not affect control. */
#define TUNING_Q_DIAG_0 10.0f
#define TUNING_Q_DIAG_1 1.0f
#define TUNING_Q_DIAG_2 1.0f
#define TUNING_Q_DIAG_3 1.0f
#define TUNING_R 0.6f

#endif
