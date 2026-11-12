#include "controller.h"
#include <stddef.h>
#include "stm32f4xx_hal.h"
#include "arm_math.h"

#define CTRL_N_INPUT 1
#define CTRL_N_STATE 5

// Define control mat vals
static float ctrl_mK_f32[CTRL_N_INPUT*CTRL_N_STATE] = 
{
    // negative K, 1x5
    57.482702159361970,	162.0245509425631,	49.490444634019987,	26.596687274730954,	-29.851066893643321
};

static float ctrl_x_f32[CTRL_N_STATE] = 
{
    // estimate of state, 5x1
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
};

static float ctrl_u_f32[CTRL_N_INPUT] = 
{
    // control action 1x1
    0.0,
};

static float ctrl_Az_f32[CTRL_N_STATE] =
{
    // state transition matrix 1x5
    -0.005, 0, 0, 0, 1,

};

static float ctrl_z_f32[CTRL_N_INPUT] =
{
    // integrator state
    0.0
};

// define control matrix variables
// rows, cols, data array
arm_matrix_instance_f32 ctrl_mK = {CTRL_N_INPUT,CTRL_N_STATE,(float32_t *)ctrl_mK_f32};
arm_matrix_instance_f32 ctrl_x = {CTRL_N_STATE,1,(float32_t*)ctrl_x_f32};
arm_matrix_instance_f32 ctrl_u = {CTRL_N_INPUT,1,(float32_t*)ctrl_u_f32};
arm_matrix_instance_f32 ctrl_Az = {1, CTRL_N_STATE,(float32_t*)ctrl_Az_f32};
arm_matrix_instance_f32 ctrl_z = {1, 1,(float32_t *)ctrl_z_f32};

// Control functions
void ctrl_init(void) {
    arm_mat_init_f32(&ctrl_mK, CTRL_N_INPUT,CTRL_N_STATE,(float32_t*)ctrl_mK_f32);
    arm_mat_init_f32(&ctrl_x, CTRL_N_STATE,1, (float32_t*)ctrl_x_f32);
    arm_mat_init_f32(&ctrl_u, CTRL_N_INPUT,1, (float32_t*)ctrl_u_f32);
    arm_mat_init_f32(&ctrl_Az, 1,CTRL_N_STATE, (float32_t*)ctrl_Az_f32);
    arm_mat_init_f32(&ctrl_z, 1,1,(float32_t*)ctrl_z_f32);
}

/* Update state vector elements */
void ctrl_set_x1(float x1)
{
    // Update state x1
    ctrl_x_f32[0] = x1;
}

void ctrl_set_x2(float x2)
{
    // Update state x2
    ctrl_x_f32[1] = x2;
}

void ctrl_set_x3(float x3)
{
    // Update state x3
    ctrl_x_f32[2] = x3;
}

void ctrl_set_x4(float x4)
{
    // Update state x4
    ctrl_x_f32[3] = x4;
}

/* Get the current control output */
float getControl(void)
{
    return ctrl_u_f32[0];
}

/* Update control output */
void ctrl_update(void)
{
    // Compute control action: Multiply negative k by x (our estimated state)
    arm_mat_mult_f32(&ctrl_mK, &ctrl_x, &ctrl_u);

    // Update integrator state
    arm_mat_mult_f32(&ctrl_Az, &ctrl_x, &ctrl_z);
    // Copy updated value of integrator state into state vector
    ctrl_x_f32[4] = ctrl_z_f32[0];
}
