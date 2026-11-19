#include "batt_voltage.h"
#include "tuning_parameters.h"

static float control_allocation_integrator = 0.0f;

void controlAllocationResetIntegrator(void)
{
    control_allocation_integrator = 0.0f;
}

float controlAllocation(float LQR_vel, float current_vel, float sample_period) {
    float error_vel = LQR_vel - current_vel;
    control_allocation_integrator += (TUNING_INTEGRATOR_SCALAR * sample_period) * error_vel;

    float desired_voltage = (TUNING_KP * error_vel) + control_allocation_integrator;

    float batt_voltage = batt_voltage_read();
    if (desired_voltage > batt_voltage) {
        return batt_voltage;
    }
    else if (desired_voltage < -batt_voltage)
    {
       return -batt_voltage;
    }
    else {
        return desired_voltage;
    }
    

}
