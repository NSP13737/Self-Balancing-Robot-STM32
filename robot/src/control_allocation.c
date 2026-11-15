#include "batt_voltage.h"
#include "tuning_parameters.h"

float controlAllocation(float LQR_vel, float current_vel, float sample_period) {
    static float z = 0; // state of integrator

    float error_vel = LQR_vel - current_vel;
    z += (TUNING_INTEGRATOR_SCALAR * sample_period) * error_vel;

    float desired_voltage = (TUNING_KP * error_vel) + z;

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
