#ifndef BATT_VOLTAGE_H
#define BATT_VOLTAGE_H

/* Initializes the battery voltage input on PC5 / ADC1_IN15. */
void batt_voltage_init(void);

/* Reads battery voltage in volts, accounting for the 4:1 divider. */
float batt_voltage_read(void);

#endif
