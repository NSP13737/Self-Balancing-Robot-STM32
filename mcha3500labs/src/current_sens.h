#ifndef CURRENT_SENS_H
#define CURRENT_SENS_H

/* Initializes ADC1 and both current-sense inputs (PC4 and PA4). */
void current_sense_init(void);

/* Left sensor: PC4 / ADC1_IN14. Right sensor: PA4 / ADC1_IN4. */
float left_current_read(void);
float right_current_read(void);

#endif
