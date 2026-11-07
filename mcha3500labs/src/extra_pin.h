#ifndef EXTRA_PIN_H
#define EXTRA_PIN_H

/* Initializes the extra analog input on PC3 / ADC1_IN13. */
void extra_pin_init(void);

/* Reads the PC3 pin voltage in volts. */
float extra_pin_read(void);

#endif
