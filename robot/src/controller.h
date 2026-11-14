#ifndef CONTROLLER_H
#define CONTROLLER_H

void ctrl_init(void);

/* Update state vector elements. */
void ctrl_set_x1(float x1);
float ctrl_get_x1(void);
void ctrl_set_x2(float x2);
void ctrl_set_x3(float x3);
float ctrl_get_z_state(void);
//void ctrl_set_x4(float x4);

float getControl(void);

void ctrl_update(void);

#endif
