#include "motor.h"
#include "motor_cmds.h"
#include <stdio.h>
#include <string.h>

void motorDir(int argc, char *argv[])
{
    if (argc != 3) {
        printf("Usage: motorDir <left|right> <FWD|BWD|NODIR>\n");
        return;
    }

    Motor motor;
    if (strcmp(argv[1], "left") == 0) {
        motor = MOTOR_LEFT;
    } else if (strcmp(argv[1], "right") == 0) {
        motor = MOTOR_RIGHT;
    } else {
        printf("Motor must be left or right\n");
        return;
    }

    MotorDirection direction;
    if (strcmp(argv[2], "FWD") == 0) {
        direction = MOTOR_DIR_FWD;
    } else if (strcmp(argv[2], "BWD") == 0) {
        direction = MOTOR_DIR_BWD;
    } else if (strcmp(argv[2], "NODIR") == 0) {
        direction = MOTOR_DIR_NODIR;
    } else {
        printf("Direction must be FWD, BWD, or NODIR\n");
        return;
    }

    motor_set_direction(motor, direction);
}