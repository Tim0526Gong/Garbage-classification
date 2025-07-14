#include <stdio.h>
#include "pico/stdlib.h"
#include "robotic_arm.h"
#include <stdlib.h>


/**
 * Create a robotic arm that have number of servos.
 * 
 * @number: Number of servos in robotic arm
 */
robotic_arm* robotic_arm_create(uint8_t number) {
    robotic_arm* robot = malloc(sizeof(robotic_arm));
    if(!robot) {
        fprintf(stderr, "Robotic arm malloc failed.\n");
        return NULL;
    }
    robot->number = number;
    robot->servos = malloc(number * sizeof(servo));
    if(!robot->servos) {
        fprintf(stderr, "Robotic arm servos malloc failed.\n");
        free(robot);
        return NULL;
    }
    return robot;
}

/**
 * Set GPIO pin of a robotic arm servo.
 * 
 * @robot: Robotic arm to set
 * @index: Index of servo in robotic arm to set
 * @pin: GPIO pin connected to the servo, must support hardware PWM
 */
void robotic_arm_set_servo_pin(robotic_arm* robot, uint8_t index, uint pin) {
    if(index >= robot->number) {
        fprintf(stderr, "Index out of range.\n");
        return ;
    }
    robot->servos[index].pin = pin;
}

/**
 * Set datasheet of a robotic arm servo from source.
 * 
 * @robot: Robotic arm to set
 * @index: Index of servo in robotic arm to set
 * @source: Servo to copy datasheet
 */
void robotic_arm_set_servo_datasheet(robotic_arm* robot, uint8_t index, servo* source) {
    if(index >= robot->number) {
        fprintf(stderr, "Index out of range.\n");
        return ;
    }
    SERVO_DATASHEET_COPY(&robot->servos[index], source);
}
/**
 * Set limits for a robotic arm servo.
 * 
 * @robot: Robotic arm to set
 * @index: Index of servo in robotic arm to set
 * @angle_lower_bound: Limit of the lowest angle the servo can move
 * @angle_upper_bound: Limit of the highest angle the servo can move
 */
void robotic_arm_set_servo_limits(robotic_arm* robot, uint8_t index, float angle_lower_bound, float angle_upper_bound) {
    if(index >= robot->number) {
        fprintf(stderr, "Index out of range.\n");
        return ;
    }
    servo_set_limits(&robot->servos[index], angle_lower_bound, angle_upper_bound);
}

/**
 * Set a robotic arm servo to angle immediately.
 * 
 * @robot: Robotic arm to set
 * @index: Index of servo in robotic arm to set
 * @angle: Target angle
 */
void robotic_arm_set_servo_angle(robotic_arm* robot, uint8_t index, float angle) {
    if(index >= robot->number) {
        fprintf(stderr, "Index out of range.\n");
        return ;
    }
    servo_set_angle(&robot->servos[index], angle);
}

/**
 * Start robotic arm.
 * Make sure all servos are properly set before calling this.
 * 
 * @robot: Robotic arm to start
 */
void robotic_arm_start(robotic_arm* robot) {
    servo* servos[robot->number];
    for(uint8_t i = 0; i < robot->number; i++) {
        servos[i] = &robot->servos[i];
    }
    // Initialize all servos
    servos_init(robot->number, servos);
}

/**
 * Smoothly move a robotic arm servo to angle.
 * 
 * @robot: Robotic arm to move
 * @index: Index of servo in robotic arm to move
 * @angle: Target angle
 */
void robotic_arm_move_servo(robotic_arm* robot, uint8_t index, float angle) {
    if(index >= robot->number) {
        fprintf(stderr, "Index out of range.\n");
        return ;
    }
    servo_smooth(&robot->servos[index], angle);
}

/**
 * Smoothly move multiple robotic arm servos to angles at once.
 * 
 * @robot: Robotic arm to move
 * @signal: Control signal
 */
void robotic_arm_move(robotic_arm* robot, robotic_arm_signal* signal) {
    servo* action_servos[signal->number];
    SERVOS_PICK(action_servos, robot->servos, signal->indexes, signal->number);
    servos_smooth(signal->number, action_servos, signal->angles);
}

/**
 * Print index and angle of a robotic arm servo
 * 
 * @robot: Robotic arm to print
 * @index: Index of servo in robotic arm to print
 */
void robotic_arm_print_servo(robotic_arm* robot, uint8_t index) {
    if(index >= robot->number) {
        fprintf(stderr, "Index out of range.\n");
        return ;
    }
    printf("Robotic arm servo %d : %f degrees\n", index, robot->servos[index].angle);
}

/**
 * Print all indexes and angles of robotic arm servos
 * 
 * @robot: Robotic arm to print
 */
void robotic_arm_print(robotic_arm* robot) {
    for(uint8_t i = 0; i < robot->number; i++)
        robotic_arm_print_servo(robot, i);
}

/**
 * Free memory malloced by robotic_arm_create().
 * 
 * @robot: Robotic arm to free
 */
void robotic_arm_free(robotic_arm* robot) {
    free(robot->servos);
    free(robot);
}

/**
 * Transfer string to robotic arm control signal.
 * Make sure signal->indexes and signal->angles are allocated before calling this.
 * 
 * @signal: Robotic arm control signal to set
 * @str: String to transfer, format is "number index angle index angle ..."
 */
void robotic_arm_signal_from_string(robotic_arm_signal* signal, char* str) {
    char* endptr;
    signal->number = strtol(str, &endptr, 10);
    if (endptr == str || *endptr != ' ') {
        fprintf(stderr, "Invalid signal string format.\n");
        return;
    }
    str = endptr + 1; // Move to the next part of the string
    if (signal->number <= 0) {
        fprintf(stderr, "Invalid number in signal string.\n");
        return;
    }
    for(int i = 0; i < signal->number; i++) {
        signal->indexes[i] = strtol(str, &endptr, 10);
        if (endptr == str || *endptr != ' ') {
            fprintf(stderr, "Invalid index in signal string.\n");
            return;
        }
        str = endptr + 1; // Move to the next part of the string
        signal->angles[i] = strtof(str, &endptr);
        if (endptr == str || (*endptr != ' ' && *endptr != '\0')) {
            fprintf(stderr, "Invalid angle in signal string.\n");
            return;
        }
        str = endptr + 1; // Move to the next part of the string
    }
}

/**
 * Smoothly move robotic arm servos by string.
 * 
 * @robot: Robotic arm to move
 * @str: String to move robotic arm, format is "number index angle index angle ..."
 */
void robotic_arm_move_by_string(robotic_arm* robot, char* str) {
    robotic_arm_signal signal;
    uint8_t servo_indexes[robot->number];
    float angles[robot->number];
    signal.indexes = servo_indexes;
    signal.angles = angles;
    robotic_arm_signal_from_string(&signal, str);
    // Print the parsed signal for debugging
    printf("Parsed robotic arm signal:\n");
    printf("Number of servos: %d\n", signal.number);
    for (int i = 0; i < signal.number; i++) {
        printf("Servo %d: Index = %d, Angle = %.2f\n", i, signal.indexes[i], signal.angles[i]);
    }
    if (signal.number <= 0) {
        fprintf(stderr, "No valid servos to move.\n");
        return;
    }
    if (signal.number > robot->number) {
        fprintf(stderr, "Too many servos specified in signal.\n");
        return;
    }
    if (signal.number == 1) {
        // If only one servo is specified, move it directly
        robotic_arm_move_servo(robot, signal.indexes[0], signal.angles[0]);
        return;
    }
    robotic_arm_move(robot, &signal);
}
