#ifndef ROBOTIC_ARM_H
#define ROBOTIC_ARM_H

#include "servo_control.h"

/**
 * @number: Number of servos in robotic arm (uint8_t)
 * @servos: Servos in robotic arm (servo*)
 */
typedef struct robotic_arm {
    uint8_t number;
    servo* servos;
} robotic_arm;

/**
 * @number: Number of servos to move (uint8_t)
 * @indexes: Indexes of servos to move (uint8_t*)
 * @angles: Target angles (float*)
 */
typedef struct robotic_arm_signal {
    uint8_t number;
    uint8_t* indexes;
    float* angles;
} robotic_arm_signal;


/**
 * Macro to iterate servos from a robotic arm.
 * 
 * @robot_ptr: Pointer to exist robotic arm (robotic_arm*)
 * @servo_ptr: Exist servo pointer to iterate (servo*)
 * @tmp: Exist variable to iterate index of servos (uint8_t)
 */
#define ROBOTIC_ARM_SERVO_ITER(robot_ptr, servo_ptr, tmp)   \
for(tmp = 0, servo_ptr = &(robot_ptr)->servos[tmp]; servo_ptr; tmp += 1, servo_ptr = (tmp < robot_ptr->number) ? &(robot_ptr)->servos[tmp] : NULL)

/**
 * Create a robotic arm that have number of servos.
 * 
 * @number: Number of servos in robotic arm
 */
robotic_arm* robotic_arm_create(uint8_t number);

/**
 * Set GPIO pin of a robotic arm servo.
 * 
 * @robot: Robotic arm to set
 * @index: Index of servo in robotic arm to set
 * @pin: GPIO pin connected to the servo, must support hardware PWM
 */
void robotic_arm_set_servo_pin(robotic_arm* robot, uint8_t index, uint pin);

/**
 * Set datasheet of a robotic arm servo from source.
 * 
 * @robot: Robotic arm to set
 * @index: Index of servo in robotic arm to set
 * @source: Servo to copy datasheet
 */
void robotic_arm_set_servo_datasheet(robotic_arm* robot, uint8_t index, servo* source);

/**
 * Set limits for a robotic arm servo.
 * 
 * @robot: Robotic arm to set
 * @index: Index of servo in robotic arm to set
 * @angle_lower_bound: Limit of the lowest angle the servo can move
 * @angle_upper_bound: Limit of the highest angle the servo can move
 */
void robotic_arm_set_servo_limits(robotic_arm* robot, uint8_t index, float angle_lower_bound, float angle_upper_bound);

/**
 * Set a robotic arm servo to angle immediately.
 * 
 * @robot: Robotic arm to set
 * @index: Index of servo in robotic arm to set
 * @angle: Target angle
 */
void robotic_arm_set_servo_angle(robotic_arm* robot, uint8_t index, float angle);

/**
 * Start robotic arm.
 * Make sure all servos are properly set before calling this.
 * 
 * @robot: Robotic arm to start
 */
void robotic_arm_start(robotic_arm* robot);

/**
 * Smoothly move a robotic arm servo to angle.
 * 
 * @robot: Robotic arm to move
 * @index: Index of servo in robotic arm to move
 * @angle: Target angle
 */
void robotic_arm_move_servo(robotic_arm* robot, uint8_t index, float angle);

/**
 * Smoothly move multiple robotic arm servos to angles at once.
 * 
 * @robot: Robotic arm to move
 * @signal: Control signal
 */
void robotic_arm_move(robotic_arm* robot, robotic_arm_signal* signal);

/**
 * Print index and angle of a robotic arm servo
 * 
 * @robot: Robotic arm to print
 * @index: Index of servo in robotic arm to print
 */
void robotic_arm_print_servo(robotic_arm* robot, uint8_t index);

/**
 * Print all indexes and angles of robotic arm servos
 * 
 * @robot: Robotic arm to print
 */
void robotic_arm_print(robotic_arm* robot);

/**
 * Free memory malloced by robotic_arm_create().
 * 
 * @robot: Robotic arm to free
 */
void robotic_arm_free(robotic_arm* robot);

/**
 * Transfer string to robotic arm control signal.
 * Make sure signal->indexes and signal->angles are allocated before calling this.
 * 
 * @signal: Robotic arm control signal to set
 * @str: String to transfer, format is "number index angle index angle ..."
 */
void robotic_arm_signal_from_string(robotic_arm_signal* signal, char* str);

/**
 * Smoothly move robotic arm servos by string.
 * 
 * @robot: Robotic arm to move
 * @str: String to move robotic arm, format is "number index angle index angle ..."
 */
void robotic_arm_move_by_string(robotic_arm* robot, char* str);


#endif  // ROBOTIC_ARM_H