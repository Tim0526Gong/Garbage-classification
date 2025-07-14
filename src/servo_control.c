#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "servo_control.h"
#include <math.h>


const uint max_servo_move_ms = 5000;    // Maximum time (ms) for a servo smooth move

/**
 * Calculate the number of steps needed for the smooth transition.
 * 
 * @angle_ratio: Ratio of difference and maximum angle (0 to 1)
 * @period: Period of PWM signal (us)
 */
uint calculate_steps(float angle_ratio, uint period) {
    return (uint)fabs(angle_ratio * 1e3 * max_servo_move_ms / period);
}

// Calculate the smooth transition ratio using a cosine function for easing effect
float calculate_smooth_ratio(float ratio_of_steps) {
    return 0.5 - cosf(M_PI * ratio_of_steps) / 2;
}

/**
 * Initialize a single servo motor.
 * Make sure all fields in motor are correctly set before calling this.
 * 
 * @motor: Servo to initialize
 */
void servo_init(servo* motor) {
    gpio_set_function(motor->pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(motor->pin);
    // 1e6 for convert period (us) to frequency (Hz)
    float clock_devider = (float)SYSTEM_CLOCK / ((float)1e6 / motor->period) / SERVO_PWM_WRAP;
    pwm_set_clkdiv(slice_num, clock_devider);
    pwm_set_wrap(slice_num, SERVO_PWM_WRAP - 1);
    servo_set_angle(motor, motor->angle);
    pwm_set_enabled(slice_num, true);
}

/**
 * Set GPIO pin of a servo motor.
 * 
 * @motor: Servo to set pin
 * @pin: GPIO pin connected to the servo, must support hardware PWM
 */
void servo_set_pin(servo* motor, uint pin) {
    motor->pin = pin;
}

/**
 * Set datasheet of a servo.
 * 
 * @motor: Servo to set
 * @angle_range: Range of angle the servo can move, usually 180 degrees
 * @period: PWM signal period (us)
 * @min_duty: Duty cycle at 0 degree (us)
 * @max_duty: Duty cycle at 180 degree (us)
 */
void servo_set_datasheet(servo* motor, float angle_range, uint period, uint min_duty, uint max_duty) {
    motor->angle_range = angle_range;
    motor->period = period;
    motor->min_duty = min_duty;
    motor->max_duty = max_duty;
}

/**
 * Set limits for servo angles.
 * 
 * @motor: Servo to set limits
 * @angle_lower_bound: Limit of the lowest angle the servo can move
 * @angle_upper_bound: Limit of the highest angle the servo can move
 */
void servo_set_limits(servo* motor, float angle_lower_bound, float angle_upper_bound) {
    motor->angle_lower_bound = angle_lower_bound;
    motor->angle_upper_bound = angle_upper_bound;
}

/**
 * Set the angle of a single servo motor immediately.
 * 
 * @motor: Servo to set angle
 * @angle: Target angle in degrees
 */
void servo_set_angle(servo* motor, float angle) {
    if(angle < motor->angle_lower_bound)
        angle = motor->angle_lower_bound;
    else if(angle > motor->angle_upper_bound)
        angle = motor->angle_upper_bound;
    float duty = (motor->angle / motor->angle_range) * (motor->max_duty - motor->min_duty) + motor->min_duty;
    uint16_t level = duty / motor->period * SERVO_PWM_WRAP;
    pwm_set_gpio_level(motor->pin, level);
    motor->angle = angle;
}

/**
 * Move a single servo motor smoothly to the target angle.
 * 
 * @motor: Servo to move
 * @angle: Target angle in degrees
 */
void servo_smooth(servo* motor, float angle) {
    float start_angle = motor->angle;
    float angle_difference = angle - motor->angle;
    uint steps = calculate_steps(angle_difference / motor->angle_range, motor->period);
    for(uint step = 1; step < steps; step++) {
        // Calculate the smooth transition ratio using a cosine function for easing effect
        float ratio = calculate_smooth_ratio((float)step / steps);
        float delta = angle_difference * ratio;
        servo_set_angle(motor, start_angle + delta);
        sleep_us(motor->period);
    }
    servo_set_angle(motor, angle);
    motor->angle = angle;
}

/**
 * Initialize multiple servo motors.
 * Make sure all servo structs are properly set before calling this.
 * 
 * @number: Number of servos to initialize
 * @motors: Servos to initialize
 */
void servos_init(uint number, servo** motors) {
    for(uint i = 0; i < number; i++) {
        gpio_set_function(motors[i]->pin, GPIO_FUNC_PWM);
        uint slice_num = pwm_gpio_to_slice_num(motors[i]->pin);
        // 1e6 for convert period (us) to frequency (Hz)
        float clock_devider = (float)SYSTEM_CLOCK / ((float)1e6 / motors[i]->period) / SERVO_PWM_WRAP;
        pwm_set_clkdiv(slice_num, clock_devider);
        pwm_set_wrap(slice_num, SERVO_PWM_WRAP - 1);
        servo_set_angle(motors[i], motors[i]->angle);
    }
    for(uint i = 0; i < number; i++) {
        uint slice_num = pwm_gpio_to_slice_num(motors[i]->pin);
        pwm_set_enabled(slice_num, true);
    }
}

/**
 * Set angles for multiple servos immediately.
 * 
 * @number: Number of servos to set angles
 * @motors: Servos to set angles
 * @angles: Target angles in degrees
 */
void servos_set_angle(uint number, servo** motors, float *angles) {
    for(uint i = 0; i < number; i++) {
        servo_set_angle(motors[i], angles[i]);
    }
}

/**
 * Smoothly move multiple servos to target angles.
 * 
 * @number: Number of servos to move
 * @motors: Servos to move
 * @angles: Target angles in degrees
 */
void servos_smooth(uint number, servo** motors, float *angles) {
    float start_angles[number];
    float angle_differences[number];
    uint max_steps = 0;
    uint max_period = 1;
    // Store start angles and angle differences for each servo
    // Determine the max steps and max period
    for(uint i = 0; i < number; i++) {
        start_angles[i] = motors[i]->angle;
        angle_differences[i] = angles[i] - start_angles[i];
        uint steps = calculate_steps(angle_differences[i] / motors[i]->angle_range, motors[i]->period);
        if(steps > max_steps)
            max_steps = steps;
        if(motors[i]->period > max_period)
            max_period = motors[i]->period;
    }
    // Perform the smooth transition in steps
    for(uint step = 1; step < max_steps; step++) {
        // Calculate the smooth transition ratio using a cosine function for easing effect
        float ratio = calculate_smooth_ratio((float)step / max_steps);
        // Set the servo angles based on the start angles and angle differences with ratio
        for(uint i = 0; i < number; i++) {
            float delta = angle_differences[i] * ratio;
            servo_set_angle(motors[i], start_angles[i] + delta);
        }
        sleep_us(max_period);
    }
    servos_set_angle(number, motors, angles);
}