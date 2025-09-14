
#include <stdint.h>

#ifndef SERVO_H
#define SERVO_H


/**
 * @brief Basic struct variable definitions and documentation for theta_vals.
 * 
 * See servo.c for more details
 */
typedef struct {
    uint8_t pad [16]; // padding done here to match up with application-ware CAN TOOLS 25 repo in json config file
    
    // all of the units in here are measured in degrees
    float angle_min;         // absolute minimum angle 
    float angle_max;         // absolute maximum angle 
    float angle_step;        // the angle that the servo moves each loop of the program
    float angle_b;           // the initial in real life servo position
    float angle_backoff;     // the amount that the servo should go back if a stall condition is met
    float angle_target;      // the calculated "predicted" angle that the servo should go to
    float angle_open;        // the threshold angle that makes "opened"
    float angle_closed;      // the threshold angle that makes "closed"
    //

    float high_thres;        // the hysteresis function lowest value, the "high" parameter variable  
    float low_thres;         // the hysteresis function highest value, the "low" parameter variable
    float i_stall;           // The current (measured in amps) that constitutes a stall condition
    uint8_t stall_max_count; // The maximum amount of times that the servo can count to before a stall condition is met
    uint32_t servo_pwm_min;  // the pwm period that makes the absolute minimum angle
    uint32_t servo_pwm_max;  // the pwm period that makes the absolute maximum angle

} theta_values_t;

extern theta_values_t* theta_vals;

/**
 * @brief Sets the servo position based with linear interpolation between the angle minimum and angle maximum
 *        provided.
 * 
 * @param angle     The angle inbetween minimum and maximum that will be chosen
 * @param angle_min The minimum angle that can be possible
 * @param angle_max The maximum angle that can be possible
 */
void servoSetPosition( float angle, float angle_min, float angle_max );

#endif