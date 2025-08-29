#include "servo.h"
#include "hal.h"
#include "controls/lerp.h"

// THESE ARE TEMPLATE VALUES, THE ACTUAL VALUES MAY BE DIFFERENT
// theta_values_t theta_vals = {
//     float angle_min                = -45, 
//     float angle_max                = 45,  
//     float angle_step               = 0.25,
//     float angle_b                  = 15,  
//     float angle_backoff            = -20, 
//     float angle_target             = -20,
//     float angle_open               = 20,
//     float angle_closed             = -20,
//     float high_thres               = 60,
//     float low_thres                = 40,
//     float i_stall                  = 0.6594,
//     uint8_t stall_max_count        = 25,
//     uint32_t servo_pwm_min         = 90,
//     uint32_t servo_pwm_max         = 211
// };
theta_values_t* theta_vals;


void servoSetPosition( float angle, float angle_min, float angle_max )
{
    
    uint16_t n = lerp2d(angle, angle_min, theta_vals->servo_pwm_min, angle_max, theta_vals->servo_pwm_max);

    pwmEnableChannel (&PWMD3, 2, n);
    
}