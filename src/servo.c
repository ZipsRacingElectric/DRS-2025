#include "servo.h"
#include "hal.h"
#include "controls/lerp.h"

#define T_MIN 90
#define T_MAX 211

void servoSetPosition( float theta )
{
    uint16_t n = lerp2d(theta, THETA_MIN, T_MIN, THETA_MAX, T_MAX);

    pwmEnableChannel (&PWMD3, 2, n);
}