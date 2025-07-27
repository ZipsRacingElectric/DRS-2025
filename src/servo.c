#include "servo.h"
#include "hal.h"
#include "controls/lerp.h"

#define T_MIN 90
#define T_MAX 211

theta_config_t theta_config = {
    -45,
    45,
    0.25,
    15,
    -20,
    -20,
    20,
    -20
};

void servoSetPosition( float theta, float theta_min, float theta_max )
{
    uint16_t n = lerp2d(theta, theta_min, T_MIN, theta_max, T_MAX);

    pwmEnableChannel (&PWMD3, 2, n);

    // Since this function is called generally each loop, saving to eeprom 
    // in here as well seems ok, but done in main.c for clarity sake


}