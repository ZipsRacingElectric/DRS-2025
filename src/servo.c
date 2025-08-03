#include "servo.h"
#include "hal.h"
#include "controls/lerp.h"

// REVIEW(Barach): These values are specific to the servo in use, so they'll need moved into the EEPROM for when we test with
// the actual wing servo. (I might've said this was fine as a constant earlier, my bad).
#define T_MIN 90
#define T_MAX 211

// REVIEW(Barach): Instead of a global instance of theta_config_t, this should be a global pointer to a theta_config_t struct
// (this means it cannot have default values). This is elaborated on a bit more in main.c.
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