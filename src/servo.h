#pragma once

/*
float theta_min = -45;
float theta_max = 45;

void servoSetPosition (float theta);
*/

// REVIEW(Barach): Documentation of meanings and units (ex theta_min is absolute minimum angle in degrees).
typedef struct {
    float theta_min;
    float theta_max;
    float theta_step;
    float theta_b;
    float theta_backoff;
    float theta_target;
    float theta_open;
    float theta_closed;
} theta_config_t;

extern theta_config_t theta_config;

// REVIEW(Barach): Unused code
// theta_config_t theta_config = {
//     theta_min = -45,
//     theta_max = 45,
//     theta_step = 0.25,
//     theta_b = 15,
//     theta_backoff = -20,
//     theta_target = -20,
//     theta_open = 20,
//     theta_closed = -20
// };

void servoSetPosition( float theta, float theta_min, float theta_max );