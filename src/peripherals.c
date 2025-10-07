#include "peripherals.h"

// REVEIW(Barach): Documentation. What are these and where do they come from?
const float R_SHUNT = 0.1;
const float CURRENT_GAIN = 50;

// REVEIW(Barach): Non-external global variables should always be static to prevent linker collision.
int stall_cur_count = 0; // current stall counter

mc24lc32_t eeprom_driver;
linearSensor_t output_current;

// REVEIW(Barach): If these values aren't changing, the struct should be constant.
// LinearSensor configuration to be used by the adc later in the code, more explained in ADC part
linearSensorConfig_t output_current_config = {
    .sampleMin = 0,
    .valueMin = 0,
    .sampleMax = 4095, // 2^12 - 1 for a 12 bit ADC
    // i_max = (V_max / X_max) / (R_shunt * current_gain) * x 
    .valueMax = (3.3 / (R_SHUNT * CURRENT_GAIN)) //3.3 is the voltage max
};

// REVEIW(Barach): If these values aren't changing, the struct should be constant.
// ADC configuration for the given STM chip, configured to read the output current of the shunt resistor
// for reading a potential stall condition
stmAdc_t adc;
stmAdcConfig_t adc_config = {
    .driver = &ADCD1, 
    .channels = 
    {
        ADC_CHANNEL_IN10
    },
    .channelCount = 1,
    .sensors = 
    {
        (analogSensor_t*) &output_current
    }
};

/**
 * @brief Returns stall after checking if stall conditions are met
 * 
 * @note &adc MUST BE INITIALIZED FIRST BEFORE USING, USE peripheralsInit()
 *            in the same source code where this function is declared.
 *              Directory / Location: DRS-2025\src\peripherals.c
 * 
 * @return true  -> stall HAS been met
 * @return false -> stall has NOT been met
 */

// REVEIW(Barach): The value of stall isn't used within this function, and the result is already returned, so why are we
//   passing a pointer in here?
bool checkStall(bool* stall)
{
    // Gather a sample of the adc 
    // Reference in stm_adc.h for more information
    stmAdcSample (&adc);

	// REVEIW(Barach): Actual names are always preferred to symbolic representations. Ex:
	//   v => voltage.
	//   i => current.
	// Name should also contain what current it is. Is this for the whole board, the ADC, or just the servo?
	// Ex. motorCurrent or inverterCurrent.

    // i = output_current.value, where "i" is the current leaving the servo or the current consumed by
    // the servo
    float i = output_current.value;

    // if "i" is greater than the stall, increment the current counter, otherwise set to false
    if (i >= theta_vals->i_stall)
    {
        ++stall_cur_count;
    }
    else {
        stall_cur_count = 0;
    }

    // If "i" is greater or equal to the max stall count make stall true
    if (stall_cur_count >= theta_vals->stall_max_count)
    {
        (*stall) = true;
    }

    return *stall;
}

/**
 * @brief Initializes all sensors and EEPROM to be read and written to
 * 
 * Current rendition:
 *  - Includes linearSensor and stmAdc to read current from shunt resistor
 *  - Also has EEPROM inits to read and write from and to.
 */
void peripheralsInit(void)
{
    // Custom functions to initialize both the linear sensor and ADC for reading shunt resistor current
    // See analog_linear.h in common/src/peripherals and stm_adc.h in common/src/peripherals for more details
    linearSensorInit (&output_current, &output_current_config);
    stmAdcInit (&adc, &adc_config);

    // Custom function to initialize I2C communication with MC24LC32 EEPROM
    // Definition is in DRS-2025\src\main.c
    mc24lc32_i2c_init();
}