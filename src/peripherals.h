#include "peripherals/i2c/mc24lc32.h"
#include "peripherals/adc/analog_linear.h"
#include "peripherals/adc/stm_adc.h"
#include "servo.h"

#ifndef PERIPHERALS_H
#define PERIPHERALS_H

// COMMUNICATION FROM EEPROM TO CAN
// Macros:
#define DRS_CAN_ID_COMMAND 0x754 //(command) or 0x755 (response) // use command address for json
#define DRS_THREAD_PERIOD TIME_MS2I(250) // The amount of time for a thread to communicate
#define EEPROM_MAGIC_STRING "DRS-2025" // Just the name for some string to act as a key
#define MC24LC32_ADDRS 0x50 // EEPROM I2C Memory Address

// Global-ish Variables

// Custom ADC and Sensor objects to read shunt resistor to gauge where stall may have taken place
// See ref. in 
//           * stm_adc.h in common/src/peripherals
//           * analog_linear.h in common/src/peripherals
extern mc24lc32_t eeprom_driver;
extern linearSensor_t output_current;

// REVIEW(Barach): Should almost never have static variables defined in a header. Move into a *.c file.

// Standard I2C config for the board
static const I2CConfig I2C1_CONFIG = 
{
	.op_mode = OPMODE_I2C,
	.clock_speed = 400000,
	.duty_cycle = FAST_DUTY_CYCLE_2
}; 

// REVIEW(Barach): Should almost never have static variables defined in a header. Move into a *.c file.

// Configuration for the EEPROM, uses the normal driver for the STM
static const mc24lc32Config_t EEPROM_CONFIG = {
	.addr = MC24LC32_ADDRS, 
	.i2c = &I2CD1,
	.timeout = TIME_MS2I(500),
	.magicString = EEPROM_MAGIC_STRING,
	.dirtyHook = NULL
}; 

// REVIEW(Barach): This should be called by peripheralsInit so it doesn't need to be defined in a header.

/**
 * @brief Initialize the mc24lc32 with I2C communication
 * @todo Better detailed documentation would be nice
 */
void mc24lc32_i2c_init(void);

/**
 * @brief Checks if stall conditions have been met and returns if so 
 * 
 * @param stall 
 * @return true stall has been met
 * @return false stall hasn't been met
 */
bool checkStall(bool* stall);

/**
 * @brief Initializes all sensors and EEPROM to be read and written to
 * 
 * Current rendition:
 *  - Includes linearSensor and stmAdc to read current from shunt resistor
 *  - Also has EEPROM inits to read and write from and to.
 */
void peripheralsInit(void);

#endif