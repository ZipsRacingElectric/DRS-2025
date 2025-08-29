#ifndef PERIPHERALS_H
#define PERIPHERALS_H

// COMMUNICATION FROM EEPROM TO CAN
// Macros:
#define DRS_CAN_ID_COMMAND 0x754 //(command) or 0x755 (response) // use command address for json
#define DRS_THREAD_PERIOD TIME_MS2I(250) // The amount of time for a thread to communicate
#define EEPROM_MAGIC_STRING "DRS-2025" // Just the name for some string to act as a key
#define MC24LC32_ADDRS 0x50 // EEPROM I2C Memory Address

// Standard I2C config for the board
static const I2CConfig I2C1_CONFIG = 
{
	.op_mode = OPMODE_I2C,
	.clock_speed = 400000,
	.duty_cycle = FAST_DUTY_CYCLE_2
}; 

// Configuration for the EEPROM, uses the normal driver for the STM
static const mc24lc32Config_t EEPROM_CONFIG = {
	.addr = MC24LC32_ADDRS, 
	.i2c = &I2CD1,
	.timeout = TIME_MS2I(500),
	.magicString = EEPROM_MAGIC_STRING,
}; 


#endif