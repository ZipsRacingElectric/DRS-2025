// Review Notes ---------------------------------------------------------------------------------------------------------------
//
// REVIEW(Barach): I would consider creating a set of peripherals.c / .h files to clean up the code a bit more. In general, the
// peripherals.c / .h should contain any code related to the board's hardware (configs, servo, current sensor, EEPROM, CAN1,
// etc.). I put a couple of notes on things I'd consider moving.
//
// REVIEW(Barach): While the EEPROM is being initialized and used correctly (eepromHandleCanCommand) the data within it isn't
// actually used anywhere. For instance, THETA_MIN in the EEPROM (addr 0x0010) doesn't interract with theta_min in servo.c,
// they're just 2 different things with the same name right now. Take a look at the BMS's eeprom_map.c / .h files for how the
// mapping struct should be defined. After the EEPROM is initialized (mc24lc32Init), the struct needs cast on top of the
// EEPROM's cache (this is done is peripherals.c for the BMS).
//
// REVIEW(Barach): Could do for more documentation on the servo behavior, variables names, and state meaning. Both you and I
// know as we worked on this together, but future developers will be lost. Some parts I'd say need attention
// - Meanings and units of the fields in theta_config.
// - Meanings of the State enum.
// - Control flow in the main loop (lines 305 to 370).

// Includes -------------------------------------------------------------------------------------------------------------------

// Includes
#include "servo.h"
#include "debug.h"

// ChibiOS
#include "ch.h"
#include "hal.h"

#include "controls/lerp.h"
#include "peripherals/stm_adc.h"
#include "peripherals/analog_linear.h"

#include "peripherals/mc24lc32.c"
#include "can/eeprom_can.c"

// CAN Configuration ----------------------------------------------------------------------------------------------------------

/**
 * @brief Configuration of the CAN 1 peripheral.
 * @note See section 32.9 of the STM32F405 Reference Manual for more details.
 */
static const CANConfig can1Config =
{
	.mcr = 	CAN_MCR_ABOM |		// Automatic bus-off management.
			CAN_MCR_AWUM |		// Automatic wakeup mode.
			CAN_MCR_TXFP,		// Chronologic FIFI priority.
	.btr =	CAN_BTR_SJW(0) |	// Max 1 TQ resynchronization jump.
			CAN_BTR_TS2(1) |	// 2 TQ for time segment 2
			CAN_BTR_TS1(10) |	// 11 TQ for time segment 1
			CAN_BTR_BRP(2)		// Baudrate divisor of 3 (1 Mbps)
};

#define SCALEFACTOR 0.02442002442002442
#define SCALEFACTORSW 0.005493247882810711

float appsOnePercent;
float brakeRearPercent;
float steeringWheel;

// Entrypoint -----------------------------------------------------------------------------------------------------------------

static const PWMConfig pwm3Config = 
{
	.frequency = 100000, //80000, F_s
	.period = 1000, //N
	.callback = NULL,
	.channels = 
	{
		{
			.mode = PWM_OUTPUT_DISABLED,
			.callback = NULL
		},
		{
			.mode = PWM_OUTPUT_DISABLED,
			.callback = NULL
		},
		{
			.mode = PWM_OUTPUT_ACTIVE_LOW,
			.callback = NULL
		},
		{
			.mode = PWM_OUTPUT_DISABLED,
			.callback = NULL
		}
	}
};

void handleMessage(CANRxFrame* frame)
{
	appsOnePercent = (((frame->data8[1] & 0b1111) << 8) | frame->data8[0]) * SCALEFACTOR;
	brakeRearPercent = (frame->data8[4] >> 4 | (frame->data8[5] << 4)) * SCALEFACTOR;
	steeringWheel = ((int16_t) ((frame->data8[7] << 8) | frame->data8[6])) * SCALEFACTORSW;
}



/**
 * @brief Initializes the mc24lc32 and i2c drivers to prepare for CAN
 * communication
 * 
 * @note If EEPROM fails at initializing, hardFaultCallback() will be called 
 */
void mc24lc32_i2c_init() 
{
	// Ensure I2C works alright, i2cStart is a chibios function
	// See pg.259 of the ChibiOS HAL Reference Manual for more info
	if (i2cStart (&I2CD1, &I2C1_CONFIG) != MSG_OK)
		hardFaultCallback();

	// Ensure EEPROM init works alright and driver hasn't failed
	// See ref. for more info
	// 		* mc24lc32Init() -> common/src/peripherals/mc24lc32.c 
	//  	* MC24LC32_STATE_FAILED -> common/src/peripherals/mc24lc32.h (Other ENUM values are there as well)
	if (!mc24lc32Init (&eeprom_driver, &EEPROM_CONFIG) && eeprom_driver.state == MC24LC32_STATE_FAILED)
		hardFaultCallback(); 

	// "Read" the eeprom's data (or in other words): 
	// Cast then assign the mc24lc32_t object's member variable "cache" to theta_vals
	theta_vals = (theta_values_t*) eeprom_driver.cache;

	// Breakpoint yessir
	__BKPT__();
}


/**
 * @brief Create a thread to read/send CAN Frames
 * 		  This is mainly ChibiOS stuff
 * 		  See ref. src/can/can_thread.h for more info.
 * 
 * @todo Clean this up last using more simple code, look to BMS-2025, blob/main/src/can_vehicle.c
 */
static THD_WORKING_AREA (canthreadWa, 512);

THD_FUNCTION (canthread, arg)
{
	(void) arg;
	while (true)
	{
		CANRxFrame obj;

		canReceiveTimeout(&CAND1, CAN_ANY_MAILBOX, &obj, TIME_INFINITE);

		if (obj.SID == 0x600)
		{
			handleMessage(&obj);
		}

		if (obj.SID == DRS_CAN_ID_COMMAND) //command
		{
			eepromHandleCanCommand (&obj, &CAND1, (eeprom_t*) &eeprom_driver); 
		}
	}
}

/**
 * @brief Returns a new boolean state based on high, low, and val. 
 * 	If state = 0, val >= high
 * 
 * true-axis				-----------------	state = 1
 * 							|       |
 * 							|		|
 * 							|		|
 * false axis			-------------			state = 0
 * 						   low		high
 * 
 * @param val    the x value that you plug in
 * @param high   the value that takes state to 1 if x >= high && state = 0
 * @param low    the value that takes state to 0 if x <= low  && state = 1
 * @param state  the initial state before hysteresis takes place
 * @return true  the new state = 1 after hysteresis function
 * @return false the new state = 0 after hysteresis function
 */
bool hysteresis(float val, float high, float low, bool state)
{
    if (state == 0 && val >= high) return 1;
    if (state == 1 && val <= low) return 0;
    return state;
}

/**
 * @brief Semi-stub function for returning a condition on how the wing
 * may be and if it is currently in an "opening" true or "closing" false state
 * (NOT NECESSARILY A State struct just yet)
 * 
 * @return true -> wing opens up (turns clockwise in our current example)
 * @return false -> wing closes (turns counterclosewise in our current example as well)
 */
bool getWingState(void)
{
	return hysteresis (appsOnePercent, theta_vals->high_thres, theta_vals->low_thres, open);
}

int main (void)
{
	// ChibiOS Initialization
	halInit ();
	chSysInit ();

	// Debug Initialization
	ioline_t ledLine = LINE_LED_HEARTBEAT;
	debugHeartbeatStart (&ledLine, LOWPRIO);

	// CAN Initialization
	canStart (&CAND1, &can1Config);
	palClearLine (LINE_CAN1_STBY);
	pwmStart (&PWMD3, &pwm3Config);

	chThdCreateStatic(&canthreadWa, sizeof(canthreadWa), NORMALPRIO, &canthread, NULL);

	bool open = 0;

	enum State {
		DRS_OPEN_STEADY,
		DRS_OPEN_TRANSITION,
		DRS_CLOSED_STEADY,
		DRS_CLOSED_TRANSITION,
		DRS_STALL
	};

	enum State lastState;

	bool stall = false;

	const float R_SHUNT = 0.1;
	const float CURRENT_GAIN = 50;

	int stall_cur_count = 0; // current stall counter

	// both in milliseconds
	float sleep_amount = 10;
	float t_stall = stall_max_count * sleep_amount;
	//


	float angle_backoff_prime = t_stall * theta_vals->angle_step + theta_vals->angle_b;
	float theta_actual = theta_vals->angle_target - t_stall * theta_vals->angle_step;

	while (true)
	{
		open = getWingState();


		if (stall)
		{
			theta_vals->angle_backoff = 
			(lastState == DRS_OPEN_TRANSITION || lastState == DRS_OPEN_STEADY) 
				? theta_vals->angle_target - angle_backoff_prime 
				: theta_vals->angle_target + angle_backoff_prime;

			theta_vals->angle_backoff = (theta_vals->angle_backoff < theta_vals->angle_closed) 
				? theta_vals->theta_closed 
				: theta_vals->angle_backoff;

			servoSetPosition (theta_vals->angle_backoff, theta_vals->angle_min, theta_vals->angle_max);
			
		} else {
			stall = checkStall();


			if (open)
			{
				theta_vals->angle_target += theta_vals->angle_step;
				lastState = DRS_OPEN_TRANSITION;

				if (theta_vals->angle_target > theta_vals->angle_open)
				{

					theta_vals->angle_target = theta_vals->angle_open;
					lastState = DRS_OPEN_STEADY;

				}
			} else {

				theta_vals->angle_target -= theta_vals->angle_step;
				lastState = DRS_CLOSED_TRANSITION;

				if (theta_vals->angle_target < theta_vals->angle_closed)
				{

					theta_vals->angle_target = theta_vals->angle_closed;
					lastState = DRS_CLOSED_STEADY;

				}
			}
			
			servoSetPosition (theta_vals->angle_target, theta_vals->angle_min, theta_vals->angle_max);		
		}

		chThdSleepMilliseconds (sleep_amount);
	}

	
}

// Callbacks ------------------------------------------------------------------------------------------------------------------

void hardFaultCallback (void)
{
	// TODO(Barach): Fault behavior
	while (true)
	{}
}