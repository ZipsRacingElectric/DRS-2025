// Includes -------------------------------------------------------------------------------------------------------------------

// Includes
#include "servo.h"
#include "debug.h"

// ChibiOS
#include "ch.h"
#include "hal.h"

#include "controls/lerp.h"
// #include "peripherals/stm_adc.h"
#include "peripherals/adc/stm_adc.h"
// #include "peripherals/analog_linear.h"
#include "peripherals/adc/analog_linear.h"

#include "peripherals/i2c/mc24lc32.h"
#include "can/can_thread.h"
#include "can/eeprom_can.h"

#include "peripherals.h"

void hardFaultCallback (void);

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

void receiveMessage(CANRxFrame* frame)
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
void mc24lc32_i2c_init(void) 
{
	// Ensure I2C works alright, i2cStart is a chibios function
	// See pg.259 of the ChibiOS HAL Reference Manual for more info
	i2cStart (&I2CD1, &I2C1_CONFIG);
	if (i2cGetErrors(&I2CD1) != 0)
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

		if (obj.SID == 0x600) // receive
		{
			receiveMessage(&obj);
		}

		if (obj.SID == DRS_CAN_ID_COMMAND) //command
		{
			eepromHandleCanCommand (&obj, &CAND1, (eeprom_t*) &eeprom_driver); 
		}
	}
}

/**
 * @brief function to handle Can messages
 * 
 * @todo Make this like a canReceiveHandler_t
 */
void handleCanMessage(CANRxFrame obj) // passing by reference??
{
	if (obj.SID == 0x600) // receive
	{
		receiveMessage(&obj);
	}

	if (obj.SID == DRS_CAN_ID_COMMAND) //command
	{
		eepromHandleCanCommand (&obj, &CAND1, (eeprom_t*) &eeprom_driver); 
	}
}

// static const canThreadConfig_t CAN1_RX_THREAD_CONFIG =
// {
// 	.name = "drs_can1_rx",
// 	.driver = &CAND1,
// 	.period = TIME_MS2I (10),
// 	.nodes = NULL,
// 	.nodeCount = 0,
// 	.rxHandler = (canReceiveHandler_t*) &handleCanMessage,
// 	.bridgeDriver = NULL
// };


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
bool getWingState(bool open)
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

	peripheralsInit();

	// canThreadStart (&canthreadWa, sizeof(canthreadWa), NORMALPRIO, &CAN1_RX_THREAD_CONFIG);

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

	

	// both in milliseconds
	float sleep_amount = 10;
	float t_stall = theta_vals->stall_max_count * sleep_amount;
	//


	float angle_backoff_prime = t_stall * theta_vals->angle_step + theta_vals->angle_b;
	// float theta_actual = theta_vals->angle_target - t_stall * theta_vals->angle_step;

	while (true)
	{
		open = getWingState(&open);


		if (stall)
		{
			theta_vals->angle_backoff = 
			(lastState == DRS_OPEN_TRANSITION || lastState == DRS_OPEN_STEADY) 
				? theta_vals->angle_target - angle_backoff_prime 
				: theta_vals->angle_target + angle_backoff_prime;

			theta_vals->angle_backoff = (theta_vals->angle_backoff < theta_vals->angle_closed) 
				? theta_vals->angle_closed 
				: theta_vals->angle_backoff;

			servoSetPosition (theta_vals->angle_backoff, theta_vals->angle_min, theta_vals->angle_max);
			
		} else {
			stall = checkStall(&stall);


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