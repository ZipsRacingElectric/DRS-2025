// REVIEW(Barach): Way too much going on in this file. Things should be split up into the below structure:

// src
// ├── can
// │   ├── receive.c        - Should contain all function prototypes and implementations for receiving CAN messages.
// │   ├── receive.h        - Should contain the prototype for one function called 'receiveMessage'. This function should
// │   │                      implement the 'canReceiveHandler_t' signature.
// │   ├── transmit.h       - Should contain 1 prototype for each unique CAN message the DRS transmits.
// │   └── transmit.c       - Should contain 1 implementation per prototype in the header.
// ├── can.c                - Should include all configurations structs and code for initializing the CAN driver & CAN thread.
// ├── can.h                - Should include 1 function prototype called 'canInterfaceInit'.
// ├── main.c               - Should include the main functionality of the application. This can keep the wing state, target
// │                          angle, and the APPS hysteresis.
// ├── peripherals
// │   ├── eeprom_map.h     - Should define a single structure called 'eepromMap_t'. This should define the memory map of the
// │   │                      device's EEPROM.
// │   ├── servo.c          - Should only contain function implementations related to the servo.
// │   └── servo.h          - Should only contain function prototypes related to the servo.
// ├── peripherals.c        - Should contain all configurations and code for initializing board hardware (excluding CAN).
// └── peripherals.h        - Should contain a 'peripheralsInit' function prototype and an external instance of the eepromMap_t
//                            structure.

// Includes -------------------------------------------------------------------------------------------------------------------

// REVIEW(Barach): Includes need cleaned up. Too difficult to tell what functionality is actually being used here. Should be
// obvious which ones can be removed once code gets moved into the above tree.

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

// REVIEW(Barach): Documentation and better naming. What is this the scale factor for?
#define SCALEFACTOR 0.02442002442002442
#define SCALEFACTORSW 0.005493247882810711

// REVIEW(Barach): Documentation. Also longhand is always preferred unless it is obvious what the shorthand stands for. Ex.
//   Tx => transmit
//   Rx => receive
// DRS Macros
#define DRS_CAN_TRNS_ID 0x7A4
#define DRS_CAN_THTA_TRGT_SF (360.0f / 1024.0f)

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

/**
 * @brief Receives a CANRxFrame and processes out several different can signals for 
 * future processing (particularly with appsOnePercent, brakeRearPercent, etc.)
 * 
 * @param frame[in] the frame that you want to process, comes from the CAN Signal thread
 */
void receiveMessage(CANRxFrame* frame)
{
	appsOnePercent = (((frame->data8[1] & 0b1111) << 8) | frame->data8[0]) * SCALEFACTOR;
	brakeRearPercent = (frame->data8[4] >> 4 | (frame->data8[5] << 4)) * SCALEFACTOR;
	steeringWheel = ((int16_t) ((frame->data8[7] << 8) | frame->data8[6])) * SCALEFACTORSW;
}


// REVIEW(Barach): Prototype is defined in peripherals.h but implementation is defined in main.c. This should be consistent
// (only in peripherals.c / peripherals.h)

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

	// REVEIW(Barach): Misleading documentation. The read operation is done in the mc24lc32Init function, this cast is just
	// changing how we interpret the data that was read.

	// "Read" the eeprom's data (or in other words): 
	// Cast then assign the mc24lc32_t object's member variable "cache" to theta_vals
	theta_vals = (theta_values_t*) eeprom_driver.cache;

}

// REVIEW(Barach): This should be replaced with an instance from the can_thread module in the common library.
/**
 * @brief Create a thread to read/send CAN Frames
 * 		  This is mainly ChibiOS stuff
 * 		  See ref. src/can/can_thread.h for more info.
 * 
 * @todo Potential: Clean this up last using more simple code, look to BMS-2025, blob/main/src/can_vehicle.c
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
 * @brief Void Function to Handle Incoming Can Frames with both EEPROM and Can Signal 
 * receive (read) / command (write) operations
 * 
 * // REVEIW(Barach): Misleading documentation. This frame is received not sent.
 * @param obj 	The CANRxFrame that you send from the canthread to then be processed
 */
void handleCanMessage(CANRxFrame obj) 
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
 * @todo: Update code here to be based on what we'd like to include / modify
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

	// REVIEW(Barach): Should be replaced with the common library can_thread.
	chThdCreateStatic(&canthreadWa, sizeof(canthreadWa), NORMALPRIO, &canthread, NULL);

	peripheralsInit();

	// Initialize:
	// lastState to a placeholder value
	// open to 0
	// stall to 1

	// REVIEW(Barach): Documentation and naming. How is this different to DRS_OPEN_STEADY and DRS_OPEN_TRANSITION?
	bool open = 0;

	// REVIEW(Barach): Missing documentation on what these states mean.
	enum State {
		DRS_OPEN_STEADY 	  = 0,
		DRS_OPEN_TRANSITION   = 1,
		DRS_CLOSED_STEADY	  = 2,
		DRS_CLOSED_TRANSITION = 3,
		DRS_STALL			  = 4
	};

	enum State lastState = DRS_OPEN_STEADY; 

	bool stall = false;

	// REVIEW(Barach): This is using an extended CAN ID (29-bit), but the DBC file has it listed as a standard (11-bit) ID.
	//   There is no reason for this to be an extended ID.
	// Initialize con't:
	// canTransmitFrame 
	CANTxFrame canTransmitFrame = { 
		.DLC = 2,
		.RTR = 0, 
		.IDE = CAN_IDE_EXT,
		.EID = (uint32_t) DRS_CAN_TRNS_ID,
	};

	// both in milliseconds
	float sleep_amount = 10;

	// the amount of time that passes when stall is being count
	float t_stall = theta_vals->stall_max_count * sleep_amount;

	// variable to help calculate the true backoff when stalling occurs in terms of angle_step and and the
	// real world predicted placement of the servo. (Since we're counting up until a stall condition is met,
	// and the servo is still moving in that time)
	float angle_backoff_prime = t_stall * theta_vals->angle_step + theta_vals->angle_b;

	while (true)
	{
		// Read the Servo state from the function.
		open = getWingState(open);


		// Sending CAN Signals:

		// REVIEW(Barach): Misleading documentation
		// convert to 10 bit representation (for angle_target):
		// from [-20,20] -> [0, 1023]
		int32_t raw_count = (int32_t) ((theta_vals->angle_target + 180) * (1 / DRS_CAN_THTA_TRGT_SF));

		// pack based on the DBC file with Intel Style (Little-Endian)
		uint16_t package = 
			// REVIEW(Barach): Redundant bitmask
			((uint16_t)(raw_count & 0x3FF) & 0x3FF) |
			(((uint8_t) lastState & 0x7) << 10) |
			(((uint8_t) eeprom_driver.state & 0x3) << 13) |
			(((uint8_t) stall & 0x1) << 15);
		
		// assign canTransmitFrame to include the package for the first 16 bits.
		canTransmitFrame.data16 [0] = package;

		// Send the frame and check if it went alright, if not, signal an error w/ hard fault callback
		msg_t canTransmitMessage = canTransmitTimeout(&CAND1, CAN_ANY_MAILBOX, &canTransmitFrame, TIME_MS2I(10));

		if (canTransmitMessage != MSG_OK)
		{
			// REVIEW(Barach): CAN Message failures aren't uncommon, the board should be tolerant to this.
			hardFaultCallback();
		}

		// move servo around based if stall has or hasn't been met. if stalled, it should move away first,
		// then stay still. Otherwise move within bounds

		// Update lastState as well to reflect changing angular positions
		if (stall)
		{
			// REVIEW(Barach): Shouldn't be modifying the EEPROM memory map unless intending to write to the EEPROM (definitely
			// not needed here). Should just be a local float.
			theta_vals->angle_backoff = 
			(lastState == DRS_OPEN_TRANSITION || lastState == DRS_OPEN_STEADY) 
				? theta_vals->angle_target - angle_backoff_prime 
				: theta_vals->angle_target + angle_backoff_prime;

			// REVIEW(Barach): Same as above.
			theta_vals->angle_backoff = (theta_vals->angle_backoff < theta_vals->angle_closed) 
				? theta_vals->angle_closed 
				: theta_vals->angle_backoff;

			servoSetPosition (theta_vals->angle_backoff, theta_vals->angle_min, theta_vals->angle_max);
			
			lastState = DRS_STALL;
		} else {
			stall = checkStall(&stall);


			if (open)
			{
				// REVIEW(Barach): Same as above.
				theta_vals->angle_target += theta_vals->angle_step;
				lastState = DRS_OPEN_TRANSITION;

				if (theta_vals->angle_target > theta_vals->angle_open)
				{

					// REVIEW(Barach): Same as above.
					theta_vals->angle_target = theta_vals->angle_open;
					lastState = DRS_OPEN_STEADY;

				}
			} else {
				// REVIEW(Barach): Same as above.
				theta_vals->angle_target -= theta_vals->angle_step;
				lastState = DRS_CLOSED_TRANSITION;

				if (theta_vals->angle_target < theta_vals->angle_closed)
				{
					// REVIEW(Barach): Same as above.
					theta_vals->angle_target = theta_vals->angle_closed;
					lastState = DRS_CLOSED_STEADY;

				}
			}
			
			servoSetPosition (theta_vals->angle_target, theta_vals->angle_min, theta_vals->angle_max);		
		}

		// Sleep so it's not high enough priority compared to other more important signals
		chThdSleepMilliseconds (sleep_amount);
	}

	
}

// Callbacks ------------------------------------------------------------------------------------------------------------------

void hardFaultCallback (void)
{
	// REVIEW(Barach): There shouldn't be a while loop here, the infinite-loop should be implemented after the call to this
	//   function.
	while (true)
	{}
}