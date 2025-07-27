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

// COMMUNICATION FROM EEPROM TO CAN

#define DRS_CAN_ID_COMMAND 0x754 //(command) or 0x755 (response) // use command address for json
#define DRS_CAN_ID_RESPONSE 0x755

#define DRS_THREAD_PERIOD TIME_MS2I(250)

#define EEPROM_MAGIC_STRING "DRS-2025"
#define MC24LC32_ADDRS 0x50 // First memory address available for EEPROM

static const I2CConfig I2C1_CONFIG = 
{
	.op_mode = OPMODE_I2C,
	.clock_speed = 400000,
	.duty_cycle = FAST_DUTY_CYCLE_2
};

static const mc24lc32Config_t eeprom_config = {
	.addr = MC24LC32_ADDRS, 
	.i2c = &I2CD1,
	.timeout = TIME_MS2I(500),
	.magicString = EEPROM_MAGIC_STRING,
};

mc24lc32_t eeprom_driver;

// theta_min, theta_max, theta_step, theta_b, theta_backoff
// target_theta, openTheta, closedTheta -> order for theta_config_t

/**
 * @brief Initializes the mc24lc32 and i2c drivers to prepare for CAN
 * communication
 */
void mc24lc32_i2c_init() 
{
	if (i2cStart (&I2CD1, &I2C1_CONFIG) != MSG_OK)
		while (true) {};

	if (!mc24lc32Init (&eeprom_driver, &eeprom_config) && eeprom_driver.state == MC24LC32_STATE_FAILED)
		while (true) {}; 
}



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
			eepromHandleCanCommand (&obj, &CAND1, (eeprom_t*) &eeprom_driver); // (eeprom_t) &eeprom_driver
			//can I use the eeprom_driver or need to make own eeprom_t?
		}
	}
}

bool hysteresis(float val, float high, float low, bool state)
{
    if (state == 0 && val >= high) return 1;
    if (state == 1 && val <= low) return 0;
    return state;
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

	mc24lc32_i2c_init();


	chThdCreateStatic(&canthreadWa, sizeof(canthreadWa), NORMALPRIO, &canthread, NULL);

	float highThres = 60;
	float lowThres = 40;

	volatile bool open = 0;

	enum State {
		OPENSTEADY,
		OPENTRANSITION,
		CLOSEDSTEADY,
		CLOSEDTRANSITION,
		STALL
	};

	enum State lastState;

	// float theta_b = 15;

	volatile bool stall = false;

	const float R_SHUNT = 0.1;
	const float CURRENT_GAIN = 50;

	float I_stall = 0.6594; //Amp(s)

	volatile int stall_max_count = 25; // amount where if stall exceeds this number, stall conditions execute
	volatile int stall_cur_count = 0; // current stall counter
	
	// volatile float theta_backoff = closedTheta; // holder value

	volatile float sleep_amount = 10; // ms

	volatile float t_stall = stall_max_count * sleep_amount; // ms

	linearSensor_t output_current;

	linearSensorConfig_t output_current_config = {
		.sampleMin = 0,
		.valueMin = 0,
		.sampleMax = 4095, // 2^12 - 1 for a 12 bit ADC
		// i_max = (V_max / X_max) / (R_shunt * current_gain) * x 
		.valueMax = (3.3 / (R_SHUNT * CURRENT_GAIN)) //3.3 is the voltage max
	};

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

	// Returns stall after checking if stall conditions are met
	bool checkStall()
	{

		stmAdcSample (&adc);

		// i = output_current.value, where "i" is the current leaving the servo or the current consumed by
		// the servo
		float i = output_current.value;

		if (i >= I_stall)
		{
			++stall_cur_count;
		}
		else {
			stall_cur_count = 0;
		}

		if (stall_cur_count >= stall_max_count)
		{
			stall = true;
		}

		return stall;
	}

	linearSensorInit (&output_current, &output_current_config);
	stmAdcInit (&adc, &adc_config);

	volatile float theta_backoff_prime = t_stall * theta_config.theta_step + theta_config.theta_b;
	volatile float theta_actual = theta_config.theta_target - t_stall * theta_config.theta_step;

	while (true)
	{
		open = hysteresis (appsOnePercent, highThres, lowThres, open);


		if (stall)
		{
			theta_config.theta_backoff = 
			(lastState == OPENTRANSITION || lastState == OPENSTEADY) 
				? theta_config.theta_target - theta_backoff_prime 
				: theta_config.theta_target + theta_backoff_prime;

			theta_config.theta_backoff = (theta_config.theta_backoff < theta_config.theta_closed) 
				? theta_config.theta_closed 
				: theta_config.theta_backoff;

			servoSetPosition (theta_config.theta_backoff, theta_config.theta_min, theta_config.theta_max);
			
		} else {
			checkStall();


			if (open)
			{

				theta_config.theta_target += theta_config.theta_step;
				lastState = OPENTRANSITION;

				if (theta_config.theta_target > theta_config.theta_open)
				{

					theta_config.theta_target = theta_config.theta_open;
					lastState = OPENSTEADY;

				}
			} else {

				theta_config.theta_target -= theta_config.theta_step;
				lastState = CLOSEDTRANSITION;

				if (theta_config.theta_target < theta_config.theta_closed)
				{

					theta_config.theta_target = theta_config.theta_closed;
					lastState = CLOSEDSTEADY;

				}
			}
			
			servoSetPosition (theta_config.theta_target, theta_config.theta_min, theta_config.theta_max);		
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