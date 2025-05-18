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
	}
}

bool hysteresis(float val, float high, float low, bool state)
{
    if (state == 0 && val >= high) return 1;
    if (state == 1 && val <= low) return 0;
    return state;
}

//stub function
bool checkstall()
{
	// I = Va / (R * A)
	// I_stalled = something
	// V_r = V_a / A

	volatile bool stubStalled = false;
	return stubStalled;
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

	/*
		Move this into servo header, make a new function servoInit();
	*/
	pwmStart (&PWMD3, &pwm3Config);


	chThdCreateStatic(&canthreadWa, sizeof(canthreadWa), NORMALPRIO, &canthread, NULL);

	//My work :) 

	float openTheta = 20;
	float closedTheta = -20;
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

	float target_theta = closedTheta;
	// float target_theta = openTheta;
	// float theta_step = 0.1;
	float theta_step = 0.25;
	float theta_b = 15;

	volatile bool stall = false;

	float theta_max = 45;
	float theta_min = -45;

	const float R_SHUNT = 0.1;
	const float CURRENT_GAIN = 50;

	// float I_stall = 1; //Amp
	// float I_stall = 0.5;
	float I_stall = 0.6594;
	// float I_stall = 0.69;

	volatile int stall_max_count = 25; // amount where if stall exceeds this number, stall conditions execute
	volatile int stall_cur_count = 0; // current stall counter
	
	volatile float theta_backoff = closedTheta; // holder value

	volatile float sleep_amount = 10; // ms

	volatile float t_stall = stall_max_count * sleep_amount; // ms

	volatile float theta_backoff_prime = t_stall * theta_step + theta_b;
	volatile float theta_actual = target_theta - t_stall * theta_step;

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

		// i >= I_stall -> stalled
		// i < I_stall -> ~stalled
		// if (i >= I_stall) { 
		// 	stall = true; 
		// }
		// else { 
		// 	stall = false; 
		// }

		// stall_cur_count = stall_cur_count;

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

	// Do nothing.
	while (true)
	{
		// normal functioning, should still be here after everything is done
		open = hysteresis (appsOnePercent, highThres, lowThres, open);
		// open = 0;


		if (stall)
		{
			//old code:
			// theta_backoff = (lastState == OPENTRANSITION || lastState == OPENSTEADY) ? target_theta - theta_b : target_theta + theta_b;
			
			// //clamping so no out of bounds
			// theta_backoff = (target_theta < closedTheta) ? closedTheta : theta_backoff;
			// theta_backoff = (target_theta > openTheta) ? openTheta : theta_backoff;

			// // +, -
			// theta_backoff_prime = (lastState == OPENTRANSITION || lastState == OPENSTEADY) ? t_stall * theta_step + theta_backoff : -(t_stall * theta_step + theta_backoff);

			// //clamping so no out of bounds
			// theta_backoff_prime = (theta_backoff_prime < closedTheta) ? closedTheta : theta_backoff_prime;
			// theta_backoff_prime = (theta_backoff_prime > openTheta) ? openTheta : theta_backoff_prime;

			// servoSetPosition (theta_backoff, theta_min, theta_max);

			theta_backoff = (lastState == OPENTRANSITION || lastState == OPENSTEADY) ?  target_theta - theta_backoff_prime : target_theta + theta_backoff_prime;
 
			//clamping stuff
			theta_backoff = (theta_backoff < closedTheta) ? closedTheta : theta_backoff;
			theta_backoff = (theta_backoff > openTheta) ? openTheta : theta_backoff;

			servoSetPosition (theta_backoff, theta_min, theta_max);

		} else {
			checkStall();


			if (open)
			{
				target_theta += theta_step;
				lastState = OPENTRANSITION;
				if (target_theta > openTheta)
				{
					target_theta = openTheta;
					lastState = OPENSTEADY;
				}
			} else {
				target_theta -= theta_step;
				lastState = CLOSEDTRANSITION;
				if (target_theta < closedTheta)
				{
					target_theta = closedTheta;
					lastState = CLOSEDSTEADY;
				}
			}

			/*
				TODO: Test servo for incorrect stalling, also check about whether or not my code is
				necessarily working for checking current and for the sort

				Logs and such:
				5/3:
				Double check that I_stall value is correct and works
				- changes: changed I_stall to work whenever current is higher than normal
					previous values were not as successful in indicating that or just movement
			*/
			
			servoSetPosition (target_theta, theta_min, theta_max);			

		}



		chThdSleepMilliseconds (sleep_amount);
		
		// if (open)
		// {
		// 	target_theta += theta_step;
		// 	lastState = OPENTRANSITION;
		// 	if (target_theta > openTheta)
		// 	{
		// 		target_theta = openTheta;
		// 		lastState = OPENSTEADY;
		// 	}
		// } else {
		// 	target_theta -= theta_step;
		// 	lastState = CLOSEDTRANSITION;
		// 	if (target_theta < closedTheta)
		// 	{
		// 		target_theta = closedTheta;
		// 		lastState = CLOSEDSTEADY;
		// 	}
		// }

		//check stalling

		// if (checkStall() && !stall)
		// if (!stall && checkStall())
		// {
		// 	target_theta = (lastState == OPENTRANSITION || lastState == OPENSTEADY) ? target_theta - theta_b : target_theta + theta_b;
		// 	target_theta = (target_theta < closedTheta) ? closedTheta : target_theta;
		// 	target_theta = (target_theta > openTheta) ? openTheta : target_theta;
		// 	stall = true;
		// } else {
		// 	servoSetPosition (target_theta, theta_min, theta_max);
		// }

		
		// chThdSleepMilliseconds (10);
	}

	
}

// Callbacks ------------------------------------------------------------------------------------------------------------------

void hardFaultCallback (void)
{
	// TODO(Barach): Fault behavior
}