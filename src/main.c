// Includes -------------------------------------------------------------------------------------------------------------------

// Includes
#include "servo.h"
#include "debug.h"

// ChibiOS
#include "ch.h"
#include "hal.h"

#include "controls/lerp.h"


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

	// float theta = 0;

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
	float theta_step = 0.1;
	float theta_b = 15;

	volatile bool stall = false;

	float theta_max = 45;
	float theta_min = -45;

	// Do nothing.
	while (true)
	{
		open = hysteresis (appsOnePercent, highThres, lowThres, open);

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

		//check stalling

		if (checkstall() && !stall)
		{
			target_theta = (lastState == OPENTRANSITION || lastState == OPENSTEADY) ? target_theta - theta_b : target_theta + theta_b;
			target_theta = (target_theta < closedTheta) ? closedTheta : target_theta;
			target_theta = (target_theta > openTheta) ? openTheta : target_theta;
			stall = true;
		}

		servoSetPosition (target_theta, theta_min, theta_max);
		chThdSleepMilliseconds (10);
	}

	
}

// Callbacks ------------------------------------------------------------------------------------------------------------------

void hardFaultCallback (void)
{
	// TODO(Barach): Fault behavior
}