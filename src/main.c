// Includes -------------------------------------------------------------------------------------------------------------------

// ChibiOS
#include "ch.h"
#include "hal.h"

// Threads --------------------------------------------------------------------------------------------------------------------

static THD_WORKING_AREA (heartbeatWa, 128);

static THD_FUNCTION (heartbeatThread, arg)
{
	(void) arg;
	chRegSetThreadName ("heartbeat");

	while (true)
	{
		palSetLine (LINE_LED_HEARTBEAT);
		chThdSleepMilliseconds (500);
		palClearLine (LINE_LED_HEARTBEAT);
		chThdSleepMilliseconds (500);
	}
}

// Entrypoint -----------------------------------------------------------------------------------------------------------------

int main (void)
{
	// ChibiOS Initialization
	halInit ();
	chSysInit ();

	// Create the heartbeat thread
	chThdCreateStatic (heartbeatWa, sizeof(heartbeatWa), NORMALPRIO, heartbeatThread, NULL);

	// Do nothing.
	while (true)
		chThdSleepMilliseconds (500);
}
