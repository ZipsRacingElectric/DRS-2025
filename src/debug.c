// Header
#include "debug.h"

// Threads --------------------------------------------------------------------------------------------------------------------

#if DEBUG_HEARTBEAT_ENABLED

	static THD_WORKING_AREA (heartbeatWa, 128);

	static THD_FUNCTION (heartbeatThread, arg)
	{
		(void) arg;
		chRegSetThreadName ("heartbeat");

		while (true)
		{
			palSetLine (DEBUG_HB);
			chThdSleepMilliseconds (500);
			palClearLine (DEBUG_HB);
			chThdSleepMilliseconds (500);
		}
	}

#endif // DEBUG_HEARTBEAT_ENABLED

// Functions ------------------------------------------------------------------------------------------------------------------

void debugInit (const char* boardName)
{
	// Create the heartbeat thread
	#if DEBUG_HEARTBEAT_ENABLED
		chThdCreateStatic (heartbeatWa, sizeof(heartbeatWa), NORMALPRIO, heartbeatThread, NULL);
	#endif // DEBUG_HEARTBEAT_ENABLED

	// Start the serial driver
	#if DEBUG_SERIAL_ENABLED
		sdStart (&DEBUG_SERIAL, NULL);
		DEBUG_PRINTF ("Launching %s...\n", boardName);
		DEBUG_PRINTF ("\tSoftware Version: " __DATE__ "\n");
	#endif // DEBUG_SERIAL_ENABLED
}