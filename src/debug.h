#ifndef DEBUG_H
#define DEBUG_H

/// @brief Enables / disables the heartbeat LED.
#define DEBUG_HEARTBEAT_ENABLED 1

/// @brief Enables / disables printing to the debug serial port.
#define DEBUG_SERIAL_ENABLED 1

// Includes -------------------------------------------------------------------------------------------------------------------

// ChibiOS
#include "hal.h"

#if DEBUG_HEARTBEAT_ENABLED
	#include "hal_pal.h"
#endif

#if DEBUG_SERIAL_ENABLED
	#include "chprintf.h"
#endif

// Macros ---------------------------------------------------------------------------------------------------------------------

/// @brief The serial port used for debug information.
#define DEBUG_SERIAL SD1

/// @brief The I/O line controlling the heartbeat LED.
#define DEBUG_HB LINE_LED_HEARTBEAT

#ifndef DEBUG_PRINTF
	#if DEBUG_SERIAL_ENABLED
		/// @brief Prints formatted output to the debug serial port.
		#define DEBUG_PRINTF(format, ...) \
			chprintf((BaseSequentialStream*) &DEBUG_SERIAL, format, ##__VA_ARGS__)
	#else
		/// @brief Prints formatted output to the debug serial port.
		#define DEBUG_PRINTF(format, ...)
	#endif
#endif // DEBUG_PRINTF

// Functions ------------------------------------------------------------------------------------------------------------------

/**
 * @brief Initializes the board's debugging related features.
 */
void debugInit (const char* boardName);

#endif // DEBUG_H