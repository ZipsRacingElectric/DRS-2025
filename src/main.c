// Includes -------------------------------------------------------------------------------------------------------------------

// Includes
#include "debug.h"

// ChibiOS
#include "ch.h"
#include "hal.h"

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

// Entrypoint -----------------------------------------------------------------------------------------------------------------

int main (void)
{
	// ChibiOS Initialization
	halInit ();
	chSysInit ();

	// Debug Initialization
	debugInit ("Drag Reduction Board, Revision AA");

	// CAN Initialization
	canStart (&CAND1, &can1Config);
	palClearLine (LINE_CAN1_STBY);

	// Do nothing.
	while (true)
		chThdSleepMilliseconds (500);
}