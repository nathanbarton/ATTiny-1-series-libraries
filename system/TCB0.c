// --------------------------------------------------------------------------------
//                                    TCB0.c
//                         Timer/Counter Type B0 routines
//                                 AVR 1 series
// --------------------------------------------------------------------------------
// This file includes functions for the timer/counter type B0 of the AVR 1 series
//    Attiny Microcontrollers.
//  The public functions are:
//		TCB0_init - initialize the module with the settings as configured
//					in the function
//		TCB0_enable - enable the timer to start running.  
//
//  The following local functions are included:
//
//  Revision History:
//     5/24/18  Nathan Barton      initial revision

#include "TCB0.h"
#include <avr/io.h>

// --------------------------------------------------------------------------------
// Procedure:			TCB0_init
// Description:			This procedure initializes the Timer/Counter type B0
//						adjust settings as necessary
// Arguments:			None.
// Return Values:		None.
// Shared Variables:	None.
// Local Variables:		None.
// Error Handling:		None.
// Data Structures:		None.
// Limitations:			None.
// Known Bugs:			None.
// Special Notes:		None.
//
// Author:			Nathan Barton
// Last Modified:	5/24/18

void TCB0_init(void)
{
		TCB0.CCMP = 0x1388;  /* Compare or Capture */

		// TCB0.CNT = 0x0; /* Count: 0x0 */

		// TCB0.CTRLB = 0 << TCB_ASYNC_bp /* Asynchronous Enable: disabled */
		//		 | 0 << TCB_CCMPEN_bp /* Pin Output Enable: disabled */
		//		 | 0 << TCB_CCMPINIT_bp /* Pin Initial State: disabled */
		//		 | TCB_CNTMODE_INT_gc; /* Periodic Interrupt */

		// TCB0.DBGCTRL = 0 << TCB_DBGRUN_bp; /* Debug Run: disabled */

		// TCB0.EVCTRL = 0 << TCB_CAPTEI_bp /* Event Input Enable: disabled */
		//		 | 0 << TCB_EDGE_bp /* Event Edge: disabled */
		//		 | 0 << TCB_FILTER_bp; /* Input Capture Noise Cancellation Filter: disabled */

		TCB0.INTCTRL = 1 << TCB_CAPT_bp; /* Capture or Timeout: Enabled */

		TCB0.CTRLA = TCB_CLKSEL_CLKDIV2_gc /* CLK_PER/2 */
		| 0 << TCB_ENABLE_bp /* Enable: disabled */
		| 0 << TCB_RUNSTDBY_bp /* Run Standby: disabled */
		| 0 << TCB_SYNCUPD_bp; /* Synchronize Update: disabled */
}


// --------------------------------------------------------------------------------
// Procedure:			TCB0_enable
// Description:			This procedure sets the TCB0 unit to run
// Arguments:			None.
// Return Values:		None.
// Shared Variables:	None.
// Local Variables:		None.
// Error Handling:		None.
// Data Structures:		None.
// Limitations:			None.
// Known Bugs:			None.
// Special Notes:		None.
//
// Author:			Nathan Barton
// Last Modified:	5/24/18

void TCB0_enable(void)
{
	TCB0.CTRLA = TCB0.CTRLA  | (1 << TCB_ENABLE_bp);
}