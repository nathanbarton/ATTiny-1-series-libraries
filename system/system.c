// --------------------------------------------------------------------------------
//                                   system.c
//                         system initialization routines
//                                 AVR 1 series
// --------------------------------------------------------------------------------
// This file includes functions for initializing the CPU core of AVR 1 series
//    Attiny Microcontrollers.  
//  The public functions are:
//    system_init - function to call to initialize all system resources
//					add additional function calls here for system initialization
//  The following local functions are included:
//		clk_init - initialize the system clock to the specified settings
//		CPUINT_init - initialize interrupts
//		pins_reset - initialize pins to low power state
//		VREF_init - initialize internal voltage reference
//  Revision History:
//     5/24/18  Nathan Barton      initial revision
//     5/29/18  Nathan Barton      Added VREF module initialization

//local include files
#include <avr/io.h>

#include "TCB0.h"

#include "system.h" 
#include "utils/protected_io.h"
#include "USART0.h"

#include "ADC.h"



// --------------------------------------------------------------------------------
// Procedure:			clk_init
// Description:			This procedure initializes the system clock.  
//						Clock speed can be adjusted by changing the values
//						of the control registers
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

void clk_init(void)
{
	//set the control B register
	ccp_write_io((void *)&(CLKCTRL.MCLKCTRLB),
				 CLKCTRL_PDIV_2X_gc /*prescale division 2 */
				| 1 << CLKCTRL_PEN_bp); /* Prescaler enable: enabled */
				
	//set the control A register
	ccp_write_io((void *)&(CLKCTRL.MCLKCTRLA),
					CLKCTRL_CLKSEL_OSC20M_gc /* 20MHz Internal Oscillator (OSC20M) */
				| 0 << CLKCTRL_CLKOUT_bp); /* System clock out: disabled */
}



// --------------------------------------------------------------------------------
// Procedure:			CPUINT_init
// Description:			This procedure initializes system interrupts
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

void CPUINT_init(void)
{
	/* IVSEL and CVT are Configuration Change Protected */

	ccp_write_io((void *)&(CPUINT.CTRLA),
		  0 << CPUINT_CVT_bp /* Compact Vector Table: disabled */
		| 1 << CPUINT_IVSEL_bp /* Interrupt Vector Select: enabled */
		| 0 << CPUINT_LVL0RR_bp); /* Round-robin Scheduling Enable: disabled */

	// CPUINT.LVL0PRI = 0x0 << CPUINT_LVL0PRI_gp; /* Interrupt Level Priority: 0x0 */

	// CPUINT.LVL1VEC = 0x0 << CPUINT_LVL1VEC_gp; /* Interrupt Vector with High Priority: 0x0 */
}

// --------------------------------------------------------------------------------
// Procedure:			VREF_init
// Description:			This procedure initializes the internal voltage reference
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
// Last Modified:	5/29/18
void VREF_init(void)
{
	//setup VREF voltage registers
	VREF_CTRLA = VREF_ADC0REFSEL_4V34_gc    /* Voltage reference at 4.34V */
	| VREF_DAC0REFSEL_0V55_gc; /* Voltage reference at 0.55V */
	
	//setup enable registers
	VREF_CTRLB = 1 << VREF_ADC0REFEN_bp    /* ADC0 reference enable: enabled */
	| 0 << VREF_DAC0REFEN_bp; /* DAC0/AC0 reference enable: disabled */
}


// --------------------------------------------------------------------------------
// Procedure:			pins_reset
// Description:			This procedure initializes the pins by setting all of the
//						pins to the low power mode.  Pins can be re-enabled
//						as needed individually
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

void pins_reset(void)
{
	/* On AVR devices all peripherals are enable from power on reset, this
	 * disables all peripherals to save power. Driver shall enable
	 * peripheral if used */

	/* Set all pins to low power mode */

	for (uint8_t i = 0; i < 8; i++) {
		*((uint8_t *)&PORTA + 0x10 + i) |= 1 << PORT_PULLUPEN_bp;
	}

	for (uint8_t i = 0; i < 8; i++) {
		*((uint8_t *)&PORTB + 0x10 + i) |= 1 << PORT_PULLUPEN_bp;
	}

	for (uint8_t i = 0; i < 8; i++) {
		*((uint8_t *)&PORTC + 0x10 + i) |= 1 << PORT_PULLUPEN_bp;
	}
}


// --------------------------------------------------------------------------------
// Procedure:			system_init
// Description:			This procedure calls all initialization functions.  
//						Add additional calls as necessary
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

void system_init(void)
{
	clk_init();
	CPUINT_init();
	pins_reset();
	
	TCB0_init();
	USART_init();
	ADC_init();
	
}