// --------------------------------------------------------------------------------
//                                    ADC.c
//                                 ADC routines
//                                 AVR 1 series
// --------------------------------------------------------------------------------
// This file includes functions for use of the ADC module of AVR 1 series
//    Attiny Microcontrollers.  
//  The public functions are:
//		ADC_init - initialize the ADC module - modify settings here
//		ADC_start_sample - start a conversion on the given channel
//		ADC_sample_done - check to see if conversion done/ready to read
//		ADC_get_sample_result - read data from the sampling run
//		ADC_take_sample - start a conversion, wait for it to complete, and
//							return the result
//  The following local functions are included:
//		ADC_pins_init - enable ADC mode of input pins - modify settings here
//  Known Bugs:		None
//
//  Revision History:
//     5/29/18  Nathan Barton      initial revision

// Local Includes
#include <avr/io.h>
#include "ADC.h"
#include "port.h"


// --------------------------------------------------------------------------------
// Procedure:			ADC_pins_init
// Description:			This procedure initializes the ADC input pins
// Arguments:			None.
// Return Values:		None.
// Shared Variables:	None.
// Local Variables:		None.
// Error Handling:		None.
// Data Structures:		None
// Limitations:			None.
// Known Bugs:			None.
// Special Notes:		Adjust for application needs
//
// Author:			Nathan Barton
// Last Modified:	5/29/18
void ADC_pins_init(void)
{
	//set PA4 to input to ADC
	PORTA_set_pin_dir(4, PORT_DIR_IN);
	PORTA_set_pin_pull_mode(4, PORT_PULL_OFF);
	
	//set PA5 for input to ADC
	PORTA_set_pin_dir(5, PORT_DIR_IN);
	PORTA_set_pin_pull_mode(5, PORT_PULL_OFF);
}

// --------------------------------------------------------------------------------
// Procedure:			ADC_init
// Description:			This procedure initializes the ADC module of the
//						attiny
// Arguments:			None.
// Return Values:		None.
// Shared Variables:	None.  
// Local Variables:		None.
// Error Handling:		None.
// Data Structures:		None
// Limitations:			None.
// Known Bugs:			None.
// Special Notes:		None.
//
// Author:			Nathan Barton
// Last Modified:	5/29/18
void ADC_init(void)
{
	// ADC0.CALIB = ADC_DUTYCYC_DUTY50_gc; /* 50% Duty cycle */

	// ADC0.CTRLB = ADC_SAMPNUM_ACC1_gc; /* 1 ADC sample */

	ADC0.CTRLC = ADC_PRESC_DIV8_gc /* CLK_PER divided by 8 */
			 | ADC_REFSEL_INTREF_gc /* Internal reference */
			 | 0 << ADC_SAMPCAP_bp; /* Sample Capacitance Selection: disabled */

	// ADC0.CTRLD = 0 << ADC_ASDV_bp /* Automatic Sampling Delay Variation: disabled */
	//		 | 0x0 << ADC_SAMPDLY_gp /* Sampling Delay Selection: 0x0 */
	//		 | ADC_INITDLY_DLY0_gc; /* Delay 0 CLK_ADC cycles */

	// ADC0.CTRLE = ADC_WINCM_NONE_gc; /* No Window Comparison */

	// ADC0.DBGCTRL = 0 << ADC_DBGRUN_bp; /* Debug run: disabled */

	// ADC0.EVCTRL = 0 << ADC_STARTEI_bp; /* Start Event Input Enable: disabled */

	// ADC0.INTCTRL = 0 << ADC_RESRDY_bp /* Result Ready Interrupt Enable: disabled */
	//		 | 0 << ADC_WCMP_bp; /* Window Comparator Interrupt Enable: disabled */

	// ADC0.MUXPOS = ADC_MUXPOS_AIN0_gc; /* ADC input pin 0 */

	// ADC0.SAMPCTRL = 0x0 << ADC_SAMPLEN_gp; /* Sample length: 0x0 */

	// ADC0.WINHT = 0x0; /* Window Comparator High Threshold: 0x0 */

	// ADC0.WINLT = 0x0; /* Window Comparator Low Threshold: 0x0 */

	ADC0.CTRLA = 1 << ADC_ENABLE_bp     /* ADC Enable: enabled */
	| 0 << ADC_FREERUN_bp  /* ADC Freerun mode: disabled */
	| ADC_RESSEL_8BIT_gc   /* 8-bit mode */
	| 0 << ADC_RUNSTBY_bp; /* Run standby mode: disabled */
}

// --------------------------------------------------------------------------------
// Procedure:			ADC_start_sample
// Description:			This procedure initiates a conversion on the ADC
// Arguments:			Channel - channel number of the ADC to select
// Return Values:		None.
// Error Handling:		None.
// Data Structures:		None
// Limitations:			None.
// Known Bugs:			None.
// Special Notes:		None.
//
// Author:			Nathan Barton
// Last Modified:	5/30/18
void ADC_start_sample(uint8_t channel)
{
	//setup channel mux
	ADC0.MUXPOS = channel;
	//and start the conversion
	ADC0.COMMAND =ADC_STCONV_bm;
}

// --------------------------------------------------------------------------------
// Procedure:			ADC_sample_done
// Description:			This procedure checks to see if a conversion is complete
// Arguments:			none.
// Return Values:		Status of the conversion - true if a conversion is complete
// Error Handling:		None.
// Data Structures:		None
// Limitations:			None.
// Known Bugs:			None.
// Special Notes:		None.
//
// Author:			Nathan Barton
// Last Modified:	5/30/18
uint8_t ADC_sample_done(void)
{
	return (ADC0.INTFLAGS & ADC_RESRDY_bm);
}

// --------------------------------------------------------------------------------
// Procedure:			ADC_get_sample_result
// Description:			This procedure returns the data from the ADC Result
//						register from a conversion
// Arguments:			none.
// Return Values:		Data from ADC unit
// Error Handling:		None - invalid data returned if conversion not ready
// Data Structures:		None
// Limitations:			None.
// Known Bugs:			None.
// Special Notes:		None.
//
// Author:			Nathan Barton
// Last Modified:	5/30/18
uint8_t ADC_get_sample_result(void)
{
	return ADC0.RESL;
}

// --------------------------------------------------------------------------------
// Procedure:			ADC_take_sample
// Description:			This procedure initiates a conversion on the given ADC
//						channel, waits for a conversion to take place, and then
//						returns the result
// Arguments:			Channel - channel number of the ADC to select
// Return Values:		Data from ADC unit
// Error Handling:		None.
// Data Structures:		None
// Limitations:			None.
// Known Bugs:			None.
// Special Notes:		None.
//
// Author:			Nathan Barton
// Last Modified:	5/30/18
uint8_t ADC_take_sample(uint8_t channel)
{
	ADC_start_sample(channel);
	while(!ADC_sample_done());
	
	return ADC_get_sample_result();
}
