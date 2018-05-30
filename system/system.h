// --------------------------------------------------------------------------------
//                                   system.h
//                         system initialization routines
//                                 AVR 1 series
// --------------------------------------------------------------------------------
// This file includes functions for initializing the CPU core of AVR 1 series
//    Attiny Microcontrollers.
//  The public functions are:
//		system_init - function to call to initialize all system resources
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


#ifndef SYSTEM_H_
#define SYSTEM_H_

//System clock speed - set this manually based on clk_init settings
#define F_CPU 10000000 


void system_init(void);



#endif /* SYSTEM_H_ */