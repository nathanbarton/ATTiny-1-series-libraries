// --------------------------------------------------------------------------------
//                                    TCB0.h
//                         Timer/Counter Type B0 routines
//                                 AVR 1 series
// --------------------------------------------------------------------------------
// This file includes functions for the timer/counter type B0 of the AVR 1 series
//    Attiny Microcontrollers.
//  The public functions are:
//		TCB0_init - initialize the module with the settings as configured
//					in the function
//		TCB0_enable - enable the timer to start running.
//  The following local functions are included:
//
//  Revision History:
//     5/24/18  Nathan Barton      initial revision



#ifndef TCB0_H_
#define TCB0_H_

void TCB0_init(void);

void TCB0_enable(void);



#endif /* TCB0_H_ */