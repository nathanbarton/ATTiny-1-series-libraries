// --------------------------------------------------------------------------------
//                                protected_io.h
//                   Config Change Protected I/0 access routines
//                                 AVR 1 series
// --------------------------------------------------------------------------------
// This file includes functions for accessing Configuration Change Protected
//    I/O registers of the CPU core of AVR 1 series Attiny Microcontrollers.
//  The following functions are included:
//		protected_write_io - assembly code function that accesses the 
//							CCP-protected register and writes to it.  
//  Revision History:
//     5/25/18  Nathan Barton      initial revision

//local include files
 #include "gas.h"  //contains useful macros for interfacing assembly code with C code
 #include <avr/io.h>

 PUBLIC_FUNCTION(protected_write_io)
	push	r31						//push Z register onto stack
	push	r30						

	ldi		r31, 0					// clear the Z register
	ldi		r30, 0

	movw    r30, r24                // Load addr into Z
	out     CCP, r22                // Start CCP handshake
	st      Z, r20                  // Write value to I/O register

	pop		r30						//pop Z register back off stack
	pop		r31
	ret                             // Return to caller
END_FUNC(protected_write_io)
END_FILE()
