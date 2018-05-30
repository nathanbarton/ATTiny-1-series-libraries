// --------------------------------------------------------------------------------
//                              interrupt_state.S
//                   system interrupt state save/restore routines
//                                 AVR 1 series
// --------------------------------------------------------------------------------
// This file includes functions for saving and restoring the interrupt state
//	  of AVR 1 series Attiny Microcontrollers.  These are used for temporarily
//    disabling interrupts for critical code.
//  The following functions are included:
//		save_sreg - function prototype declaration for assembly code
//		restore_sreg - function prototype declaration for assembly code
//  Revision History:
//     5/27/18  Nathan Barton      initial revision

//local include files
 #include "gas.h"  //contains useful macros for interfacing assembly code with C code
 #include <avr/io.h>

PUBLIC_FUNCTION(save_sreg)
	IN		R0, SREG				//read in the cpu status register
	CLI								//and disable interrupts
	MOV		R24, R0					//move to return value location
	RET                             // Return to caller
END_FUNC(save_sreg)

PUBLIC_FUNCTION(restore_sreg)
// value passed into R24
	OUT		SREG, R24				//restore SREG to previous state
	ret                             // Return to caller
END_FUNC(restore_sreg)

END_FILE()
