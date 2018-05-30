// --------------------------------------------------------------------------------
//                              interrupt_state.h
//                   system interrupt state save/restore routines
//                                 AVR 1 series
// --------------------------------------------------------------------------------
// This file includes functions for saving and restoring the interrupt state
//	  of AVR 1 series Attiny Microcontrollers.  These are used for temporarily
//    disabling interrupts for critical code.
//  The public functions are:
//		save_interrupt_state - reads in the SREG and stores to local variable
//		restore_interrupt_state - takes sreg buffer and writes value to SREG 
//									(possibly re-enabling interrupts)
//  The following local functions are included:
//		save_sreg - function prototype declaration for assembly code
//		restore_sreg - function prototype declaration for assembly code
//
//
//  Revision History:
//     5/27/18  Nathan Barton      initial revision
//     5/27/18  Nathan Barton      initial revision

#ifndef INTERRUPT_STATE_H_
#define INTERRUPT_STATE_H_

extern uint8_t save_sreg(void);
extern void restore_sreg(uint8_t value);

//shared variable for holding status register
uint8_t sreg_buffer;

void save_interrupt_state(void)
{
	sreg_buffer = save_sreg();
}
void restore_interrupt_state(void)
{
	restore_sreg(sreg_buffer);
}

#endif /* INTERRUPT_STATE_H_ */