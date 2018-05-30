// --------------------------------------------------------------------------------
//                                protected_io.h
//                   Config Change Protected I/0 access routines
//                                 AVR 1 series
// --------------------------------------------------------------------------------
// This file includes functions for accessing Configuration Change Protected
//    I/O registers of the CPU core of AVR 1 series Attiny Microcontrollers.
//  The public functions are:
//		ccp_write_io - writes a value to the CCP-protected register
//						at the given address
//  The following local functions are included:
//		protected_write_io - function prototype declaration for the 
//								assembly code that accesses the CCP-protected
//								register.  
//  Revision History:
//     5/25/18  Nathan Barton      initial revision


#ifndef PROTECTED_IO_H_
#define PROTECTED_IO_H_

extern void protected_write_io(void *addr, uint8_t code, uint8_t value);

static inline void ccp_write_io(void *addr, uint8_t value)
{
	protected_write_io(addr, CCP_IOREG_gc, value);
}


#endif /* PROTECTED_IO_H_ */