// --------------------------------------------------------------------------------
//                                   USART0.h
//                                USART routines
//                                 AVR 1 series
// --------------------------------------------------------------------------------
// This file includes functions for use of the USART0 module of AVR 1 series
//    Attiny Microcontrollers.
//  Operation modes:	Polled - all operations of the USART module are directly
//								controlled, program must wait while operations
//								take place and must poll regularly
//						Interrupt driven - Module uses an interrupt driven
//											backend, allowing the simplicity of 
//											polled software with the performance
//											gains of interrupt driven code 
//  The public functions are:
//		USART_init - initialize the USART module and local variables
//		USART_Rx_ready - if USART has received a byte
//		USART_Rx - receive USART data
//		USART_Tx_ready - if USART is ready for another byte
//		USART_Tx - send USART data
//  The following functions are included for polled operation:
//		USART_read - reads the Rx data register and returns it
//		USART_write - writes data to the Tx register
//  Setup guidelines:	1) Set constants for baud rate, buffer size, and 
//							operation mode  
//						2) Modify USART_configure_pins to use the appropriate 
//							pins for USART communication
//						3 Modify the appropriate USART_init for special
//							USART settings
//  Known Bugs:		Catastrophic behavior if either queue is full in interrupt
//						driven mode
//  Revision History:
//     5/25/18  Nathan Barton      initial revision
//     5/27/18  Nathan Barton      modified queues to use is_full flag
//     5/28/18  Nathan Barton      added polled operation mode

#include <avr/io.h>


#ifndef USART0_H_
#define USART0_H_
// --------------------------------------------------------------------------------
//       constant declarations

#define	BUFFER_SIZE	64				//queue occupies N bytes (if interrupt driven)
#define USART0_BAUD_RATE	38400  //baud rate for USART data transfer

//#define USART0_INTERRUPT_DRIVEN		//define for interrupt driven operation
#define USART0_POLLED_DRIVEN			//define for polled operation





// --------------------------------------------------------------------------------
//       function prototype declarations
void USART_init(void);
uint8_t USART_Rx_ready(void);
uint8_t USART_Rx(void);
uint8_t USART_Tx_ready(void);
void USART_Tx(uint8_t data);

#ifdef USART0_POLLED_DRIVEN
uint8_t USART_read(void);
void USART_write(uint8_t data);
#endif



#endif /* USART0_H_ */