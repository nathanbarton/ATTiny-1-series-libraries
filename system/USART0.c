// --------------------------------------------------------------------------------
//                                   USART0.c
//                                USART routines
//                                 AVR 1 series
// --------------------------------------------------------------------------------
// This file includes functions for use of the USART0 module of AVR 1 series
//    Attiny Microcontrollers.
//  The public functions are:
//		USART_init - initialize the USART module and local variables
//		USART_Rx_ready - if USART has received a byte
//		USART_Rx - receive USART data
//		USART_Tx_ready - if USART is ready for another byte
//		USART_Tx - send USART data
//  The following functions are included for polled operation:
//		USART_read - reads the Rx data register and returns it
//		USART_write - writes data to the Tx register
//  The following local functions are included:
//
//  Known Bugs:		Catastrophic behavior if either queue is full in interrupt
//						driven mode
//  Revision History:
//     5/25/18  Nathan Barton      initial revision
//     5/27/18  Nathan Barton      modified to use DRE interrupt for Tx
//     5/27/18  Nathan Barton      modified queues to use is_full flag
//     5/28/18  Nathan Barton      added polled operation mode
//     5/28/18  Nathan Barton      added procedure for shared operations between
//									both operation modes

//local include files
//#include <avr/io.h>
#include <avr/interrupt.h>
#include "USART0.h"
#include "port.h"
#include "utils/interrupt_state.h"
#include "system.h"

///////////////////////////////////////////////////////////////////////////////////
// Functions used for both operation modes
///////////////////////////////////////////////////////////////////////////////////

// --------------------------------------------------------------------------------
// Procedure:			GET_USART0_BAUD_RATE
// Description:			This procedure gets the value to write to the Baud
//						rate register of the USART0 module
// Arguments:			BAUD_RATE - intended baud rate
// Return Values:		value to write to baud rate register
// Limitations:			None.
// Known Bugs:			None.
// Special Notes:		None.
//
// Author:			Nathan Barton
// Last Modified:	5/27/18
#define GET_USART0_BAUD_RATE(BAUD_RATE) ((float)F_CPU * 64 / (16 * (float)BAUD_RATE))

// --------------------------------------------------------------------------------
// Procedure:			USART0_configure_pins
// Description:			This procedure configures the output ports for the
//						USART0 module
// Arguments:			None.  
// Return Values:		None.  
// Limitations:			None.
// Known Bugs:			None.
// Special Notes:		modify these settings for the appropriate values
//
// Author:			Nathan Barton
// Last Modified:	5/28/18
void USART0_configure_pins(void)
{
	//set PA2 to input for USART
	PORTA_set_pin_dir(2, PORT_DIR_IN);
	PORTA_set_pin_pull_mode(2, PORT_PULL_OFF);
	PORTMUX.CTRLB |= PORTMUX_USART0_bm;
	
	//set PA1 for output to USART
	PORTA_set_pin_dir(1,PORT_DIR_OUT);
	PORTA_set_pin_level(1, false);
	PORTMUX.CTRLB |= PORTMUX_USART0_bm;
}


// --------------------------------------------------------------------------------
//define interrupt driven operation
// --------------------------------------------------------------------------------
#ifdef USART0_INTERRUPT_DRIVEN
// --------------------------------------------------------------------------------
// Structure:			queue
// Description:			This structure is for a circular queue implementation
//						to store data for the TX/RX buffers.  The queue
//						has length BUFFER_SIZE, and has two 8-bit values that
//						point to the head and tail of the queue.
// Arguments:			Data - array of length BUFFER_SIZE, to hold buffer data
//						head - head of the queue - points to first memory
//								location where data is stored, except
//								when the queue is empty
//						tail - tail of the queue - points to the first available
//								/empty memory location in the queue
//						is_full - when head = tail, then either queue is
//									full or queue is empty
// Special Notes:		when head == tail, then queue empty
//						head and tail must be initialized to 0 before using
//						is_full must be initialized to 0 before using
//
// Author:			Nathan Barton
// Last Modified:	5/27/18
struct queue
{
	uint8_t data[BUFFER_SIZE];
	uint8_t head;
	uint8_t tail;
	uint8_t is_full;
};

//Shared Variable Declaration
struct queue Tx_queue;
struct queue Rx_queue;

// --------------------------------------------------------------------------------
// Procedure:			isEmpty
// Description:			This procedure determines if a queue is empty
// Arguments:			buffer - pointer to the queue structure
// Return Values:		None.
// Shared Variables:	None.
// Local Variables:		None.
// Error Handling:		None.  
// Data Structures:		queue structure
// Limitations:			None.
// Known Bugs:			None.
// Special Notes:		None.
//
// Author:			Nathan Barton
// Last Modified:	5/27/18
uint8_t isEmpty(struct queue *buffer)
{
	//return  ((buffer->head == buffer->tail) || 
	//		((buffer->head == BUFFER_SIZE) && (buffer->tail == 0)) ||
	//		((buffer->head == 0) && (buffer->tail == BUFFER_SIZE))  );
	
	if((buffer->head == buffer->tail) && !(buffer->is_full))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}	

// --------------------------------------------------------------------------------
// Procedure:			isFull
// Description:			This procedure determines if a queue is full
// Arguments:			buffer - pointer to the queue structure
// Return Values:		None.
// Shared Variables:	None.
// Local Variables:		None.  
// Error Handling:		None.
// Data Structures:		queue structure
// Limitations:			None.
// Known Bugs:			None.
// Special Notes:		None.
//
// Author:			Nathan Barton
// Last Modified:	5/27/18
uint8_t isFull(struct queue * buffer)
{
	//return  ((buffer->head == (buffer->tail) + 1) ||
	//		((buffer->head == 0) && (buffer->tail == (BUFFER_SIZE))) );
	if((buffer->head == buffer->tail) && buffer->is_full)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}


// --------------------------------------------------------------------------------
// Procedure:			enqueue
// Description:			This procedure enqueues a datum to a circular queue
// Arguments:			buffer - pointer to the queue structure
//						data - value to enqueue to buffer
// Return Values:		None.
// Shared Variables:	None.
// Local Variables:		None.
// Error Handling:		None - queue overflow results in queue being considered
//								empty/all values dropped
// Data Structures:		queue structure
// Limitations:			None.
// Known Bugs:			None.
// Special Notes:		None.
//
// Author:			Nathan Barton
// Last Modified:	5/27/18
void enqueue(struct queue * buffer, uint8_t data)
{
	if(buffer->tail < BUFFER_SIZE)
	{
		buffer->data[buffer->tail] = data;
		buffer->tail += 1;
	}
	else
	{
		buffer->data[0] = data;
		buffer->tail = 1;
	}
	if(buffer->head == buffer->tail)
	{
		buffer->is_full = 1;
	}
}

// --------------------------------------------------------------------------------
// Procedure:			dequeue
// Description:			This procedure dequeues a datum from a circular queue
// Arguments:			buffer - pointer to the queue structure
// Return Values:		data - data dequeued from buffer
// Shared Variables:	None.
// Local Variables:		None.
// Error Handling:		If queue is empty, then 0 returned
// Data Structures:		queue structure
// Limitations:			None.
// Known Bugs:			None.
// Special Notes:		None.
//
// Author:			Nathan Barton
// Last Modified:	5/27/18
uint8_t dequeue(struct queue * buffer)
{
	if(isEmpty(buffer))
	{
		return 0;
	}
	else
	{
		buffer->is_full = 0;
		if (buffer->head >= BUFFER_SIZE)
		{
			buffer->head = 0;
		}
		return buffer->data[(buffer->head)++];
	}
}


// --------------------------------------------------------------------------------
// Procedure:			USART_init
// Description:			This procedure initializes the USART0 module of the
//						attiny
// Arguments:			None.  
// Return Values:		None.  
// Shared Variables:	Tx Buffer - head and tail initialized
//						Rx Buffer - head and tail initialized
// Local Variables:		None.
// Error Handling:		None.  
// Data Structures:		queue structure
// Limitations:			None.
// Known Bugs:			None.
// Special Notes:		None.
//
// Author:			Nathan Barton
// Last Modified:	5/28/18
void USART_init(void)
{
	//configure I/0 pins
	USART0_configure_pins();
	
	//setup USART Control Registers
	
	USART0.BAUD = (uint16_t)GET_USART0_BAUD_RATE(USART0_BAUD_RATE); /* set baud rate register */

	USART0.CTRLA = 0 << USART_ABEIE_bp /* Auto-baud Error Interrupt Enable: disabled */
			 | 0 << USART_DREIE_bp /* Data Register Empty Interrupt Enable: disabled */
			 | 0 << USART_LBME_bp /* Loop-back Mode Enable: disabled */
			 | USART_RS485_OFF_gc /* RS485 Mode disabled */
			 | 1 << USART_RXCIE_bp /* Receive Complete Interrupt Enable: enabled */
			 | 0 << USART_RXSIE_bp /* Receiver Start Frame Interrupt Enable: disabled */
			 | 0 << USART_TXCIE_bp; /* Transmit Complete Interrupt Enable: disabled */

	USART0.CTRLB = 0 << USART_MPCM_bp       /* Multi-processor Communication Mode: disabled */
	| 0 << USART_ODME_bp     /* Open Drain Mode Enable: disabled */
	| 1 << USART_RXEN_bp     /* Reciever enable: enabled */
	| USART_RXMODE_NORMAL_gc /* Normal mode */
	| 0 << USART_SFDEN_bp    /* Start Frame Detection Enable: disabled */
	| 1 << USART_TXEN_bp;    /* Transmitter Enable: enabled */

	// USART0.CTRLC = USART_CMODE_ASYNCHRONOUS_gc /* Asynchronous Mode */
	//		 | USART_CHSIZE_8BIT_gc /* Character size: 8 bit */
	//		 | USART_PMODE_DISABLED_gc /* No Parity */
	//		 | USART_SBMODE_1BIT_gc; /* 1 stop bit */

	// USART0.DBGCTRL = 0 << USART_ABMBP_bp /* Autobaud majority voter bypass: disabled */
	//		 | 0 << USART_DBGRUN_bp; /* Debug Run: disabled */

	// USART0.EVCTRL = 0 << USART_IREI_bp; /* IrDA Event Input Enable: disabled */

	// USART0.RXPLCTRL = 0x0 << USART_RXPL_gp; /* Receiver Pulse Length: 0x0 */

	// USART0.TXPLCTRL = 0x0 << USART_TXPL_gp; /* Transmit pulse length: 0x0 */
	
	//initialize TX/RX queues
	Tx_queue.head = 0;
	Tx_queue.tail = 0;
	Tx_queue.is_full = 0;
	
	Rx_queue.head = 0;
	Rx_queue.tail = 0;
	Tx_queue.is_full = 0;
}

// --------------------------------------------------------------------------------
// Procedure:			USART_Rx_ready
// Description:			This procedure returns if the USART has received
//						another byte.  It determines if data is available in the 
//						Rx buffer
// Arguments:			None.
// Return Values:		state of Rx buffer - true if not empty
// Shared Variables:	Rx Buffer - check to see if not empty
// Local Variables:		None.
// Error Handling:		None.
// Data Structures:		queue structure
// Limitations:			None.
// Known Bugs:			None.
// Special Notes:		None.
//
// Author:			Nathan Barton
// Last Modified:	5/27/18
uint8_t USART_Rx_ready(void)
{
	return !isEmpty(&Rx_queue);
}

// --------------------------------------------------------------------------------
// Procedure:			USART_Rx
// Description:			This procedure returns data from the Rx buffer.  it
//						dequeues a value from the Rx queue and returns it.  
// Arguments:			None.
// Return Values:		Rx queue data
// Shared Variables:	Rx Buffer - dequeues a value
// Local Variables:		data - temporary buffer to store dequeued data
// Error Handling:		if queue is empty, then 0 returned
// Data Structures:		queue structure
// Limitations:			None.
// Known Bugs:			None.
// Special Notes:		This section contains critical code
//							interrupts are disabled while accessing Rx queue
//							interrupt state is restored after accessing Rx queue
//
// Author:			Nathan Barton
// Last Modified:	5/27/18
uint8_t USART_Rx(void)
{
	uint8_t data;
	save_interrupt_state();
	data = dequeue(&Rx_queue);
	restore_interrupt_state();
	return data;
}

// --------------------------------------------------------------------------------
// Procedure:			USART_Tx_ready
// Description:			This procedure returns if the USART is ready to transmit 
//						another byte.  It determines if the Tx buffer is full
// Arguments:			None.
// Return Values:		state of Tx buffer - true if not full
// Shared Variables:	Tx Buffer - check to see if not full
// Local Variables:		None.
// Error Handling:		None.
// Data Structures:		queue structure
// Limitations:			None.
// Known Bugs:			None.
// Special Notes:		None.
//
// Author:			Nathan Barton
// Last Modified:	5/27/18
uint8_t USART_Tx_ready(void)
{
	return !isFull(&Tx_queue);
}

// --------------------------------------------------------------------------------
// Procedure:			USART_Tx
// Description:			This procedure adds data to the Tx buffer.  It
//						enqueues a value to the Tx queue
// Arguments:			Data - data to enqueue to Tx buffer
// Return Values:		None.  
// Shared Variables:	Tx Buffer - enqueues a value
// Local Variables:		None.  
// Error Handling:		does nothing if Tx_queue is full
// Data Structures:		queue structure
// Limitations:			None.  
// Known Bugs:			Catastrophic behavior if Tx_queue is full
// Special Notes:		This section contains critical code
//							interrupts are disabled while accessing Tx queue
//							interrupt state is restored after accessing Tx queue
//
// Author:			Nathan Barton
// Last Modified:	5/27/18
void USART_Tx(uint8_t data)
{	
	while((isFull(&Tx_queue)));
	
	
		save_interrupt_state();
		enqueue(&Tx_queue, data);
		//if no transmission ongoing, then start a new one by enabling the interrupt
		USART0.CTRLA |= 1 << USART_DREIE_bp; /* Data Register Empty Interrupt Enable: enabled */
		restore_interrupt_state();
	
}

// --------------------------------------------------------------------------------
// Procedure:			USART0_RXC_vect interrupt handler
// Description:			This procedure is the interrupt handler for the receive
//						complete interrupt.  it reads the new received 
//						value and enqueues it into the Rx buffer
// Arguments:			None.  
// Return Values:		None.
// Shared Variables:	Rx Buffer - enqueues a value
// Local Variables:		None.  
// Error Handling:		if queue is full, then no action taken
// Data Structures:		queue structure
// Limitations:			None.
// Known Bugs:			None.
// Special Notes:		None.  
//
// Author:			Nathan Barton
// Last Modified:	5/27/18
ISR(USART0_RXC_vect)
{
	//the interrupt flag for the USART0 module is cleared when the RXDATA register 
		//is read
	enqueue(&Rx_queue, USART0.RXDATAL);
}

// --------------------------------------------------------------------------------
// Procedure:			USART0_DRE_vect interrupt handler
// Description:			This procedure is the interrupt handler for the data
//						register empty interrupt.  It checks to see if any additional
//						data is available in the Tx buffer, and if so, dequeues
//						and transmits it.  otherwise it disables the data
//						register empty interrupt.  
// Arguments:			None.
// Return Values:		None.
// Shared Variables:	Tx Buffer - Dequeues a value if not empty
// Local Variables:		None.
// Error Handling:		if queue is empty, then no action taken
// Data Structures:		queue structure
// Limitations:			None.
// Known Bugs:			None.
// Special Notes:		None.
//
// Author:			Nathan Barton
// Last Modified:	5/27/18
ISR(USART0_DRE_vect)
{
	//the interrupt flag for the USART0 module is cleared when data added to Tx data register
	if(!isEmpty(&Tx_queue))
	{
		USART0.TXDATAL = dequeue(&Tx_queue);
	}
	else //no more data, so disable the interrupt
	{
		USART0.CTRLA &= ~(1 << USART_DREIE_bp); /* Data Register Empty Interrupt Enable: disabled */
	}
}

#endif

///////////////////////////////////////////////////////////////////////////////////
//define polled operation
///////////////////////////////////////////////////////////////////////////////////
#ifdef USART0_POLLED_DRIVEN

// --------------------------------------------------------------------------------
// Procedure:			USART_init
// Description:			This procedure initializes the USART0 module of the
//						attiny for polled operation
// Arguments:			None.
// Return Values:		None.
// Limitations:			None.
// Known Bugs:			None.
// Special Notes:		None.
//
// Author:			Nathan Barton
// Last Modified:	5/28/18
void USART_init(void)
{
	//configure I/0 pins
	USART0_configure_pins();
	
	//setup USART Control Registers
	
	USART0.BAUD = (uint16_t)GET_USART0_BAUD_RATE(USART0_BAUD_RATE); /* set baud rate register */

	//USART0.CTRLA = 0 << USART_ABEIE_bp /* Auto-baud Error Interrupt Enable: disabled */
	//| 0 << USART_DREIE_bp /* Data Register Empty Interrupt Enable: disabled */
	//| 0 << USART_LBME_bp /* Loop-back Mode Enable: disabled */
	//| USART_RS485_OFF_gc /* RS485 Mode disabled */
	//| 1 << USART_RXCIE_bp /* Receive Complete Interrupt Enable: enabled */
	//| 0 << USART_RXSIE_bp /* Receiver Start Frame Interrupt Enable: disabled */
	//| 0 << USART_TXCIE_bp; /* Transmit Complete Interrupt Enable: disabled */

	USART0.CTRLB = 0 << USART_MPCM_bp       /* Multi-processor Communication Mode: disabled */
	| 0 << USART_ODME_bp     /* Open Drain Mode Enable: disabled */
	| 1 << USART_RXEN_bp     /* Reciever enable: enabled */
	| USART_RXMODE_NORMAL_gc /* Normal mode */
	| 0 << USART_SFDEN_bp    /* Start Frame Detection Enable: disabled */
	| 1 << USART_TXEN_bp;    /* Transmitter Enable: enabled */

	// USART0.CTRLC = USART_CMODE_ASYNCHRONOUS_gc /* Asynchronous Mode */
	//		 | USART_CHSIZE_8BIT_gc /* Character size: 8 bit */
	//		 | USART_PMODE_DISABLED_gc /* No Parity */
	//		 | USART_SBMODE_1BIT_gc; /* 1 stop bit */

	// USART0.DBGCTRL = 0 << USART_ABMBP_bp /* Autobaud majority voter bypass: disabled */
	//		 | 0 << USART_DBGRUN_bp; /* Debug Run: disabled */

	// USART0.EVCTRL = 0 << USART_IREI_bp; /* IrDA Event Input Enable: disabled */

	// USART0.RXPLCTRL = 0x0 << USART_RXPL_gp; /* Receiver Pulse Length: 0x0 */

	// USART0.TXPLCTRL = 0x0 << USART_TXPL_gp; /* Transmit pulse length: 0x0 */
	
}

// --------------------------------------------------------------------------------
// Procedure:			USART_Rx_ready
// Description:			This procedure returns if the USART has received
//						another byte
// Arguments:			None.
// Return Values:		state of Rx - true if data available
// Error Handling:		None.
// Limitations:			None.
// Known Bugs:			None.
// Special Notes:		None.
//
// Author:			Nathan Barton
// Last Modified:	5/28/18
uint8_t USART_Rx_ready(void)
{
	return (USART0.STATUS & USART_RXCIF_bm);
}

// --------------------------------------------------------------------------------
// Procedure:			USART_Rx
// Description:			This procedure returns data from the Rx.  it waits for 
//						data to be received and then reads the Rx data register 
//						and return it
// Arguments:			None.
// Return Values:		Rx data
// Shared Variables:	None.  
// Local Variables:		None
// Error Handling:		If data not available, then hangs
// Data Structures:		None.  
// Limitations:			None.
// Known Bugs:			None.
// Special Notes:		None.  
//
// Author:			Nathan Barton
// Last Modified:	5/28/18
uint8_t USART_Rx(void)
{
	while(!USART_Rx_ready());
	
	return USART0.RXDATAL;
}

// --------------------------------------------------------------------------------
// Procedure:			USART_read
// Description:			This procedure returns data from the Rx.  it reads the Rx 
//						data register and returns it
// Arguments:			None.
// Return Values:		Rx data
// Shared Variables:	None.
// Local Variables:		None
// Error Handling:		If data not available, then returns 0
// Data Structures:		None.
// Limitations:			None.
// Known Bugs:			None.
// Special Notes:		None.
//
// Author:			Nathan Barton
// Last Modified:	5/28/18
uint8_t USART_read(void)
{
	if(USART_Rx_ready())
	{
		return USART0.RXDATAL;
	}
	else
	{
		return 0;
	}	
}

// --------------------------------------------------------------------------------
// Procedure:			USART_Tx_ready
// Description:			This procedure returns if the USART is ready to transmit
//						another byte.  It determines if the Tx is currently
//						transmitting
// Arguments:			None.
// Return Values:		state of Tx - true if ready
// Shared Variables:	None.  
// Local Variables:		None.
// Error Handling:		None.
// Limitations:			None.
// Known Bugs:			None.
// Special Notes:		None.
//
// Author:			Nathan Barton
// Last Modified:	5/28/18
uint8_t USART_Tx_ready(void)
{
	return (USART0.STATUS & USART_DREIF_bm);
}

// --------------------------------------------------------------------------------
// Procedure:			USART_Tx
// Description:			This procedure writes data to the Tx buffer.  It waits 
//						for the present operation to be complete then writes
//						another byte to the Tx register
// Arguments:			Data - data to send to Tx
// Return Values:		None.
// Shared Variables:	None.  
// Local Variables:		None.
// Error Handling:		waits while Tx busy
// Limitations:			None.
// Known Bugs:			None.  
// Special Notes:		None.  
//
// Author:			Nathan Barton
// Last Modified:	5/28/18
void USART_Tx(uint8_t data)
{
	while (!USART_Tx_ready());
	
	USART0.TXDATAL = data;
}

// --------------------------------------------------------------------------------
// Procedure:			USART_write
// Description:			This procedure writes data to the Tx register
// Arguments:			Data - data to send to Tx
// Return Values:		None.
// Shared Variables:	None.
// Local Variables:		None.
// Error Handling:		Does nothing if Tx is busy
// Limitations:			None.
// Known Bugs:			None.
// Special Notes:		None.
//
// Author:			Nathan Barton
// Last Modified:	5/28/18
void USART_write(uint8_t data)
{
	if(USART_Tx_ready())
	{
		USART0.TXDATAL = data;
	}
}

#endif