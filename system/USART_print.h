// --------------------------------------------------------------------------------
//                                 USART_print.h
//                         USART string printing routines
//                                 AVR 1 series
// --------------------------------------------------------------------------------
// This file includes functions for printing strings and values over USART. 
//    It interfaces with the USART0 library for the AVR 1 series
//    Attiny Microcontrollers.
//  The public functions are:
//		usart_write_line - write a string to the USART
//		usart_send_uint - send an unsigned integer over USART
//		usart_send_int - send a signed integer over USART
//		usart_send_float - send a floating point value over USART
//  Revision History:
//     1/16/18  Nathan Barton      initial revision
//     5/27/18  Nathan Barton      updated to interface with new USART library


#ifndef USART_PRINT_H_
#define USART_PRINT_H_
#include <avr/io.h>

void usart_write_line(const char *string);
void usart_send_uint(uint32_t n);
void usart_send_int(int32_t n);
void usart_send_float(float n, uint8_t precision);



#endif /* USART_PRINT_H_ */