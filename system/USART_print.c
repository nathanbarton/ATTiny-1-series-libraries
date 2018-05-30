// --------------------------------------------------------------------------------
//                                 USART_print.c
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
//  The following local functions are included:
//		usart_write_line - helper function for writing a string to USART
//  Revision History:
//     1/16/18  Nathan Barton      initial revision
//     5/27/18  Nathan Barton      updated to interface with new USART library

//local includes
#include "USART_print.h"
#include "USART0.h"

void usart_send_uint(uint32_t n)
{
	char tmp[11];
	int i=sizeof(tmp)-1;
	tmp[i] = '\0';
	
	do
	{
		tmp[--i]='0'+n%10;
		n/=10;
	} while (n);
	
	usart_write_line(tmp+i);
}

void usart_send_int(int32_t n)
{
	if(n<0){
		usart_write_line("-");
		n*= (-1);
	}
	usart_send_uint(n);
}

void usart_send_float(float n, uint8_t precision)
{
	uint8_t i=0;
	uint32_t value=0;
	if(n<0){
		usart_write_line("-");
		n*=(-1);
	}
	value=(uint32_t) n;
	usart_send_uint(value);
	n-=value;
	
	usart_write_line(".");
	
	while(n!=0.0 && i<precision)
	{
		n*=10;
		value=(uint32_t) n;
		usart_send_uint(value);
		n-=value;
		i++;
	}
}

void usart_write_line(const char *string)
{
	while(*string!='\0')
	{
		USART_Tx(*string++);
	}
}