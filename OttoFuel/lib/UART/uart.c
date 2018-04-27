#include <avr/io.h>
#include "uart.h"
//#define F_CPU 8000000UL
#define USART_BAUDRATE 19200 //9600 Modified Baud Rate for MPC PID Controller
#define UBRR_VALUE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)
#define DATA_REGISTER_EMPTY (1<<UDRE0)
#define RX_COMPLETE (1<<RXC0)
#define FRAMING_ERROR (1<<FE0)
#define PARITY_ERROR (1<<UPE0)
#define DATA_OVERRUN (1<<DOR0)

//------------------------------- UART 0 ------------------------------------------ UART 0 -------------------------

void Uart0Init(void)
	{	
	UCSR0A=(0<<RXC0) | (0<<TXC0) | (0<<UDRE0) | (0<<FE0) | (0<<DOR0) | (0<<UPE0) | (0<<U2X0) | (0<<MPCM0);
	//Enable Rx, Tx n Interrupts for Rx n Tx
	//UCSRB=(0<<RXCIE) | (0<<TXCIE) | (0<<UDRIE) | (1<<RXEN) | (1<<TXEN) | (0<<UCSZ2) | (0<<RXB8) | (0<<TXB8);
	UCSR0B=(1<<RXCIE0) | (1<<TXCIE0) | (0<<UDRIE0) | (1<<RXEN0) | (1<<TXEN0) | (0<<UCSZ02) | (0<<RXB80) | (0<<TXB80);
	// Set frame format to 8 data bits, no parity, 1 stop bit
	//UCSR0C=(1<<UMSEL01) | (0<<UMSEL00) | (0<<UPM01) | (0<<UPM00) | (0<<USBS0) | (1<<UCSZ01) | (1<<UCSZ00) | (0<<UCPOL0);
	// Set frame format to 8 data bits, no parity, 2 stop bit
	UCSR0C=(1<<UMSEL01) | (0<<UMSEL00) | (0<<UPM01) | (0<<UPM00) | (1<<USBS0) | (1<<UCSZ01) | (1<<UCSZ00) | (0<<UCPOL0);
	
	
	// Set Baud Rate (Calculated by Clock Frequency)			
	UBRR0H = (uint8_t)(UBRR_VALUE>>8);				
	UBRR0L = (uint8_t)UBRR_VALUE;
	}

void Uart0Putc(uint8_t u8Data)
{
	while(!(UCSR0A&(1<<UDRE0))){};	//wait while previous byte is completed
	UDR0 = u8Data;					// Transmit data
}

uint8_t Uart0Getc()
{
	while(!(UCSR0A&(1<<RXC0))){};		// Wait for byte to be received
	return UDR0;		// Return received data
}

void Uart0Puts(const char *s )
{
    while (*s) 
      Uart0Putc(*s++);
}

void Uart0Println(const char *s )	// Send String followed by '\n \r' [line feed and carriage return]
{
    while (*s)
    {
		Uart0Putc(*s++);
	}
	Uart0Putc('\n');
    Uart0Putc('\r');	
}

void Uart0Puts_P(const char *s )	// Send Strings Stored in PROGRAM MEMORY (FLASH).
{
	while (pgm_read_byte(s))	// Read Byte from Flash Memory.
	Uart0Putc(pgm_read_byte(s++));
}