#ifndef UART_H_INCLUDED
#define UART_H_INCLUDED

//------------UART0-------------
void Uart0Init(void);				// Initialize
void Uart0Putc(uint8_t u8Data);		// Send Byte
uint8_t Uart0Getc(void);			// Receive Byte
void Uart0Puts(const char *s );		// Send String
void Uart0Println(const char *s );	// Send String followed by '\n \r' [line feed and carriage return]

void Uart0Puts_P(const char *s );	// Send Strings Stored in PROGRAM MEMORY (FLASH).

#endif