#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include "config.h"
#include "serial.h"

#ifdef ENABLE_SERIAL

#if defined(UCSRA) /* e.g. ATMega 8*/
	#define UART_CONF_A UCSRA
	#define UART_CONF_B UCSRB
	#define UART_CONF_C UCSRC
	#define UART_CONF_C_VAL (1<<URSEL | 1<<UCSZ1 | 1<<UCSZ0)
	#define UART_BRRH UBRRH
	#define UART_BRRL UBRRL
	#define UART_U2X U2X
	#define UART_UDRE UDRE
	#define UART_TXEN TXEN
	#define UART_RXEN RXEN
	#define UART_BUSY (UART_CONF_A & (1<<UDRE))
	#define UART_UDR UDR
#elif defined(UCSR0A) /* e.g. ATMega_8 */
	#define UART_CONF_A UCSR0A
	#define UART_CONF_B UCSR0B
	#define UART_CONF_C UCSR0C
	#define UART_CONF_C_VAL (1<<UCSZ01 | 1<<UCSZ00)
	#define UART_BRRH UBRR0H
	#define UART_BRRL UBRR0L
	#define UART_U2X U2X0
	#define UART_UDRE UDRE0
	#define UART_TXEN TXEN0
	#define UART_RXEN RXEN0
	#define UART_BUSY (UART_CONF_A & (1<<UDRE0))
	#define UART_UDR UDR0
#else
	#error "Unable to determine UART configuration"
#endif

void serial_init(void) {
	#define BAUD 38400
	#include <util/setbaud.h>
	UART_BRRH = UBRRH_VALUE;
	UART_BRRL = UBRRL_VALUE;
	#if USE_2X /* output from setbaud.h */
		UART_CONF_A |= 1<<UART_U2X;
	#else
		UART_CONF_A &= ~(1<<UART_U2X);
	#endif
	#undef BAUD
	UART_CONF_B |= (1<<UART_TXEN | 1<<UART_RXEN);
	UART_CONF_C = UART_CONF_C_VAL;
}

void serial_write(char c) {
	while(!(UART_BUSY));
	UART_UDR = c;
}

void serial_fwrite(const char *format_string, ...) {
	char str[64];
	va_list args;
	va_start(args, format_string);
	vsnprintf(str, sizeof(str), format_string, args);
	va_end(args);
	serial_write_str(str);
};

void serial_write_str(char *s) {
	while (*s) {
		serial_write(*s++);
	}
}

void serial_write_int(int16_t i) {
	serial_fwrite("%i", i);
}

void serial_write_uint(uint16_t i) {
	serial_fwrite("%u", i);
}

#else
void serial_write(char c) {};
void serial_fwrite(const char *fmt, ...) {};
void serial_write_str(const char *c) {};
void serial_write_int(int16_t i) {};
void serial_write_uint(uint16_t u) {};
#endif
