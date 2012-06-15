#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

void serial_init(void) {
	#define BAUD 38400
	#include <util/setbaud.h>
	UBRRH = UBRRH_VALUE;
	UBRRL = UBRRL_VALUE;
	#if USE_2X /* output from setbaud.h */
		UCSRA |= 1<<U2X;
	#else
		UCSRA &= ~(1<<U2X);
	#endif
	#undef BAUD
	UCSRB |= (1<<TXEN | 1<<RXEN);
	UCSRC |= (1<<URSEL | 1<<UCSZ1 | 1<<UCSZ0);
}

void serial_write(char c) {
	while(!(UCSRA & (1<<UDRE)));
	UDR = c;
}

void serial_write_str(char *s) {
	while (*s) {
		serial_write(*s++);
	}
}
