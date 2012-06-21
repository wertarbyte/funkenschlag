#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/twi.h>
#include "twi.h"

void twi_init(void) {
	TWSR = 0;
	TWBR = (F_CPU / 100000UL - 16) / 2;
	TWCR = 1<<TWEN;
}

static void twi_wait(void) {
	while (!(TWCR & (1<<TWINT)));
}

void twi_start(uint8_t addr) {
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
	twi_wait();
	TWDR = addr;
	TWCR = (1<<TWINT) | (1<<TWEN);
	twi_wait();
}

void twi_stop(void) {
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
}

void twi_write(uint8_t data) {
	TWDR = data;
	TWCR = (1<<TWINT) | (1<<TWEN);
	twi_wait();
}

uint8_t twi_read(uint8_t ack) {
	TWCR = (1<<TWINT) | (1<<TWEN) | (ack? (1<<TWEA) : 0);
	twi_wait();
	uint8_t result = TWDR;
	if (!ack) twi_stop();
	return result;
}
