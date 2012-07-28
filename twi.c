#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/twi.h>
#include <util/delay.h>
#include "twi.h"
#include "config.h"

#ifndef TWI_BITRATE
#define TWI_BITRATE 100000UL
#endif

uint16_t twi_error_cnt;

void twi_init(void) {
	TWSR = 0;
	TWBR = (F_CPU / (TWI_BITRATE) - 16) / 2;
	TWCR = 1<<TWEN;
	twi_error_cnt = 0;
}

static uint8_t twi_wait(void) {
#ifdef TWI_TIMEOUT
	uint8_t count = 255;
#endif
	while (!(TWCR & (1<<TWINT))) {
#ifdef TWI_TIMEOUT
		if (! --count) {
			twi_error_cnt++;
			return 1;
		}
#endif
	}
	return 0;
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

void twi_write_reg(uint8_t addr, uint8_t reg, uint8_t val) {
	twi_start(addr<<1);
	twi_write(reg);
	twi_write(val);
	twi_stop();
}
