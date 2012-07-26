#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/delay.h>
#include <math.h>
#include "twi.h"
#include "acc.h"
#include "serial.h"
#include "config.h"

#ifdef USE_ACC

/* ADXL345 */
#define ACC_ADDRESS 0x53
#define ACC_SCALE_FACTOR_2G 0.0039

#ifndef ACC_ORIENTATION
#define ACC_ORIENTATION(X, Y, Z)  {acc_data[A_X]  = (X); acc_data[A_Y]  = (Y); acc_data[A_Z]  = (Z);}
#endif

static int16_t acc_data[3];

#define A_X 0
#define A_Y 1
#define A_Z 2

void acc_init(void) {
	_delay_ms(10);
	twi_write_reg(ACC_ADDRESS, 0x2D, 1<<3);
	twi_write_reg(ACC_ADDRESS, 0x31, 0x0B);
	twi_write_reg(ACC_ADDRESS, 0x2C, 0x09);
}

void acc_query(void) {
	uint8_t buf[6];
	twi_start(ACC_ADDRESS<<1);
	twi_write(0x32);
	twi_start(ACC_ADDRESS<<1 | 1);
	for (uint8_t i=0; i<sizeof(buf); i++) {
		buf[i] = twi_read(i<sizeof(buf)-1);
	}
	twi_stop();

	ACC_ORIENTATION(buf[1]<<8 | buf[0],
			buf[3]<<8 | buf[2],
			buf[5]<<8 | buf[4]);
}

void acc_get_scaled_data(float scaled[3]) {
	for (uint8_t i=0; i<3; i++) {
		scaled[i] = acc_data[i];
	}
}

void acc_dump(void) {
#ifdef ENABLE_DEBUG_SERIAL_DUMP
	serial_write_str("ACC: ");
	for (uint8_t i=0; i<3; i++) {
		serial_write_int(acc_data[i]);
		serial_write(' ');
	}
	serial_write('\n');
#endif
}

#endif
