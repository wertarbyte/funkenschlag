#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/delay.h>
#include "twi.h"
#include "mag.h"
#include "serial.h"
#include "config.h"

#ifdef USE_MAG

/* HMC5883 */
#define MAG_ADDRESS 0x1E
#define MAG_DATA_REGISTER 0x03

#ifndef MAG_ORIENTATION
#define MAG_ORIENTATION(X, Y, Z)  {mag_data[M_X]  = -(X); mag_data[M_Y]  = -(Y); mag_data[M_Z]  = (Z);}
#endif

static int16_t mag_data[3];

#define M_X 0
#define M_Y 1
#define M_Z 2

void mag_init(void) {
	_delay_ms(100);
	twi_write_reg(MAG_ADDRESS, 0x00, 0x70);
	twi_write_reg(MAG_ADDRESS, 0x01, 0x20);
	twi_write_reg(MAG_ADDRESS, 0x02, 0x00);
}

void mag_query(void) {
	uint8_t buf[6];
	twi_start(MAG_ADDRESS<<1);
	twi_write(MAG_DATA_REGISTER);
	twi_start(MAG_ADDRESS<<1 | 1);
	for (uint8_t i=0; i<sizeof(buf); i++) {
		buf[i] = twi_read(i<sizeof(buf)-1);
	}
	twi_stop();

	MAG_ORIENTATION(buf[0]<<8 | buf[1],
			buf[4]<<8 | buf[5],
			buf[2]<<8 | buf[3]);
}

uint16_t mag_heading(void) {
	float heading = atan2(mag_data[M_Y], mag_data[M_X]);
	if (heading < 0) heading += 2*M_PI;
	uint16_t headingDeciDegrees = heading * 1800/M_PI;
	return headingDeciDegrees;
}

void mag_dump(void) {
	serial_write_str("MAG: ");
	for (uint8_t i=0; i<3; i++) {
		serial_write_int(mag_data[i]);
		serial_write(' ');
	}
	serial_write_int( mag_heading() );
	serial_write('\n');
}

#endif
