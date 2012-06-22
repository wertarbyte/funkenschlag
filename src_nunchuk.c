#include <stdlib.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <string.h>
#include "config.h"
#include "serial.h"
#include "src_nunchuk.h"
#include "twi.h"

#ifdef USE_NUNCHUK

#define NUNCHUK_ADDR 0x52

#define ZEROX 510
#define ZEROY 490
#define ZEROZ 460
#define RADIUS 210.0

enum {
	NC_ROLL,
	NC_PITCH,
	NC_JOY_X,
	NC_JOY_Y,
	NC_ACC_X,
	NC_ACC_Y,
	NC_ACC_Z,

	NC_MAX,
};

static int16_t ref_angle[2];


int16_t nunchuk_raw   [NC_MAX] = {0};
int16_t nunchuk_values[NC_MAX] = {0};

int16_t nunchuk_get(uint8_t n) {
	return nunchuk_values[n];
}

int16_t nunchuk_get_raw(uint8_t n) {
	return nunchuk_raw[n];
}

void nunchuk_dump(void) {
	serial_write_str("Nunchuk: ");
	for (uint8_t i=NC_ROLL; i<=NC_JOY_Y; i++) {
		serial_write_int(nunchuk_raw[i]);
		serial_write(' ');
	}
	serial_write('(');
	for (uint8_t i=0; i<2; i++) {
		serial_write_int(ref_angle[i]);
		serial_write(' ');
	}
	serial_write(')');
	serial_write('\n');
}

void nunchuk_query(void) {
	uint8_t nunchuk_data[6];
	memset(nunchuk_data, 0, sizeof(nunchuk_data));

	_delay_ms(1);
	twi_start(NUNCHUK_ADDR<<1);
	twi_write(0x00);
	twi_stop();
	_delay_ms(1);

	twi_start(NUNCHUK_ADDR<<1 | 1);
	for (uint8_t i=0; i<6; i++) {
		nunchuk_data[i] = twi_read(i < 5);
	}
	twi_stop();

	nunchuk_raw[NC_JOY_X] = nunchuk_data[0];
	nunchuk_raw[NC_JOY_Y] = nunchuk_data[1];

	nunchuk_raw[NC_ACC_X] = ( ((uint16_t)nunchuk_data[2]<<2) | ((nunchuk_data[5]>>2)&0x03) ) - ZEROX;
	nunchuk_raw[NC_ACC_Y] = ( ((uint16_t)nunchuk_data[3]<<2) | ((nunchuk_data[5]>>4)&0x03) ) - ZEROY;
	nunchuk_raw[NC_ACC_Z] = ( ((uint16_t)nunchuk_data[4]<<2) | ((nunchuk_data[5]>>6)&0x03) ) - ZEROZ;

	//float radius = sqrt(nunchuk_raw[NC_ACC_X]*nunchuk_raw[NC_ACC_X] + nunchuk_raw[NC_ACC_Y]*nunchuk_raw[NC_ACC_Y] + nunchuk_raw[NC_ACC_Z]*nunchuk_raw[NC_ACC_Z]);
	float radius = RADIUS;

	int16_t roll = (atan2((float)nunchuk_raw[NC_ACC_X], (float)nunchuk_raw[NC_ACC_Z])/ M_PI * 180.0);
	int16_t pitch = 180-(acos((float)nunchuk_raw[NC_ACC_Y]/radius)/ M_PI * 180.0) - 90;

	/* as long as Z is not pressed, keep saving the reference angle */
	if (!!(nunchuk_data[5] & 1<<0)) {
		/* Z is now unpressed */
		ref_angle[0] = roll;
		ref_angle[1] = pitch;
	}
	nunchuk_raw[NC_ROLL] = roll - ref_angle[0];
	nunchuk_raw[NC_PITCH] = pitch - ref_angle[1];
	for (uint8_t i=NC_ROLL; i<=NC_PITCH; i++) {
		if (nunchuk_raw[i] < -180) nunchuk_raw[i] += 360;
		if (nunchuk_raw[i] >  180) nunchuk_raw[i] += 360;
	}

#ifndef NUNCHUK_CTRL_ANGLE
#define NUNCHUK_CTRL_ANGLE 30
#endif
	for (uint8_t i=NC_ROLL; i<=NC_PITCH; i++) {
		int16_t angle = nunchuk_raw[i];
		if (angle < -(NUNCHUK_CTRL_ANGLE)) angle = -(NUNCHUK_CTRL_ANGLE);
		if (angle >  (NUNCHUK_CTRL_ANGLE)) angle =  (NUNCHUK_CTRL_ANGLE);
		nunchuk_values[i] = 1023L*(angle+(NUNCHUK_CTRL_ANGLE))/(2*(NUNCHUK_CTRL_ANGLE));
	}
	for (uint8_t i=NC_JOY_X; i<=NC_JOY_Y; i++) {
		nunchuk_values[i] = 1023L*nunchuk_raw[i]/255;
	}
}

void nunchuk_init(void) {
	twi_start(NUNCHUK_ADDR<<1);
	twi_write(0xF0);
	twi_write(0x55);
	twi_stop();

	twi_start(NUNCHUK_ADDR<<1);
	twi_write(0xFB);
	twi_write(0x01);
	twi_stop();
}

#endif
