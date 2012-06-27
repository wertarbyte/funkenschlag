#include <stdlib.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <string.h>
#include "src_sw.h"
#include "src_adc.h"
#include "src_ds.h"
#include "datenschlag_structs.h"
#include "datenschlag.h"
#include "input.h"
#include "config.h"

void ds_prepare(void) {
	#define DS_CMD_ANY 0xFF

	uint8_t ds_payload[DS_MAX_PAYLOAD_LENGTH] = {0};
#ifdef DS_SEND_AUX_SWITCHES
	/* check switches for Datenschlag */
	uint8_t sw[] = DS_SEND_AUX_SWITCHES;
	for (uint8_t i=0; i<sizeof(sw)/sizeof(sw[0]) && i<8; i++) {
		uint8_t n=sw[i];
		if (n==0) continue; // 0 (SRC_NULL) ignores the switch channel
		switch (get_input_scaled(n, -1, 1)) {
			case 0:
				ds_payload[i/4] |= 1<<(i%4);
				break;
			case 1:
				ds_payload[i/4] |= 1<<(i%4);
				ds_payload[i/4] |= 1<<((i%4)+4);
				break;
			case -1:
				ds_payload[i/4] |= 1<<((i%4)+4);
				break;
		}
	}
	#define DS_CMD_AUX (2<<5| 0x0A)
	/* if the switch state changes, remove obsolete frames from the queue */
#ifdef DS_BULLY_UPDATE
	static uint8_t old_switch = 0;
	if (old_switch != ds_payload[0]) {
		while (ds_abort_frame(DS_CMD_ANY));
		old_switch = ds_payload[0];
	}
#endif
	/* queue datenschlag frame */
	if (!ds_frame_queued(DS_CMD_AUX) && ds_frame_buffers_available()) {
		/* 0x4A: 001 01010 */
		ds_add_frame(DS_CMD_AUX, &ds_payload[0], 2);
	}
#endif
#ifdef DS_SEND_MAG_HEADING
	memset(ds_payload, 0, sizeof(ds_payload));
	/* send orientation */
	#define DS_CMD_SET_HEADING (2<<5 | 0x04)
	int16_t dir = 0; // set real orientation here
#ifdef DS_HEADING_INPUT
	dir = get_input_scaled(DS_HEADING_INPUT, -180, 180);
#endif
	if ( dir > 180) dir = dir - 360;
	else if (dir < -180) dir = dir + 360;
	ds_payload[0] = (dir&0xFF00)>>8;
	ds_payload[1] = dir&0x00FF;
	if (!ds_frame_queued(DS_CMD_SET_HEADING) && ds_frame_buffers_available()) {
		ds_add_frame(DS_CMD_SET_HEADING, &ds_payload[0], 2);
	}
#endif
#ifdef DS_SEND_GIMBAL_ANGLE
	memset(ds_payload, 0, sizeof(ds_payload));
	#define DS_GIMBAL_ANGLE_THRESHOLD 30
	/* send orientation */
	#define DS_CMD_SET_GIMBAL (1<<5 | 0x0C)
	/* angle in a range from 0 to 255 (180°) */
	uint8_t angle = 0; // set desired angle here
#ifdef DS_GIMBAL_INPUT
	angle = get_input_scaled(DS_GIMBAL_INPUT, 0, 255);
#endif
	ds_payload[0] = angle;
#ifdef DS_BULLY_UPDATE
	static uint8_t last_angle = 0;
	/* if an angle change exceeds the threshold, abort other messages and transmit at once */
	if ((angle > last_angle && angle-last_angle >= DS_GIMBAL_ANGLE_THRESHOLD) || (angle < last_angle && last_angle-angle >= DS_GIMBAL_ANGLE_THRESHOLD) ) {
		while (ds_abort_frame(DS_CMD_ANY));
		last_angle = angle;
	}
#endif
	if (!ds_frame_queued(DS_CMD_SET_GIMBAL) && ds_frame_buffers_available()) {
		ds_add_frame(DS_CMD_SET_GIMBAL, &ds_payload[0], 1);
	}
#endif
}
