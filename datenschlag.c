#include <stdint.h>
#include <string.h>
#include <util/atomic.h>
#include <stddef.h>

#include "datenschlag.h"
#include "datenschlag_structs.h"

#define DS_RETRANSMITS 3
#define DS_TX_BUFFER_SIZE 5

struct ds_qframe {
	struct ds_frame frame;
	uint8_t transmits;
};

static struct ds_qframe tx_buffer[DS_TX_BUFFER_SIZE];
static volatile uint8_t tx_buffer_start = 0;

static volatile uint8_t tx_buffer_items = 0;

static void advance_to_next_frame(void) {
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		/* we consumed one frame */
		tx_buffer_items--;
		/* move the start pointer to the next one */
		tx_buffer_start++;
		if (tx_buffer_start == DS_TX_BUFFER_SIZE) {
			tx_buffer_start = 0;
		}
	}
}

uint8_t ds_frame_buffers_available(void) {
	return DS_TX_BUFFER_SIZE-tx_buffer_items;
}

uint8_t ds_frame_queued(uint8_t cmd) {
	uint8_t n = 0;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		for (uint8_t i=tx_buffer_start; i!=(tx_buffer_start+tx_buffer_items)%DS_TX_BUFFER_SIZE; i=(i+1)%DS_TX_BUFFER_SIZE) {
			struct ds_frame *f = &tx_buffer[i].frame;
			if (f->cmd == cmd) {
				n++;
			}
		}
	}
	return n;
}

uint8_t ds_add_frame(uint8_t cmd, uint8_t *payload, uint8_t size) {
	if (!ds_frame_buffers_available()) {
		return 0;
	}

	/* clear the next free slot in the tx queue */
	struct ds_qframe *qf = &tx_buffer[(tx_buffer_start+tx_buffer_items)%DS_TX_BUFFER_SIZE];
	qf->transmits = DS_RETRANSMITS;
	struct ds_frame *f = &qf->frame;
	memset(f, 0, sizeof(*f));
	/* fill the frame */
	f->cmd = cmd;
	/* do not copy more payload data than the frame can hole */
	uint8_t pl_size = size > sizeof(f->data) ? sizeof(f->data) : size;
	/* and not more than the command type specifies */
	if (pl_size > DS_CMD_PAYLOAD_SIZE(cmd)) {
		pl_size = DS_CMD_PAYLOAD_SIZE(cmd);
	}
	memcpy(&f->data, payload, pl_size);
	/* calculate checksum */
	f->chk ^= cmd;
	for (uint8_t i=0; i<sizeof(f->data); i++) {
		f->chk ^= payload[i];
	}

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		tx_buffer_items++;
	}
	return 1;
}

uint8_t ds_abort_frame(uint8_t cmd) {
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		if (! tx_buffer_items) return 0;
		if (tx_buffer[tx_buffer_start].frame.cmd != cmd) return 0;
		advance_to_next_frame();
	}
	return 1;
}

#define CALIBRATION_FRAMES 2
static int8_t ds_get_next_nibble(uint8_t *dst, uint8_t peek_only) {
	static int8_t i = -(CALIBRATION_FRAMES);

	/* nothing to send? */
	if (tx_buffer_items == 0) {
		return 0;
	}
	/* a new  frame just started, send calibration pulse */
	if (i < 0) {
		return peek_only ? i : i++;
	}

	/* now that i >= 0, the real data transmission starts */
	struct ds_qframe *q = &tx_buffer[tx_buffer_start];
	struct ds_frame *f = &q->frame;
	uint8_t *b = (uint8_t*) f;
	/* select the n'th byte out of our frame struct */
	if (i < 2*sizeof(*f)) {
		if (i%2) {
			*dst = (b[i/2] & 0xF0)>>4;
		} else {
			*dst = b[i/2] & 0x0F;
		}
		/* since consecutive identical nibbles slow down the transfer,
		 * we flip a single bit in each nibble to add diversity
		 */
		*dst ^= 1<<i%4;
		if (!peek_only) {
			i++;
			/* if we already transmitted the entire payload needed by the cmd,
			 * jump to the checksum
			 */
			if (i == 2*(DS_CMD_PAYLOAD_SIZE(f->cmd)+offsetof(struct ds_frame, data))) {
				i = 2*offsetof(struct ds_frame, chk);
			}
		}
	}
	/* end of frame reached */
	if (!peek_only && i >= 2*sizeof(*f)) {
		if (! --q->transmits) {
			advance_to_next_frame();
			i = -(CALIBRATION_FRAMES);
		}
	}
	return 1;
}

#define DATENSCHLAG_MAX_PULSE_LENGTH 1100
uint16_t ds_get_next_pulse(void) {
	uint16_t val = 0;
	/* did the last pulse send a data nibble? */
	static int8_t nibble_sent = 0;
	static uint8_t last_nibble = 0;
	uint8_t peek_value = 0;
	int8_t p_r = ds_get_next_nibble(&peek_value, 1);
	/* did the last pulse sent a nibble? Is the next nibble identical? */
	if (nibble_sent && p_r > 0 && peek_value == last_nibble) {
		/* if yes, close the deal by pulling the channel low */
		nibble_sent = 0;
		val = 0;
	} else {
		uint8_t v = 0;
		int8_t r = ds_get_next_nibble(&v, 0);
		/* calibration pulses before a frame */
		if (r <= 0) {
			val = (r<=-2 ? 0 : DATENSCHLAG_MAX_PULSE_LENGTH);
		} else if (r) {
			/* transmit the binary value of v */
			val = ((uint32_t)DATENSCHLAG_MAX_PULSE_LENGTH*((v&0x0F)+1))/(0x0F+2);
			nibble_sent = 1;
			last_nibble = v;
		}
	}
	return val;
}
