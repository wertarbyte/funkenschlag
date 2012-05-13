#include <stdint.h>
#include <string.h>
#include <util/atomic.h>

#include "datenschlag.h"
#include "datenschlag_structs.h"

#define DS_TX_BUFFER_SIZE 5

static struct ds_frame tx_buffer[DS_TX_BUFFER_SIZE];
static volatile uint8_t tx_buffer_start = 0;

static volatile uint8_t tx_buffer_items = 0;

uint8_t ds_frame_buffers_available(void) {
	return DS_TX_BUFFER_SIZE-tx_buffer_items;
}

uint8_t ds_add_frame(uint8_t cmd, uint8_t *payload, uint8_t size) {
	if (!ds_frame_buffers_available()) {
		return 0;
	}

	/* clear the next free slot in the tx queue */
	struct ds_frame *f = &tx_buffer[(tx_buffer_start+tx_buffer_items)%DS_TX_BUFFER_SIZE];
	memset(f, 0, sizeof(*f));
	/* fill the frame */
	f->cmd = cmd;
	/* do not copy more payload data than the frame can hole */
	uint8_t pl_size = size > sizeof(f->data) ? sizeof(f->data) : size;
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

int8_t ds_get_next_byte(uint8_t *dst) {
	static int8_t i = -2;

	/* nothing to send? */
	if (tx_buffer_items == 0) {
		return 0;
	}
	/* a new  frame just started, send calibration pulse */
	if (i < 0) {
		return i++;
	}

	/* now that i >= 0, the real data transmission starts */
	struct ds_frame *f = &tx_buffer[tx_buffer_start];
	uint8_t *b = (uint8_t*) f;
	/* select the n'th byte out of our frame struct */
	if (i < sizeof(*f)) {
		*dst = b[i];
		i++;
	}
	/* end of frame reached */
	if (i == sizeof(*f)) {
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
			/* we consumed one frame */
			tx_buffer_items--;
			/* move the start pointer to the next one */
			tx_buffer_start++;
			if (tx_buffer_start == DS_TX_BUFFER_SIZE) {
				tx_buffer_start = 0;
			}
		}
		i = -2;
	}
	return 1;
}
