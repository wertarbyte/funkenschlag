#include <stdint.h>
#include <string.h>

#include "datenschlag.h"
#include "datenschlag_structs.h"

#define DS_TX_BUFFER_SIZE 5

static struct ds_frame tx_buffer[DS_TX_BUFFER_SIZE];
static uint8_t tx_buffer_r = 0;

static uint8_t tx_buffer_queue = 0;

uint8_t ds_add_frame(uint8_t cmd, uint8_t *payload, uint8_t size) {
	if (tx_buffer_queue == DS_TX_BUFFER_SIZE) {
		return 0;
	}

	struct ds_frame *f = &tx_buffer[(tx_buffer_r+tx_buffer_queue)%DS_TX_BUFFER_SIZE];
	memset(f, 0, sizeof(*f));
	f->cmd = cmd;
	uint8_t pl_size = size > sizeof(f->data) ? sizeof(f->data) : size;
	memcpy(&f->data, payload, pl_size);
	f->chk ^= cmd;
	for (uint8_t i=0; i<sizeof(f->data); i++) {
		f->chk ^= payload[i];
	}
	tx_buffer_queue++;
	return 1;
}

int8_t ds_next_byte(uint8_t *dst) {
	static int8_t i = -2;

	/* nothing to send */
	if (tx_buffer_queue == 0) {
		return 0;
	}
	/* a new  frame just started, send calibration pulse */
	if (i < 0) {
		return i++;
	}

	struct ds_frame *f = &tx_buffer[tx_buffer_r];
	uint8_t *b = (uint8_t*) f;
	if (i < sizeof(*f)) {
		*dst = b[i];
		i++;
	}
	/* end of frame reached */
	if (i == sizeof(*f)) {
		tx_buffer_queue--;
		tx_buffer_r++;
		i = -2;
	}
	return 1;
}
