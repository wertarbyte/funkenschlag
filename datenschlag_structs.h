#ifndef DATENSCHLAG_STRUCTS_H
#define DATENSCHLAG_STRUCTS_H

#define DS_MAX_PAYLOAD_LENGTH 5

struct ds_frame {
	uint8_t cmd; /* command type with encoded payload size */
	uint8_t data[DS_MAX_PAYLOAD_LENGTH]; /* payload */
	uint8_t chk; /* checksum (XOR all other fields) */
};

#endif
