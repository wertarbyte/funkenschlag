/* each command has a fixed amount of payload associated with it.
 * The number of payload bytes is encoded in the highest 3 bits of
 * the command byte (although capped at DS_MAX_PAYLOAD_LENGTH)
 */
#define DS_CMD_PAYLOAD_SIZE(x) ( (x & 0xFF) >> 5 )

uint8_t ds_add_frame(uint8_t cmd, uint8_t *payload, uint8_t size);
uint16_t ds_get_next_pulse(void);
uint8_t ds_frame_buffers_available(void);
