#include <stdint.h>
#include "input.h"
#include "config.h"

isrc_t channel_source[] =
#ifdef CHANNEL_SRC
	CHANNEL_SRC
#else
#error "No channels have been defined. Please check CHANNEL_SRC in config.h."
#endif
;

#define N_CHANNELS (sizeof(channel_source)/sizeof(*channel_source))
uint8_t channel_count = N_CHANNELS;



