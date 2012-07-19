#include <stdint.h>
#include "input.h"
#include "config.h"

isrc_t channel_source[] =
#ifdef CHANNEL_SRC
	CHANNEL_SRC
#else
{
	SRC_ID(SRC_ADC, 0),
	SRC_ID(SRC_ADC, 1),
	SRC_ID(SRC_ADC, 2),
	SRC_ID(SRC_ADC, 3),
	SRC_ID(SRC_SW,  0),
	SRC_ID(SRC_DS,  0),
#ifdef USE_ADC4_ADC5
	SRC_ID(SRC_ADC, 4),
	SRC_ID(SRC_ADC, 5),
#elif defined(USE_TWI_ADC)
	SRC_ID(SRC_TWI_ADC, 0),
	SRC_ID(SRC_TWI_ADC, 1),
#endif
}
#endif
;

#define N_CHANNELS (sizeof(channel_source)/sizeof(*channel_source))
uint8_t channel_count = N_CHANNELS;



