#include <stdint.h>
#include "input.h"
#include "src_adc.h"
#include "src_twi_adc.h"
#include "src_sw.h"
#include "src_nunchuk.h"

#include "config.h"

int16_t get_input_range(uint8_t src, uint8_t max) {
	int16_t val = 0;
	switch (SRC_SYS(src)) {
		case SRC_ADC:
			val = max ? 1023 : 0;
			break;
		case SRC_SW:
			val = max ? 1 : -1;
			break;
#ifdef USE_TWI_ADC
		case SRC_TWI_ADC:
			val = max ? 255 : 0;
			break;
#endif
#ifdef USE_NUNCHUK
		case SRC_NUNCHUK:
			val = nunchuk_get_range(SRC_NUM(src), max);
			break;
#endif
		case SRC_DS: /* SRC_DS will not yield a return value */
		case SRC_NULL:
		default: /* unknown source */
			break;
	}
	return val;
}

int16_t get_input(uint8_t src) {
	int16_t val = 0;
	switch (SRC_SYS(src)) {
		case SRC_ADC:
			val = adc_get_raw(SRC_NUM(src));
			break;
		case SRC_SW:
			val = sw_get_raw(SRC_NUM(src));
			break;
#ifdef USE_TWI_ADC
		case SRC_TWI_ADC:
			val = twi_adc_get_raw(SRC_NUM(src));
			break;
#endif
#ifdef USE_NUNCHUK
		case SRC_NUNCHUK:
			val = nunchuk_get_raw(SRC_NUM(src));
			break;
#endif
		case SRC_DS: /* SRC_DS will not yield a return value */
		case SRC_NULL:
		default: /* unknown source */
			break;
	}
	return val;
}

int16_t get_input_scaled(uint8_t src, int16_t min, int16_t max) {
	int32_t smax = get_input_range(src, 1);
	int32_t smin = get_input_range(src, 0);
	int32_t value = get_input(src);
	int32_t srange = smax-smin;
	int32_t r_value = value-smin;

	int32_t omax = max-min;
	if (!srange) return 0;
	return ((int32_t)omax*r_value/srange)+min;
}
