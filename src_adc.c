#include <stdlib.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include "config.h"
#include "src_adc.h"

static uint8_t adc_map[] = {
	0,
	1,
	2,
	3,
#ifdef USE_ADC4_ADC5
	4,
	5,
#endif
};

#define ADC_CHANNELS ( sizeof(adc_map)/sizeof(*adc_map) )

static uint8_t adc_invert[(ADC_CHANNELS+7)/8] = { 0 };

static int16_t adc_trim[ADC_CHANNELS] = {
	[0] = -10,
	[1] = 40,
	[2] = 0,
	[3] = 15,
};
static int8_t adc_scale[ADC_CHANNELS] = {
	[0] = 50,
	[1] = 40,
	[2] = 30,
	[3] = 25,
};

uint16_t adc_raw   [ADC_CHANNELS] = {0};
int16_t  adc_values[ADC_CHANNELS] = {0};

static int16_t read_adc(uint8_t adc) {
	int16_t result = 0;
#define ADC_READS 8
	uint8_t reads = ADC_READS;
	while (reads--) {
		/* set input */
		ADMUX = ( (ADMUX & 0xF0) | (0x0F & adc_map[adc]) );
		/* start conversion */
		ADCSRA |= (1<<ADSC);
		/* wait for completion */
		while (ADCSRA & (1<<ADSC)) {};
		result += ADC;
	}
	result /= ADC_READS;
	adc_raw[adc] = result;
	/* is this axis inverted? */
	if (adc_invert[adc/(8*sizeof(*adc_invert))] & 1<<(adc%(8*sizeof(*adc_invert)))) {
		result = 1023-result;
	}

	/* adjust the channel value */
	result += adc_trim[adc];
	if (adc_scale[adc]) {
		int32_t d = (int32_t)1023/2 - result;
		int32_t nd = (d*(100+adc_scale[adc])/(100));
		result = 1023/2 - nd;
	}
	return result;
}

int16_t adc_get(uint8_t n) {
	return adc_values[n];
}

uint16_t adc_get_raw(uint8_t n) {
	return adc_raw[n];
}

void adc_query(void) {
	/* keep sampling adc data */
	for (uint8_t adc = 0; adc < ADC_CHANNELS; adc++) {
		adc_values[adc] = read_adc(adc);
	}
}

void adc_init(void) {
	ADCSRA = (1<<ADEN | 1<<ADPS2 | 1<<ADPS1 | 0<<ADPS0);
	ADMUX = (1<<REFS0);

	/* mark roll axis as inverted */
	adc_invert[0] |= 1<<0;
}
