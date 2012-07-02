#include <stdlib.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include "config.h"
#include "serial.h"
#include "src_twi_adc.h"
#include "twi.h"

#ifdef USE_TWI_ADC

/* PCF8591 I²C ADC/DAC */
#define TWI_ADC_ADDR 0x48
#define TWI_ADC_CHANNELS 4

uint16_t twi_adc_raw   [TWI_ADC_CHANNELS] = {0};
int16_t  twi_adc_values[TWI_ADC_CHANNELS] = {0};

int16_t twi_adc_get(uint8_t n) {
	return twi_adc_values[n];
}

uint16_t twi_adc_get_raw(uint8_t n) {
	return twi_adc_raw[n];
}

void twi_adc_query(void) {
	/* keep sampling adc data */
	twi_start(TWI_ADC_ADDR<<1);
	/* start at AIN0, autoincrement through channels */
	twi_write(1<<2 | (0 & 0x03));
	twi_stop();
	twi_start((TWI_ADC_ADDR<<1) | 1);
	twi_read(1); /* discard result code of precious cycle */
	for (uint8_t adc = 0; adc < TWI_ADC_CHANNELS; adc++) {
		twi_adc_raw[adc] = twi_read(adc < TWI_ADC_CHANNELS-1);
		/* scale to 10 bit values */
		twi_adc_values[adc] = twi_adc_raw[adc]<<2;
	}
}

void twi_adc_init(void) {
}

#endif
