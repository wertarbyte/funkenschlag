#include <stdlib.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include "datenschlag.h"

#define N_CHANNELS 6
#define ADC_CHANNELS 4
#define SW_CHANNELS 1

#define DS_CHANNEL 6

#define PPM_DDR  DDRB
#define PPM_PORT PORTB
#define PPM_PIN  PINB
#define PPM_BIT  PB0

#define LED_DDR  DDRB
#define LED_PORT PORTB
#define LED_PIN  PINB
#define LED_BIT  PB5

#define VOL_DDR  DDRB
#define VOL_PORT PORTB
#define VOL_PIN  PINB
#define VOL_BIT  PB1

/* switch multiplexing */
#define MPX_DDR  DDRB
#define MPX_PORT PORTB
#define MPX_PIN  PINB
#define MPX_BIT  PB2

#define FRAME_US 20000L
#define STOP_US 500

static uint8_t adc_inputs[ADC_CHANNELS] = {
	0,
	1,
	2,
	3,
};

static uint8_t adc_invert[(ADC_CHANNELS+7)/8] = { 0 };

static int16_t trim[N_CHANNELS] = {
	[0] = -10,
	[1] = 40,
	[2] = 0,
	[3] = 15,
	[4] = 25,
	[5] = 25
};
static int8_t scale[N_CHANNELS] = {
	[0] = 50,
	[1] = 40,
	[2] = 25,
	[3] = 25,
};

/* we are using switches with 3 positions (neutral, up, down),
 * so each switch uses two digital input pins
 */
static struct {
	volatile uint8_t *pin;
	volatile uint8_t *port;
	uint8_t bit;
} sw_inputs[SW_CHANNELS] = {
	{ &PIND, &PORTD, PD7 },
//	{ &PIND, &PORTD, PD6 },
};

static uint16_t adc_values[ADC_CHANNELS] = {0};
static uint16_t sw_values[SW_CHANNELS] = {0};

static uint8_t current_channel;
static uint16_t frame_time_remaining = 0;
static uint16_t frame_times[N_CHANNELS] = {0};

/* milliseconds since startup */
static volatile uint32_t millis = 0;

static void set_ppm(uint8_t h) {
	if (h) {
		PPM_PORT |= 1<<PPM_BIT;
	} else {
		PPM_PORT &= ~(1<<PPM_BIT);
	}
}

static uint16_t get_channel(uint8_t i) {
	uint16_t val = 0;
	if (i < ADC_CHANNELS) {
		val = adc_values[i];
	} else if (i < ADC_CHANNELS+SW_CHANNELS) {
		val = sw_values[i-ADC_CHANNELS];
	} else if (i == DS_CHANNEL-1) {
		uint8_t v = 0;
		uint8_t r = ds_get_next_byte(&v);
		/* calibration pulses before a frame */
		if (r < -64) {
			val = 0;
		} else if (r < 0) {
			val = 1023;
		} else if (r) {
			/* transmit the binary value of v */
			val = ((uint32_t)1023*(v+1)/(255+2));
		} else {
			val = 0;
		}
	} else {
		return 0;
	}
	/* adjust the channel value */
	val += trim[i];
	if (scale[i]) {
		int32_t d = (int32_t)1023/2 - val;
		int32_t nd = (d*(100+scale[i])/(100));
		val = 1023/2 - nd;
	}
	return val;
}

static void start_ppm_frame(void) {
	current_channel = 0;
	frame_time_remaining = FRAME_US;
	for (uint8_t i=0; i<N_CHANNELS; i++) {
		frame_times[i] = (1000+get_channel(i));
		frame_time_remaining -= frame_times[i];
	}
}

static void start_ppm_pulse(void) {
	if (current_channel < N_CHANNELS) {
		/* get the pulse width for the current channel */
		OCR1A = frame_times[current_channel];
		current_channel++;
	} else {
		/* we already transmitted the last channel, only wait for the frame to finish */
		OCR1A = frame_time_remaining;
		start_ppm_frame();
	}
}

static uint16_t read_adc(uint8_t adc) {
	uint16_t result = 0;
#define ADC_READS 8
	uint8_t reads = ADC_READS;
	while (reads--) {
		/* set input */
		ADMUX = ( (ADMUX & 0xF0) | (0x0F & adc_inputs[adc]) );
		/* start conversion */
		ADCSRA |= (1<<ADSC);
		/* wait for completion */
		while (ADCSRA & (1<<ADSC)) {};
		result += ADC;
	}
	return result/ADC_READS;
}

int main(void) {
	/* configure PPM output port */
	PPM_DDR |= (1<<PPM_BIT);
	PPM_PORT &= ~(1<<PPM_BIT);

	/* configure LED output port */
	LED_DDR |= (1<<LED_BIT);
	LED_PORT |= (1<<LED_BIT);

	/* configure VOL(tage) warning port */
	VOL_DDR &= ~(1<<VOL_BIT);
	VOL_PORT |= (1<<VOL_BIT); // enable pullup

	MPX_DDR |= 1<<MPX_BIT;

	/* enable pull-up resistors for switches */
	for (uint8_t sw = 0; sw < SW_CHANNELS; sw++) {
		*sw_inputs[sw].port |= 1<<sw_inputs[sw].bit;
	}
	PORTD |= 1<<PD6;

	/* configure ADC */
	ADCSRA = (1<<ADEN | 1<<ADPS2 | 1<<ADPS1 | 0<<ADPS0);
	ADMUX = (1<<REFS0);

	/* configure watchfog timer to reset after 60ms */
	wdt_enable(WDTO_60MS);

	/* configure timer */

	/* Timer 2 generates overflows at 1kHz */
	TCCR2 = (1<<WGM21 | 1<<CS22);
	OCR2 = 0x7D;

	/* enable CTC waveform generation (TOP == OCR1A) */
	TCCR1B |= (1<<WGM12);
	/* set compare value for the stop pulse to 300µs */
	OCR1B = STOP_US;
	/* set pulse width to max for now */
	OCR1A = ~0;
	/* set Timer 1 to clk/8, giving us ticks of 1 µs */
	TCCR1B |= (1<<CS11);

	/* enable compare and overflow interrupts */
	TIMSK = (1<<OCIE2 | 1<<OCIE1B | 1<<OCIE1A);

	/* initialize channel data */
	start_ppm_frame();
	set_ppm(1);
	start_ppm_pulse();

	/* enable interrupts */
	sei();

	/* mark roll axis as inverted */
	adc_invert[0] |= 1<<0;

	while (1) {
		/* reset watchdog */
		wdt_reset();

		/* keep sampling adc data */
		for (uint8_t adc = 0; adc < ADC_CHANNELS; adc++) {
			uint16_t val = read_adc(adc);
			/* is this axis inverted? */
			if (adc_invert[adc/(8*sizeof(*adc_invert))] & 1<<(adc%(8*sizeof(*adc_invert)))) {
				val = 1023-val;
			}
			adc_values[adc] = val;
		}
		/* query switches */
		for (uint8_t sw = 0; sw < SW_CHANNELS; sw++) {
			/* check with multiplexing on high wire */
			MPX_PORT &= ~(1<<MPX_BIT);
			/* give the wires some time */
			_delay_us(10);
			uint8_t up = !!(~(*sw_inputs[sw].pin) & 1<<sw_inputs[sw].bit);
			/* check with multiplexing on low wire */
			MPX_PORT |= 1<<MPX_BIT;
			/* give the wires some time */
			_delay_us(10);
			//uint8_t down = 0;
			uint8_t down = !!(~(*sw_inputs[sw].pin) & 1<<sw_inputs[sw].bit);
			sw_values[sw] = 500+(up*500)-(down*500);
		}
		/* check voltage */
		if ((~VOL_PIN) & 1<<VOL_BIT) {
			// everything OK
			LED_PORT |= (1<<LED_BIT);
		} else {
			// voltage dropped, alert the user!
			if (millis/250 % 2) {
				LED_PORT |= (1<<LED_BIT);
			} else {
				LED_PORT &= ~(1<<LED_BIT);
			}

		}

		uint8_t ds_payload[4];
		memset(&ds_payload[0], 0xFF, 4);
		/* check switch for Datenschlag */
		MPX_PORT &= ~(1<<MPX_BIT);
		_delay_us(10);
		if (~PORTD & 1<<PD6) {
			ds_payload[0] = 0;
		}
		/* queue datenschlag frame */
		if (ds_frame_buffers_available()) {
			ds_add_frame(0xDE, &ds_payload[0], 4);
		}
	}
	return 0;
}

/* 1ms has passed, increment the counter */
ISR(TIMER2_COMP_vect) {
	millis++;
}

/* the timer has reached OCR1A, so the current PPM pulse has been completed */
ISR(TIMER1_COMPA_vect) {
	set_ppm(1);
	start_ppm_pulse();
}

/* finished sending the stop pulse */
ISR(TIMER1_COMPB_vect) {
	set_ppm(0);
}
