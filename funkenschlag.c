#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include "datenschlag.h"
#include "datenschlag_structs.h"

#define N_CHANNELS 6
#define ADC_CHANNELS 4
#define SW_CHANNELS 1

#define DS_CHANNEL 5

#define N_3W_SWITCHES 4

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

static uint8_t adc_map[ADC_CHANNELS] = {
	0,
	1,
	2,
	3,
};

static uint8_t sw_map[SW_CHANNELS] = {
	0,
};

static uint8_t adc_invert[(ADC_CHANNELS+7)/8] = { 0 };

static int16_t trim[N_CHANNELS] = {
	[0] = -10,
	[1] = 40,
	[2] = 0,
	[3] = 15,
	[4] = 25,
	[5] = 0,
};
static int8_t scale[N_CHANNELS] = {
	[0] = 50,
	[1] = 40,
	[2] = 30,
	[3] = 25,
	[5] = 0,
};

/* we are using switches with 3 positions (neutral, up, down),
 * each switch using a single input pin through multiplexing
 */
static struct {
	volatile uint8_t *pin;
	volatile uint8_t *port;
	uint8_t bit;
} sw_inputs[N_3W_SWITCHES] = {
	{ &PIND, &PORTD, PD7 },
	{ &PIND, &PORTD, PD6 },
	{ &PINB, &PORTB, PB6 },
	{ &PINB, &PORTB, PB7 },
};

static int8_t sw_positions[N_3W_SWITCHES] = {0};

static uint16_t adc_values[ADC_CHANNELS] = {0};

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
		uint8_t sw_id = sw_map[i-ADC_CHANNELS];
		val = (1023/2)+(1023/2)*sw_positions[sw_id];
	} else if (i == DS_CHANNEL) {
		val = ds_get_next_pulse();
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
		ADMUX = ( (ADMUX & 0xF0) | (0x0F & adc_map[adc]) );
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
	for (uint8_t sw = 0; sw < N_3W_SWITCHES; sw++) {
		*sw_inputs[sw].port |= 1<<sw_inputs[sw].bit;
	}

	/* configure ADC */
	ADCSRA = (1<<ADEN | 1<<ADPS2 | 1<<ADPS1 | 0<<ADPS0);
	ADMUX = (1<<REFS0);

	/* configure watchfog timer to reset after 60ms */
	wdt_enable(WDTO_60MS);

	/* configure timer */

	/* enable CTC waveform generation (TOP == OCR1A) */
	TCCR1B |= (1<<WGM12);
	/* set compare value for the stop pulse to 300µs */
	OCR1B = STOP_US;
	/* set pulse width to max for now */
	OCR1A = ~0;
	/* set Timer 1 to clk/8, giving us ticks of 1 µs */
	TCCR1B |= (1<<CS11);

	/* Timer 2 generates overflows at 1kHz */
#if defined(TCCR2) /* e.g. ATMega8 */
#define TIMER2_COMP_IRQ TIMER2_COMP_vect
	TCCR2 = (1<<WGM21 | 1<<CS22);
	OCR2 = 0x7D;
	/* enable compare and overflow interrupts */
	TIMSK = (1<<OCIE2 | 1<<OCIE1B | 1<<OCIE1A);
#elif defined(TCCR2A) /* e.g. ATMega{8,16,32}8 */
#define TIMER2_COMP_IRQ TIMER2_COMPA_vect
	TCCR2A = (1<<WGM21);
	TCCR2B = (1<<CS22);
	OCR2A = 0x7D;
	/* enable compare and overflow interrupts */
	TIMSK1 = (1<<OCIE1B | 1<<OCIE1A);
	TIMSK2 = (1<<OCIE2A);
#else
#error "Unable to determine timer 2 configuration registers"
#endif

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
		for (uint8_t sw = 0; sw < N_3W_SWITCHES; sw++) {
			/* check with multiplexing on high wire */
			MPX_PORT &= ~(1<<MPX_BIT);
			/* give the wires some time */
			_delay_us(10);
			uint8_t up = !!(~(*sw_inputs[sw].pin) & 1<<sw_inputs[sw].bit);
			/* check with multiplexing on low wire */
			MPX_PORT |= 1<<MPX_BIT;
			/* give the wires some time */
			_delay_us(10);
			uint8_t down = !!(~(*sw_inputs[sw].pin) & 1<<sw_inputs[sw].bit);
			sw_positions[sw] = (-1*down)+(1*up);
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

		/* check switch for Datenschlag */
		static uint8_t old_switch = 0;
		uint8_t ds_payload[DS_MAX_PAYLOAD_LENGTH] = {0};
		for (uint8_t i=SW_CHANNELS; i<N_3W_SWITCHES; i++) {
			switch (sw_positions[i]) {
				case 0:
					ds_payload[0] |= 1<<i;
					break;
				case 1:
					ds_payload[0] |= 1<<i;
					ds_payload[0] |= 1<<(i+4);
					break;
				case -1:
					ds_payload[0] |= 1<<(i+4);
					break;
			}
		}
		#define DS_CMD_AUX (1<<5| 0x0A)
		/* if the switch state changes, remove obsolete frames from the queue */
		if (old_switch != ds_payload[0]) {
			while (ds_abort_frame(DS_CMD_AUX));
			old_switch = ds_payload[0];
		}
		/* queue datenschlag frame */
		if (ds_frame_buffers_available()) {
			/* 0x2A: 001 01010 */
			ds_add_frame(DS_CMD_AUX, &ds_payload[0], 1);
		}
#ifdef SEND_MAG_HEADING
		/* send orientation */
		#define DS_CMD_SET_HEADING (2<<5 | 0x04)
		static int16_t dir = 0; // set real orientation here
		ds_payload[0] = dir>>8;
		ds_payload[1] = dir&0x0F;
		if (!ds_frame_queued(DS_CMD_SET_HEADING) && ds_frame_buffers_available()) {
			ds_add_frame(DS_CMD_SET_HEADING, &ds_payload[0], 2);
		}
#endif
	}
	return 0;
}

/* 1ms has passed, increment the counter */
ISR(TIMER2_COMP_IRQ) {
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
