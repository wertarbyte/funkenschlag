#include <stdlib.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define N_CHANNELS 6
#define ADC_CHANNELS 4
#define SW_CHANNELS 2

#define PPM_DDR  DDRB
#define PPM_PORT PORTB
#define PPM_PIN  PINB
#define PPM_BIT  PB0

#define FRAME_MS 20000L
#define STOP_MS  300


static uint8_t adc_inputs[ADC_CHANNELS] = {
	0,
	1,
	2,
	3,
};

static uint8_t adc_invert[(ADC_CHANNELS+7)/8] = { 0 };

static int16_t adc_trim[ADC_CHANNELS] = {0};

/* we are using switches with 3 positions (neutral, up, down),
 * so each switch uses two digital input pins
 */
static struct {
	volatile uint8_t *pin;
	volatile uint8_t *port;
	uint8_t up;
	uint8_t down;
} sw_inputs[SW_CHANNELS] = {
	{ &PINB, &PORTB, PB6, PB7 },
	{ &PIND, &PORTD, PD6, PD7 },
};

static uint16_t adc_values[ADC_CHANNELS] = {0};
static uint16_t sw_values[SW_CHANNELS] = {0};

static uint8_t current_channel;
static uint16_t frame_time_remaining;

static void set_ppm(uint8_t h) {
	if (h) {
		PPM_PORT |= 1<<PPM_BIT;
	} else {
		PPM_PORT &= ~(1<<PPM_BIT);
	}
}

static void start_ppm_frame(void) {
	frame_time_remaining = FRAME_MS*2;
	current_channel = 0;
}

static uint16_t get_channel(uint8_t i) {
	if (i < ADC_CHANNELS) {
		return adc_values[i];
	} else if (i < ADC_CHANNELS+SW_CHANNELS) {
		return sw_values[i-ADC_CHANNELS];
	} else {
		return 0;
	}
}

uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int main(void) {
	/* configure PPM output port */
	PPM_DDR |= (1<<PPM_BIT);
	PPM_PORT &= ~(1<<PPM_BIT);

	/* enable pull-up resistors for switches */
	for (uint8_t sw = 0; sw < SW_CHANNELS; sw++) {
		*sw_inputs[sw].port |= 1<<sw_inputs[sw].up;
		*sw_inputs[sw].port |= 1<<sw_inputs[sw].down;
	}

	/* configure ADC */
	ADCSRA = (1<<ADEN);

	/* configure timer */

	/* enable CTC waveform generation (TOP == OCR1A) */
	TCCR1B |= (1<<WGM12);
	/* enable compare interrupts */
	TIMSK = (1<<OCIE1B | 1<<OCIE1A);
	/* set compare value for the stop pulse to 300µs */
	OCR1B = STOP_MS*2;
	/* set pulse width to max for now */
	OCR1A = ~0;
	/* set Timer 1 to clk/8, giving us ticks of 1/2 µs */
	TCCR1B |= (1<<CS11);

	/* initialize channel data */
	start_ppm_frame();
	set_ppm(1);

	/* enable interrupts */
	sei();

	while (1) {
		/* keep sampling adc data */
		for (uint8_t adc = 0; adc < ADC_CHANNELS; adc++) {
			/* set input */
			ADMUX = ( (ADMUX & 0xF0) | (0x0F & adc_inputs[adc]) );
			/* start conversion */
			ADCSRA |= (1<<ADSC);
			/* wait for completion */
			while (ADCSRA & (1<<ADSC)) {};
			uint16_t val = map(ADC, 0, 1023, 0, 1000);
			val += adc_trim[adc];
			/* is this axis inverted? */
			if (adc_invert[adc/(8*sizeof(*adc_invert))] & 1<<(adc%(8*sizeof(*adc_invert)))) {
				val = 1000-val;
			}
			adc_values[adc] = val;
		}
		/* query switches */
		for (uint8_t sw = 0; sw < SW_CHANNELS; sw++) {
			uint8_t up = !!(~(*sw_inputs[sw].pin) & 1<<sw_inputs[sw].up);
			uint8_t down = !!(~(*sw_inputs[sw].pin) & 1<<sw_inputs[sw].down);
			sw_values[sw] = 500+(up*500)-(down*500);
		}
	}
	return 0;
}

/* the timer has reached OCR1A, so the current PPM pulse has been completed */
ISR(TIMER1_COMPA_vect) {
	set_ppm(1);
}

/* finished sending the stop pulse */
ISR(TIMER1_COMPB_vect) {
	set_ppm(0);
	if (current_channel < N_CHANNELS) {
		/* get the pulse width for the current channel */
		uint16_t timeout = 2*(get_channel(current_channel)+1000);
		OCR1A = timeout;
		frame_time_remaining -= timeout;
		current_channel++;
	} else {
		/* we already transmitted the last channel, only wait for the frame to finish */
		OCR1A = frame_time_remaining;
		start_ppm_frame();
	}
}
