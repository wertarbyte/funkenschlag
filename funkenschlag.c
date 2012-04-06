#include <stdlib.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define N_CHANNELS 6
#define ADC_CHANNELS 4

#define PPM_DDR  DDRB
#define PPM_PORT PORTB
#define PPM_PIN  PINB
#define PPM_BIT  PB0

#define FRAME_MS 20000L
#define STOP_MS  300


static uint16_t adc_values[ADC_CHANNELS] = {0};

static uint8_t current_channel;
static uint16_t frame_time_remaining;

static void toggle_ppm(void) {
	PPM_PIN |= (1<<PPM_BIT);
}

static void start_ppm_frame(void) {
	frame_time_remaining = FRAME_MS*2;
	current_channel = 0;
}

static uint16_t get_channel(uint8_t i) {
	if (i < ADC_CHANNELS) {
		return adc_values[i];
	} else {
		return 0;
	}
}

int main(void) {
	/* configure PPM output port */
	PPM_DDR |= (1<<PPM_BIT);

	/* configure ADC */
	// TODO

	/* configure timer */

	/* enable CTC waveform generation (TOP == OCR1A) */
	TCCR1B |= (1<<WGM12);
	/* enable compare interrupts */
	TIMSK = (1<<OCIE1B | 1<<OCIE1A);
	/* set compare value for the stop pulese to 300µs */
	OCR1B = STOP_MS*2;
	/* set pulse width to max for now */
	OCR1A = ~0;
	/* set Timer 1 to clk/8, giving us ticks of 1/2 µs */
	TCCR1B |= (1<<CS11);

	/* initialize channel data */
	start_ppm_frame();

	/* enable interrupts */
	sei();

	while (1) {
		/* keep sampling adc data */
		// TODO
	}
	return 0;
}

/* the timer has reached OCR1A, so the current PPM pulse has been completed */
ISR(TIMER1_COMPA_vect) {
	toggle_ppm();
}

/* finished sending the stop pulse */
ISR(TIMER1_COMPB_vect) {
	toggle_ppm();
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
