#include <stdlib.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include "src_sw.h"

/* switch multiplexing */
#define MPX_DDR  DDRB
#define MPX_PORT PORTB
#define MPX_PIN  PINB
#define MPX_BIT  PB2


/* we are using switches with 3 positions (neutral, up, down),
 * each switch using a single input pin through multiplexing
 */
static struct {
	volatile uint8_t *pin;
	volatile uint8_t *port;
	uint8_t bit;
} sw_inputs[] = {
	{ &PIND, &PORTD, PD7 },
	{ &PIND, &PORTD, PD6 },
	{ &PINB, &PORTB, PB6 },
	{ &PINB, &PORTB, PB7 },
};

#define N_3W_SWITCHES ( sizeof(sw_inputs)/sizeof(*sw_inputs) )

static int8_t sw_positions[N_3W_SWITCHES] = {0};

int16_t sw_get(uint8_t n) {
	return (1023/2)+(1023/2)*sw_positions[n];
}

int8_t sw_get_raw(uint8_t n) {
	if (n >= N_3W_SWITCHES) return 0;
	return sw_positions[n];
}

void sw_query(void) {
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
}

void sw_init(void) {
	MPX_DDR |= 1<<MPX_BIT;

	/* enable pull-up resistors for switches */
	for (uint8_t sw = 0; sw < N_3W_SWITCHES; sw++) {
		*sw_inputs[sw].port |= 1<<sw_inputs[sw].bit;
	}
}
