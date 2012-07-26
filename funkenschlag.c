#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include "serial.h"
#include "twi.h"
#include "src_adc.h"
#include "src_sw.h"
#include "src_ds.h"
#include "src_twi_adc.h"
#include "src_nunchuk.h"
#include "datenschlag.h"
#include "input.h"
#include "datenschlag_structs.h"
#include "mag.h"
#include "acc.h"
#include "lcd.h"
#include "lcd_menu.h"
#include "lcd_status.h"

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

#define FRAME_US 20000L
#define STOP_US 500

#include "config.h"

extern isrc_t channel_source[];
extern uint8_t channel_count;

static uint8_t current_channel;
static uint16_t frame_time_remaining = 0;
#define MAX_CHANNELS 9
static uint16_t frame_times[MAX_CHANNELS] = {0};

/* milliseconds since startup */
volatile uint32_t millis = 0;

uint8_t low_voltage = 0;

static void set_ppm(uint8_t h) {
	if (h) {
		PPM_PORT |= 1<<PPM_BIT;
	} else {
		PPM_PORT &= ~(1<<PPM_BIT);
	}
}

static int16_t get_channel(uint8_t i) {
	int16_t val = 0;
	uint8_t src = channel_source[i];
	switch (SRC_SYS(src)) {
		case SRC_ADC:
			val = adc_get(SRC_NUM(src));
			break;
		case SRC_SW:
			val = sw_get(SRC_NUM(src));
			break;
		case SRC_DS:
			val = ds_get_next_pulse();
			break;
#ifdef USE_TWI_ADC
		case SRC_TWI_ADC:
			val = twi_adc_get(SRC_NUM(src));
			break;
#endif
#ifdef USE_NUNCHUK
		case SRC_NUNCHUK:
			val = nunchuk_get(SRC_NUM(src));
			break;
#endif
		default: /* unknown source */
			break;
	}
	return val;
}

static void start_ppm_frame(void) {
	current_channel = 0;
	frame_time_remaining = FRAME_US;
	for (uint8_t i=0; i<channel_count; i++) {
		frame_times[i] = (1000+get_channel(i));
		frame_time_remaining -= frame_times[i];
	}
}

static void start_ppm_pulse(void) {
	if (current_channel < channel_count) {
		/* get the pulse width for the current channel */
		OCR1A = frame_times[current_channel];
		current_channel++;
	} else {
		/* we already transmitted the last channel, only wait for the frame to finish */
		OCR1A = frame_time_remaining;
		start_ppm_frame();
	}
}

int main(void) {
	wdt_enable(WDTO_4S);
	wdt_reset();
	/* configure PPM output port */
	PPM_DDR |= (1<<PPM_BIT);
	PPM_PORT &= ~(1<<PPM_BIT);

	/* configure LED output port */
	LED_DDR |= (1<<LED_BIT);
	LED_PORT |= (1<<LED_BIT);

	/* configure VOL(tage) warning port */
	VOL_DDR &= ~(1<<VOL_BIT);
	VOL_PORT |= (1<<VOL_BIT); // enable pullup

#ifdef ENABLE_SERIAL
	serial_init();
#endif
#ifdef ENABLE_TWI
	twi_init();
#endif
#ifdef USE_NUNCHUK
	nunchuk_init();
	wdt_reset();
#endif

	/* configure switches */
	sw_init();

	/* configure ADC */
	adc_init();

#if defined(USE_TWI_ADC)
	twi_adc_init();
	wdt_reset();
#endif
#if defined(USE_MAG)
	mag_init();
	wdt_reset();
#endif
#if defined(USE_ACC)
	acc_init();
#endif
#if defined(USE_LCD)
	/* initialize LCD twice (due to timing issues?) */
	lcd_init();
	wdt_reset();
	_delay_ms(100);
	wdt_reset();
	lcd_init();
	wdt_reset();
	lcd_splash();
#endif

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

	serial_write_str("Welcome!\n");
	while (1) {
		/* reset watchdog */
		wdt_reset();

		/* keep sampling adc data */
		adc_query();

		/* query switches */
		sw_query();

		/* prepare Datenschlag data frames */
		ds_prepare();

#ifdef USE_TWI_ADC
		/* query TWI/I²C ADC */
		twi_adc_query();
#endif
#if defined(USE_MAG)
#ifdef MAG_CENTER_CALIBRATION_TRIGGER_INPUT
		if (get_input_scaled( (MAG_CENTER_CALIBRATION_TRIGGER_INPUT), 1, -1) == (MAG_CENTER_CALIBRATION_TRIGGER_VALUE)) {
			mag_set_calibration(millis + 10000L);
		}
#endif
		if (mag_is_calibrating()) {
			mag_calibrate(0);
		} else {
			mag_calibrate(1);
			mag_query();
			mag_dump();
		}
#endif
#if defined(USE_ACC)
		acc_query();
		acc_dump();
#endif

#ifdef USE_NUNCHUK
		nunchuk_query();
#endif

		/* check voltage */
		low_voltage = !((~VOL_PIN) & 1<<VOL_BIT);

		/* switch LED */
		if (!low_voltage || (millis/250 % 2)) {
			LED_PORT |= (1<<LED_BIT);
		} else {
			LED_PORT &= ~(1<<LED_BIT);
		}

#ifdef USE_LCD
		static enum {
			LCD_MODE_STATUS,
#ifdef LCD_MENU
			LCD_MODE_MENU,
#endif
			LCD_MODE_CNT
		} lcd_mode;

		uint8_t lcd_mode_changed = 0;
#ifdef LCD_MODE_SWITCH_INPUT
		static int8_t old_sw_state = 0;
		int8_t sw_state = get_input_scaled(LCD_MODE_SWITCH_INPUT, -1, 1);
		if (old_sw_state != sw_state) {
			lcd_mode += LCD_MODE_CNT;
			lcd_mode += sw_state;
			lcd_mode %= LCD_MODE_CNT;
			old_sw_state = sw_state;
			lcd_mode_changed = 1;
		}
#endif
		switch (lcd_mode) {
			case LCD_MODE_STATUS:
				lcd_status_update(lcd_mode_changed);
				break;
#ifdef LCD_MENU
			case LCD_MODE_MENU:
				lcd_menu_update(lcd_mode_changed);
				break;
#endif
			default:
				break;
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
