#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include "config.h"
#include "twi.h"
#include "lcd.h"
#include "mag.h"
#include "input.h"
#include "lcd_status.h"
#include "config.h"

#ifdef USE_LCD

extern uint32_t millis;
extern uint8_t low_voltage;
extern isrc_t channel_source[];
extern uint8_t channel_count;

enum lcd_status_line {
#ifdef LCD_SHOW_DS_SWITCHES
	STATUS_LCD_SWITCHES,
#endif
#ifdef LCD_SHOW_MAG
	STATUS_LCD_MAG,
#endif
#ifdef LCD_SHOW_TIMER
	STATUS_LCD_TIMER,
#endif

	STATUS_LCD_MAX
};

static void lcd_status_channels(uint8_t row) {
#ifdef LCD_SHOW_CHANNELS
	lcd_set_cursor(row, 0);
#ifndef LCD_CHANNEL_INPUTS
	isrc_t *ch_inp = channel_source;
#define LCD_CHANNEL_COUNT (channel_count)
#else
	isrc_t ch[] = LCD_CHANNEL_INPUTS;
	isrc_t *ch_inp = ch;
#define LCD_CHANNEL_COUNT ( sizeof(ch)/sizeof(*ch) )
#endif
	for (uint8_t i=0; i<LCD_CHANNEL_COUNT && i<8; i++) {
		isrc_t v = get_input_scaled(ch_inp[i], 0, 6);
		lcd_write(lcd_get_bargraph(v));
	}
#elif defined(LCD_SHOW_CROSSHAIRS)
#ifndef LCD_CROSSHAIR_INPUTS
#define LCD_CROSSHAIR_INPUTS { {channel_source[3], channel_source[2]}, {channel_source[0], channel_source[1]} }
#endif
	isrc_t ch_input[2][2] = LCD_CROSSHAIR_INPUTS;

	lcd_create_crosshair( ch_input[0][0], ch_input[0][1], 6);
	lcd_create_crosshair( ch_input[1][0], ch_input[1][1], 7);
#ifdef LCD_CROSSHAIR_BAR_LEFT
	lcd_create_bargraph(LCD_CROSSHAIR_BAR_LEFT, 4);
#endif
#ifdef LCD_CROSSHAIR_BAR_RIGHT
	lcd_create_bargraph(LCD_CROSSHAIR_BAR_RIGHT, 5);
#endif
	lcd_set_cursor(row,0);
	lcd_write(6);
#ifdef LCD_CROSSHAIR_BAR_LEFT
	lcd_write(4);
#endif
#ifdef LCD_CROSSHAIR_BAR_RIGHT
	lcd_set_cursor(row,6);
	lcd_write(5);
#endif
	lcd_set_cursor(row,7);
	lcd_write(7);
#endif
}

static void lcd_status_draw(uint8_t row, enum lcd_status_line what) {
	lcd_set_cursor(row, 0);
	switch (what) {
#ifdef LCD_SHOW_DS_SWITCHES
		case STATUS_LCD_SWITCHES:
			{
				uint8_t sw[] = DS_SEND_AUX_SWITCHES;
				for (uint8_t i=0; i<sizeof(sw)/sizeof(sw[0]) && i<8; i++) {
					uint8_t n=sw[i];
					if (n==0) {
						lcd_write('_');
					} else {
						switch (get_input_scaled(n, -1, 1)) {
							case -1:
								lcd_write('V');
								break;
							case  0:
								lcd_write('-');
								break;
							case  1:
								lcd_write(LCD_CHAR_CHEVRON_UP);
								break;
							default:
								lcd_write(' ');
						}
					}
				}
			}
			break;
#endif
#ifdef LCD_SHOW_MAG
		case STATUS_LCD_MAG:
			lcd_write(LCD_CHAR_ARROW_RIGHT);
			int16_t h = mag_heading();
			if (h < 0) {
				lcd_write_str("MAG ERR");
			} else {
				lcd_fwrite("%3d", mag_heading()/10);
				lcd_write(LCD_CHAR_DEGREES);
				lcd_write_str("   ");
			}
			break;
#endif
#ifdef LCD_SHOW_TIMER
		case STATUS_LCD_TIMER:
			{
				uint8_t minutes = millis/1000/60;
				uint8_t seconds = (millis/1000)%60;
				lcd_write('\0');
				lcd_fwrite("%2u:%02u  ", minutes, seconds);;
			}
			break;
#endif
		default:
			break;
	}
}

static void lcd_status_battery(uint8_t row) {
	lcd_set_cursor(row, 7);
	if (low_voltage) {
		lcd_write((millis/1000 % 2) ? LCD_CHAR_OMEGA : '!');
	}
}

void lcd_status_update(uint8_t reset) {
#ifdef USE_LCD
#define LCD_AUTO_SWITCH_INTERVAL 2000
#define LCD_MANUAL_SWITCH_INTERVAL (3*(LCD_AUTO_SWITCH_INTERVAL))
	static enum lcd_status_line status_lcd_state;
	static uint32_t next_update;
	if (!reset && millis < next_update) {
		return;
	}
	next_update = millis+100;

	static uint32_t next_switch;

#ifdef LCD_STATUS_SWITCH_INPUT
	static int8_t old_sw_state = 0;
	int8_t sw_state = get_input_scaled(LCD_STATUS_SWITCH_INPUT, -1, 1);
	if (sw_state != old_sw_state && sw_state > 0) {
		status_lcd_state++;
		next_switch = millis+2*(LCD_AUTO_SWITCH_INTERVAL);
	}
	if (sw_state != old_sw_state && sw_state < 0) {
		if (status_lcd_state == 0) {
			status_lcd_state = STATUS_LCD_MAX-1;
		} else {
			status_lcd_state--;
		}
		next_switch = millis+(LCD_MANUAL_SWITCH_INTERVAL);
	}
	old_sw_state = sw_state;
#endif

	if (millis > next_switch) {
		status_lcd_state++;
		next_switch = millis+(LCD_AUTO_SWITCH_INTERVAL);
	}
	if (status_lcd_state >= STATUS_LCD_MAX) status_lcd_state = 0;

	/* the first line is static */
	lcd_status_channels(0);
	/* the second line iterates through the various displays */
	lcd_status_draw(1, status_lcd_state);
#ifdef LCD_SHOW_BATTERY_WARNING
	/* are we running low on juice? */
	lcd_status_battery(1);
#endif
#endif
}

#endif
