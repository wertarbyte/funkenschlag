#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <math.h>
#include "config.h"
#include "twi.h"
#include "lcd.h"
#include "mag.h"
#include "input.h"
#include "serial.h"
#include "lcd_menu.h"
#include "config.h"

#ifdef LCD_MENU

extern volatile uint32_t millis;

#define LCD_MAG_CALIBRATION_SEC 10

static enum lcd_menu_t {
	LCD_MENU_START,
	LCD_MENU_MAG_CALIB,
	LCD_MENU_MAG_CALIB_PROGRESS,
	LCD_MENU_CNT
} lcd_menu_state = LCD_MENU_START;

static void lcd_menu_switch(enum lcd_menu_t s) {
	lcd_clear();
	switch (s) {
		case LCD_MENU_START:
			lcd_write_str("Funken-");
			lcd_set_cursor(1,0);
			lcd_write_str("schlag!");
			break;
		case LCD_MENU_MAG_CALIB:
			lcd_write_str("Calib");
			lcd_set_cursor(1,0);
			lcd_write_str("mag?");
			break;
		case LCD_MENU_MAG_CALIB_PROGRESS:
			lcd_write_str("Spin me!");
			lcd_set_cursor(1,0);
			int8_t sec_left = (8L*mag_is_calibrating()/LCD_MAG_CALIBRATION_SEC)/1000L;
			int8_t p = 8-sec_left;
			for (uint8_t i=0; i<8; i++) {
				if (i == p) {
					lcd_write(LCD_CHAR_ARROW_RIGHT);
				} else if (i < p) {
					lcd_write('-');
				}
			}
			break;
		default:
			return;
	}
	lcd_menu_state = s;
}

static void lcd_menu_input(int8_t ud, int8_t lr) {
	switch (lcd_menu_state) {
		case LCD_MENU_START:
			switch(ud) {
				case -1:
					lcd_menu_switch(LCD_MENU_MAG_CALIB);
			}
			break;
		case LCD_MENU_MAG_CALIB:
			switch(lr) {
				case 1:
					mag_set_calibration( millis + LCD_MAG_CALIBRATION_SEC*1000L );
					lcd_menu_switch(LCD_MENU_MAG_CALIB_PROGRESS);
					break;
			}

			switch(ud) {
				case 1:
					lcd_menu_switch(LCD_MENU_START);
					break;
			}
			break;
		default:
			break;
	}
}

static uint8_t input_deadzone(int8_t i, uint8_t deadzone) {
	if (abs(i) <= deadzone) {
		return 0;
	} else {
		return i<0 ? -1 : 1;
	}
}

void lcd_menu_update(uint8_t reset) {
	// process input data
	#define INPUT_RANGE 5
	int8_t updown = input_deadzone( get_input_scaled( LCD_MENU_UPDOWN_INPUT, -INPUT_RANGE, INPUT_RANGE ), INPUT_RANGE/2 );
	int8_t leftright = input_deadzone( get_input_scaled( LCD_MENU_LEFTRIGHT_INPUT, -INPUT_RANGE, INPUT_RANGE ), INPUT_RANGE/2);
	if (reset || lcd_refresh_timeout()) lcd_menu_switch(lcd_menu_state);
	static uint8_t neutralized = 0;
	if (updown == 0 && leftright == 0) {
		neutralized = 1;
	} else if (neutralized) {
		neutralized = 0;
		lcd_menu_input(updown, leftright);
	}

	if ((lcd_menu_state == LCD_MENU_MAG_CALIB_PROGRESS) && !mag_is_calibrating()) {
		lcd_menu_switch(LCD_MENU_START);
	}
}

#endif
