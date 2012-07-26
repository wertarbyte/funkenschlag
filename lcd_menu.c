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
static enum lcd_menu_t {
	LCD_MENU_START,
	LCD_MENU_MAG_CALIB,
	LCD_MENU_CNT
} lcd_menu_state = LCD_MENU_START;

static void lcd_menu_switch(enum lcd_menu_t s) {
	lcd_clear();
	lcd_set_cursor(0,0);
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
		default:
			lcd_write_str("UNKNOWN");
			lcd_set_cursor(1,0);
			lcd_write_str("MENU!");
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
					//mag_start_calibration();
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
	if (reset) lcd_menu_switch(lcd_menu_state);
	static uint8_t neutralized = 0;
	if (updown == 0 && leftright == 0) {
		neutralized = 1;
	} else if (neutralized) {
		neutralized = 0;
		lcd_menu_input(updown, leftright);
	}
}

#endif
