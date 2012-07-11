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

#ifdef USE_LCD

#define LCD_ADDR 0x3E

#define LCD_CONF (LCD_8BITMODE | LCD_1LINE | LCD_2LINE | LCD_5x8DOTS)
static uint8_t _lcd_basic    = LCD_INSTRUCTION_SET_BASIC    | LCD_CONF;
static uint8_t _lcd_extended = LCD_INSTRUCTION_SET_EXTENDED | LCD_CONF;

static void lcd_send(uint8_t value, uint8_t mode) {
	twi_write_reg(LCD_ADDR, mode, value);
	_delay_us(30);
}

static void lcd_cmd(uint8_t cmd) {
	lcd_send(cmd, 0x00);
}

static void lcd_data(uint8_t data) {
	lcd_send(data, 0x40);
}

void lcd_init(void) {
	uint16_t contrast = 0x0A;

	lcd_cmd(LCD_FUNCTIONSET | _lcd_basic);
	lcd_cmd(LCD_FUNCTIONSET | _lcd_extended);

	lcd_cmd(LCD_BIAS_OSC_CONTROL | LCD_BIAS1_5 | LCD_OSC_192);
	lcd_cmd(LCD_CONTRAST_LOW_BYTE | (contrast & LCD_CONTRAST_LOW_BYTE_MASK));
	lcd_cmd(LCD_ICON_CONTRAST_HIGH_BYTE | LCD_ICON_ON | LCD_BOOSTER_ON | (contrast >> 4 & LCD_CONTRAST_HIGH_BYTE_MASK));
	lcd_cmd(LCD_FOLLOWER_CONTROL | LCD_FOLLOWER_ON | LCD_Rab_2_00);
	_delay_ms(20);
	lcd_cmd(LCD_FUNCTIONSET | _lcd_basic);
	lcd_cmd(LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF);
	lcd_cmd(LCD_ENTRYMODESET | LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT);

	lcd_cmd(LCD_CLEARDISPLAY);
	_delay_ms(2);
	lcd_cmd(LCD_RETURNHOME);

	lcd_set_cursor(0,0);
	lcd_write_str("FUNKEN-");
	lcd_set_cursor(1,0);
	lcd_write_str("SCHLAG!");
}

void lcd_set_cursor(uint8_t l, uint8_t c) {
	static uint8_t offsets[] = {0x00, 0x40, 0x14, 0x54};
	lcd_cmd(LCD_SETDDRAMADDR | (c+offsets[l]));
}

void lcd_write(char c) {
	lcd_data(c);
}

void lcd_write_str(const char *s) {
	while (*s) {
		lcd_write(*s++);
	}
}

void lcd_write_int(int16_t i) {
	char str[10];
	snprintf(str, 10, "%i", i);
	lcd_write_str(str);
}

void lcd_write_uint(uint16_t i) {
	char str[10];
	snprintf(str, 10, "%u", i);
	lcd_write_str(str);
}
#endif
