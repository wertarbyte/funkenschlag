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
#include "input.h"

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

static void lcd_create_char(uint8_t slot, uint8_t map[]) {
	slot &= 0x7;
	lcd_cmd(LCD_SETCGRAMADDR | (slot << 3));
	for (uint8_t i=0; i<8; i++) {
		lcd_write(map[i]);
	}
}

#ifdef LCD_SHOW_CHANNELS
static void lcd_load_bargraph(void) {
	uint8_t graph[8];
	memset(graph, 0, 8);
	/* we only generate bars for 2-6
	 * since 0 and 1 are free lunch
	 * (line 7 is the cursor)
	 */
	graph[6] = 0xFF;
	for (uint8_t i=2; i<7; i++) {
		graph[7-i] = 0xFF;
		lcd_create_char(i, graph);
	}
}

char lcd_get_bargraph(uint8_t i) {
	switch (i) {
		case 0:
			return ' ';
		case 1:
			return '_';
	}
	return i & 0x7;
}
#endif

void lcd_load_icons(void) {
	uint8_t clock[8] = {0x0,0xe,0x15,0x17,0x11,0xe,0x0};
	lcd_create_char(0, clock);
}

#ifdef LCD_SHOW_CROSSHAIRS
void lcd_create_crosshair(isrc_t xd, isrc_t yd, uint8_t slot) {
	uint8_t x = get_input_scaled(xd, 4, 0);
	uint8_t y = get_input_scaled(yd, 7, 0);
	uint8_t g[8];
	memset(g, (1<<(x)), sizeof(g));
	g[y] ^= 0xFF;
	lcd_create_char(slot, g);
}

/* create bargraph on the fly */
void lcd_create_bargraph(isrc_t bd, uint8_t slot) {
	uint8_t g[8];
	uint8_t y = get_input_scaled(bd, 0, sizeof(g)-1);
	memset(g, 0xFF, sizeof(g));
	memset(g, 0x00, 7-y);
	lcd_create_char(slot, g);
}
#endif

void lcd_init(void) {
	uint16_t contrast = 0x0A;

	lcd_cmd(LCD_FUNCTIONSET | _lcd_basic);
	lcd_cmd(LCD_FUNCTIONSET | _lcd_extended);

	lcd_cmd(LCD_BIAS_OSC_CONTROL | LCD_BIAS1_4 | LCD_OSC_347);
	lcd_cmd(LCD_CONTRAST_LOW_BYTE | (contrast & LCD_CONTRAST_LOW_BYTE_MASK));
	lcd_cmd(LCD_ICON_CONTRAST_HIGH_BYTE | LCD_ICON_ON | LCD_BOOSTER_ON | (contrast >> 4 & LCD_CONTRAST_HIGH_BYTE_MASK));
	lcd_cmd(LCD_FOLLOWER_CONTROL | LCD_FOLLOWER_ON | LCD_Rab_2_00);
	_delay_ms(20);
	lcd_cmd(LCD_FUNCTIONSET | _lcd_basic);
	lcd_cmd(LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF);
	lcd_cmd(LCD_ENTRYMODESET | LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT);

	lcd_clear();
	_delay_ms(2);
	lcd_cmd(LCD_RETURNHOME);

#ifdef LCD_SHOW_CHANNELS
	lcd_load_bargraph();
#endif
	lcd_load_icons();
}

static void lcd_char_demo(void) {
	for (uint8_t m=0; m<32; m++) {
		lcd_clear();
		_delay_ms(500);
		lcd_set_cursor(0,0);
		lcd_write('H');
		lcd_write_uint(m);
		lcd_set_cursor(1,0);
		for (int j=0; j<8; j++) {
			lcd_write(m*8+j);
		}
		_delay_ms(2000);
	}
}

void lcd_clear(void) {
	lcd_cmd(LCD_CLEARDISPLAY);
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
	lcd_fwrite("%i", i);
}

void lcd_write_uint(uint16_t i) {
	lcd_fwrite("%u", i);
}

void lcd_fwrite(const char *format_string, ...) {
	char str[10];
	va_list args;
	va_start(args, format_string);
	vsnprintf(str, sizeof(str), format_string, args);
	va_end(args);
	lcd_write_str(str);
}

void lcd_splash(void) {
#ifdef LCD_SPLASH_SCREEN
	lcd_set_cursor(0,0);
	lcd_write_str("FUNKEN-");
	lcd_set_cursor(1,0);
	lcd_write_str("SCHLAG!");
	_delay_ms(1000);
	lcd_clear();
#endif
}
#endif
