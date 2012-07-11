#define LCD_CLEARDISPLAY			0x01		// Command (HD44780)			Page 21
#define LCD_RETURNHOME				0x02		// Command (HD44780)

// flags for display entry mode
#define LCD_ENTRYMODESET			0x04		// Command (HD44780)
#define LCD_ENTRYSHIFTINCREMENT			0x01		// Setting (HD44780)*
#define LCD_ENTRYSHIFTDECREMENT			0x00		// Setting (HD44780)
#define LCD_ENTRYLEFT 				0x02		// Setting (HD44780)*
#define LCD_ENTRYRIGHT				0x00		// Setting (HD44780)

// flags for display on/off control
#define LCD_DISPLAYCONTROL			0x08		// Command (HD44780)			Page 22
#define LCD_DISPLAYON 				0x04		// Setting (HD44780)*
#define LCD_DISPLAYOFF 				0x00		// Setting (HD44780)
#define LCD_CURSORON 				0x02		// Setting (HD44780)
#define LCD_CURSOROFF 				0x00		// Setting (HD44780)*
#define LCD_BLINKON 				0x01		// Setting (HD44780)
#define LCD_BLINKOFF 				0x00		// Setting (HD44780)*

// flags for display/cursor shift
#define LCD_CURSORSHIFT				0x10		// Command (HD44780)

#define LCD_DISPLAYMOVE				0x08		// Setting (HD44780)			
#define LCD_CURSORMOVE				0x00		// Setting (HD44780)			
#define LCD_MOVERIGHT				0x04		// Setting (HD44780)			
#define LCD_MOVELEFT				0x00		// Setting (HD44780)			

#define LCD_FUNCTIONSET 			0x20		// Command (HD44780)			Page 23
#define LCD_INSTRUCTION_SET_BASIC		0x00		// Setting (ST7032)			
#define LCD_INSTRUCTION_SET_EXTENDED		0x01		// Setting (ST7032)			
#define LCD_2LINE				0x08		// Setting (HD44780)*			
#define LCD_1LINE				0x00		// Setting (HD44780)			
#define LCD_8BITMODE				0x10		// Setting (HD44780)*
//#define LCD_4BITMODE				0x00		// Setting (HD44780)	Not used in I2C mode
//#define LCD_5x10DOTS 				0x04		// Setting (HD44780)	Not supported by ST7032
#define LCD_5x8DOTS				0x00		// Setting (HD44780)*

#define LCD_SETCGRAMADDR			0x40		// Command (HD44780)			Page 24
#define LCD_SETDDRAMADDR			0x80		// Command (HD44780)

#define LCD_BIAS_OSC_CONTROL			0x10		// Command (ST7032)			Page 26
#define LCD_BIAS1_4 				0x08		// Setting (ST7032)					 
#define LCD_BIAS1_5 				0x00		// Setting (ST7032)*
					
// Internal frequency adjust for VDD = 3.0 V
#define LCD_OSC_122				0x00		// Setting (ST7032)			
#define LCD_OSC_131				0x01		// Setting (ST7032)			
#define LCD_OSC_144				0x02		// Setting (ST7032)			
#define LCD_OSC_161				0x03		// Setting (ST7032)			
#define LCD_OSC_183				0x04		// Setting (ST7032)			
#define LCD_OSC_221				0x05		// Setting (ST7032)			
#define LCD_OSC_274				0x06		// Setting (ST7032)			
#define LCD_OSC_347				0x07		// Setting (ST7032)
			
// Internal frequency adjust for VDD = 5.0 V
#define LCD_OSC_120				0x00		// Setting (ST7032)			
#define LCD_OSC_133				0x01		// Setting (ST7032)			
#define LCD_OSC_149				0x02		// Setting (ST7032)			
#define LCD_OSC_167				0x03		// Setting (ST7032)			
#define LCD_OSC_192				0x04		// Setting (ST7032)*			
#define LCD_OSC_227				0x05		// Setting (ST7032)			
#define LCD_OSC_277				0x06		// Setting (ST7032)			
#define LCD_OSC_347				0x07		// Setting (ST7032)

#define ICON_RAMADDRESSSET			0x40		// Command (ST7032)		

#define LCD_ICON_CONTRAST_HIGH_BYTE		0x50		// Command (ST7032)
#define LCD_ICON_ON				0x08		// Setting (ST7032)
#define LCD_ICON_OFF				0x00		// Setting (ST7032)
#define LCD_BOOSTER_ON				0x04		// Setting (ST7032)*
#define LCD_BOOSTER_OFF				0x00		// Setting (ST7032)
#define LCD_CONTRAST_HIGH_BYTE_MASK		0x03		// Only used for bit masking (ST7032)

#define LCD_FOLLOWER_CONTROL			0x60		// Command (ST7032)			Page 27
#define LCD_FOLLOWER_ON				0x08		// Setting (ST7032)*			
#define LCD_FOLLOWER_OFF			0x00		// Setting (ST7032)			
#define LCD_Rab_1_00				0x00		// Setting (ST7032)			
#define LCD_Rab_1_25				0x01		// Setting (ST7032)			
#define LCD_Rab_1_50				0x02		// Setting (ST7032)			
#define LCD_Rab_1_80				0x03		// Setting (ST7032)			
#define LCD_Rab_2_00				0x04		// Setting (ST7032)*			
#define LCD_Rab_2_50				0x05		// Setting (ST7032)			
#define LCD_Rab_3_00				0x06		// Setting (ST7032)			
#define LCD_Rab_3_75				0x07		// Setting (ST7032)			

#define LCD_CONTRAST_LOW_BYTE			0x70		// Command (ST7032)
#define LCD_CONTRAST_LOW_BYTE_MASK		0x0F		// Only used for bit masking (ST7032)

void lcd_init(void);

void lcd_write(char c);
void lcd_write_str(const char *c);
void lcd_write_int(int16_t t);
void lcd_write_uint(uint16_t t);

void lcd_set_cursor(uint8_t c, uint8_t r);
