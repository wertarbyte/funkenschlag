MCU = atmega168
F_CPU = 8000000
TARGET = funkenschlag
SRC = funkenschlag.c input.c channels.c serial.c twi.c src_adc.c src_sw.c src_ds.c src_twi_adc.c mag.c acc.c src_nunchuk.c datenschlag.c lcd.c lcd_status.c lcd_menu.c
COMBINE_SRC = 0

include avr-tmpl.mk
