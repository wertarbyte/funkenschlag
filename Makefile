MCU = atmega8
F_CPU = 8000000
TARGET = funkenschlag
SRC = funkenschlag.c input.c serial.c twi.c src_adc.c src_sw.c src_ds.c src_twi_adc.c datenschlag.c
COMBINE_SRC = 0

include avr-tmpl.mk
