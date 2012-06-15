MCU = atmega8
F_CPU = 8000000
TARGET = funkenschlag
SRC = funkenschlag.c src_adc.c src_sw.c src_ds.c datenschlag.c
COMBINE_SRC = 0

include avr-tmpl.mk
