#define SRC_NULL 0
#define SRC_ADC 1
#define SRC_SW  2
#define SRC_DS  3
#define SRC_TWI_ADC  4
#define SRC_NUNCHUK 5

#ifndef TYPEDEF_ISRC_T
#define TYPEDEF_ISRC_T
typedef uint8_t isrc_t;
#endif

#define SRC_ID(s,n) ( (((s)&0x0F)<<4) | ((n)&0x0F) )

#define SRC_SYS(n)  ((n)>>4)
#define SRC_NUM(n) ((n)&0x0F)

int16_t get_input(isrc_t i);
int16_t get_input_range(isrc_t i, uint8_t max);
int16_t get_input_scaled(isrc_t src, int16_t min, int16_t max);
