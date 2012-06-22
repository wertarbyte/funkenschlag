#define SRC_NULL 0
#define SRC_ADC 1
#define SRC_SW  2
#define SRC_DS  3
#define SRC_TWI_ADC  4
#define SRC_NUNCHUK 5
#define SRC_ID(s,n) ( (((s)&0x0F)<<4) | ((n)&0x0F) )

#define SRC_SYS(n)  ((n)>>4)
#define SRC_NUM(n) ((n)&0x0F)

int16_t get_input(uint8_t i);
int16_t get_input_range(uint8_t i, uint8_t max);
int16_t get_input_scaled(uint8_t src, int16_t min, int16_t max);
