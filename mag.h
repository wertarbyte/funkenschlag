void mag_init(void);
void mag_query(void);
void mag_dump(void);

int16_t mag_heading(void);
void mag_calibrate(uint8_t complete);
uint32_t mag_is_calibrating(void);
void mag_set_calibration(uint32_t timeout);
