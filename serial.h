void serial_init(void);

void serial_write(char c);
void serial_fwrite(const char *fmt, ...);
void serial_write_str(const char *c);
void serial_write_int(int16_t t);
void serial_write_uint(uint16_t t);
