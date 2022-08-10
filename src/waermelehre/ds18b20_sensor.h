#include <zephyr/types.h>

unsigned char ow_reset(void);
unsigned char read_bit(void);
void write_bit(char bitval);
unsigned char read_byte(void);
void write_byte(char val);
int read_temperature(void);

