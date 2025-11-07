#ifndef LCD_DRIVER_H
#define LCD_DRIVER_H

#include <stdint.h>
#include <stddef.h>

int lcd_init(void);
int lcd_write_data(uint8_t addr, const uint8_t *data, size_t len);
int lcd_clear(void);
int lcd_test_all_segments(void);
void lcd_test_walking_segments(void);

#endif
