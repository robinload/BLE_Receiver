#ifndef LCD_DRIVER_H
#define LCD_DRIVER_H

#include <stdint.h>
#include <stddef.h>

int lcd_init(void);
int lcd_write_data(uint8_t addr, const uint8_t *data, size_t len);
int lcd_clear(void);
int lcd_test_all_segments(void);
void lcd_test_walking_active_segments(void);
void lcd_test_mapping (void);
void lcd_test_walk_by_S(void);
int lcd_test_active_segments(void);
int showUnitNewton(void);
int showUnitGram(void);
int showUnitPound(void);
int showUnitTon(void);
int showUnitKPound(void);
int showUnitKNewton(void);
int showUnitKGram(void);
int showDecimalPoints(int pos);
int showCharacterByPosition(int pos, uint8_t value);
int showCharacterByPositionChar(int pos, char ch);
int lcd_show_string(const char *str);

/** Show a number (decimal or hexadecimal) on the display. */
int lcd_show_number(double value, int decimal_pos);
#endif
