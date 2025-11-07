#ifndef LCD_SEGMENTS_H
#define LCD_SEGMENTS_H

#include <stdint.h>
#include <stdbool.h>

/*
 * PCF8576DT LCD Segment Mapping (1/4 duty)
 * -----------------------------------------
 * Each LCD segment = one S-line (0–39) × COM index (0–3)
 * Safe for all Zephyr + picolibc toolchains.
 */

typedef struct {
    uint8_t s;   /* Segment index (S0–S39) */
    uint8_t com; /* COM index (0–3) */
} seg_t;

/* Macro replaced with inline initializer for GCC safety */
static inline seg_t make_seg(uint8_t s, uint8_t com)
{
    seg_t seg = { s, com };
    return seg;
}

/* Digit map lookup (6 digits × 8 segments [A–G + DP]) */
extern const seg_t *digit_map[6];

/* Display control */
void lcd_show_digit(uint8_t pos, uint8_t value, uint8_t *ram);
void lcd_show_number(uint32_t value, uint8_t *ram);

/* Diagnostic helper */
void lcd_test_digit_segments(uint8_t pos, uint8_t *ram);

#endif /* LCD_SEGMENTS_H */
