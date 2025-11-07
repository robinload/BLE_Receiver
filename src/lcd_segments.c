#include "lcd_segments.h"
#include <string.h>
#include <zephyr/kernel.h>  // for k_msleep()

/*
 * LCD Segment Mapping for PCF8576DT (1/4 duty)
 * COM naming avoids conflicts with Zephyr UART macros.
 */

#define COM_IDX1 0
#define COM_IDX2 1
#define COM_IDX3 2
#define COM_IDX4 3

/* ==============================================================
 * Digit segment mapping
 * ============================================================== */

static const seg_t digit0_map[8] = {
    {15, COM_IDX1}, // A
    {16, COM_IDX1}, // B
    {16, COM_IDX3}, // C
    {15, COM_IDX4}, // D
    {16, COM_IDX4}, // E
    {16, COM_IDX2}, // F
    {17, COM_IDX3}, // G
    {17, COM_IDX4}  // DP
};

static const seg_t digit1_map[8] = {
    {0, COM_IDX1}, {0, COM_IDX1}, {0, COM_IDX1},
    {0, COM_IDX1}, {0, COM_IDX1}, {0, COM_IDX1},
    {0, COM_IDX1}, {0, COM_IDX1}
};

static const seg_t digit2_map[8] = {
    {0, COM_IDX1}, {0, COM_IDX1}, {0, COM_IDX1},
    {0, COM_IDX1}, {0, COM_IDX1}, {0, COM_IDX1},
    {0, COM_IDX1}, {0, COM_IDX1}
};

static const seg_t digit3_map[8] = {
    {0, COM_IDX1}, {0, COM_IDX1}, {0, COM_IDX1},
    {0, COM_IDX1}, {0, COM_IDX1}, {0, COM_IDX1},
    {0, COM_IDX1}, {0, COM_IDX1}
};

static const seg_t digit4_map[8] = {
    {0, COM_IDX1}, {0, COM_IDX1}, {0, COM_IDX1},
    {0, COM_IDX1}, {0, COM_IDX1}, {0, COM_IDX1},
    {0, COM_IDX1}, {0, COM_IDX1}
};

static const seg_t digit5_map[8] = {
    {0, COM_IDX1}, {0, COM_IDX1}, {0, COM_IDX1},
    {0, COM_IDX1}, {0, COM_IDX1}, {0, COM_IDX1},
    {0, COM_IDX1}, {0, COM_IDX1}
};

/* Lookup table */
const seg_t *digit_map[6] = {
    digit0_map, digit1_map, digit2_map,
    digit3_map, digit4_map, digit5_map
};

/* ==============================================================
 * Segment patterns for numbers 0–9
 * ============================================================== */
static const uint8_t segmask[10] = {
    0b0111111, // 0
    0b0000110, // 1
    0b1011011, // 2
    0b1001111, // 3
    0b1100110, // 4
    0b1101101, // 5
    0b1111101, // 6
    0b0000111, // 7
    0b1111111, // 8
    0b1101111  // 9
};

/* ==============================================================
 * Internal helpers
 * ============================================================== */
static inline void set_seg(uint8_t *ram, seg_t seg, bool on)
{
    if (on)
        ram[seg.s] |= (1 << seg.com);
    else
        ram[seg.s] &= ~(1 << seg.com);
}

/* ==============================================================
 * Draw one digit (0–9)
 * ============================================================== */
void lcd_show_digit(uint8_t pos, uint8_t value, uint8_t *ram)
{
    if (pos > 5 || value > 9)
        return;

    const seg_t *map = digit_map[pos];

    for (int s = 0; s < 7; s++) {
        bool on = segmask[value] & (1 << s);
        set_seg(ram, map[s], on);
    }
}

/* ==============================================================
 * Draw full number (right aligned)
 * ============================================================== */
void lcd_show_number(uint32_t value, uint8_t *ram)
{
    for (int pos = 0; pos < 6; pos++) {
        uint8_t digit = value % 10;
        value /= 10;
        lcd_show_digit(pos, digit, ram);
    }
}

/* ==============================================================
 * Diagnostic: cycle through A–G + DP for one digit
 * ============================================================== */
void lcd_test_digit_segments(uint8_t pos, uint8_t *ram)
{
    if (pos > 5)
        return;

    const seg_t *map = digit_map[pos];

    printk("Testing digit %d...\n", pos);

    for (int s = 0; s < 8; s++) {
        memset(ram, 0x00, 40);
        set_seg(ram, map[s], true);

        /* Send buffer to PCF8576 */
        int ret = lcd_write_data(0x00, ram, 40);
        if (ret < 0) {
            printk("Segment %d write failed: %d\n", s, ret);
        } else {
            printk("Segment %d ON (S=%d, COM=%d)\n", s, map[s].s, map[s].com);
        }

        k_msleep(400);
    }

    /* Clear display after test */
    memset(ram, 0x00, 40);
    lcd_write_data(0x00, ram, 40);
}
