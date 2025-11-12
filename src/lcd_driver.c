#include "lcd_driver.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(lcd_driver, LOG_LEVEL_INF);

/* --- Hardware config --- */
#define LCD_I2C_NODE DT_ALIAS(i2c_lcd0)
#define LCD_ADDR 0x38

/* --- Command constants --- */
#define CMD_CONTINUE        0x80
#define CMD_LAST            0x00
#define CMD_MODE_SET        0x40
#define CMD_LOAD_DP         0x00
#define CMD_DEVICE_SELECT   0x60

#define MODE_ENABLE         (1 << 3)
#define MODE_BIAS_1_3       (0 << 2)
#define MODE_DRIVE_1_4      (0x00)
#define LCD_RAM_SIZE        20

/* --- Segment bit mapping --- */
#define SEG_A (1 << 3)   // from display_ram[18 - pos]
#define SEG_F (1 << 1)
#define SEG_E (1 << 2)
#define SEG_D (1 << 0)

#define SEG_B (1 << 7)   // from display_ram[19 - pos]
#define SEG_G (1 << 5)
#define SEG_C (1 << 6)

/* --- Internal state --- */
static const struct device *i2c_dev;
static uint8_t display_ram[LCD_RAM_SIZE];
static bool initialized = false;

/* --- I2C helper --- */
static int lcd_i2c_write(const uint8_t *buf, size_t len)
{
    int ret = i2c_write(i2c_dev, buf, len, LCD_ADDR);
    if (ret)
        LOG_ERR("I2C write (%d bytes) failed: %d", len, ret);
    return ret;
}

/* --- Initialization --- */
int lcd_init(void)
{
    int ret;
    uint8_t buf[3];

    i2c_dev = DEVICE_DT_GET(LCD_I2C_NODE);
    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }

    LOG_INF("Initializing PCF8576DT at 0x%02X", LCD_ADDR);
    k_msleep(100);

    uint8_t mode = MODE_ENABLE | MODE_BIAS_1_3 | MODE_DRIVE_1_4;
    buf[0] = CMD_CONTINUE | CMD_MODE_SET | mode;
    buf[1] = CMD_CONTINUE | CMD_LOAD_DP;
    buf[2] = CMD_LAST | CMD_DEVICE_SELECT | 0x00;

    ret = lcd_i2c_write(buf, 3);
    if (ret)
        return ret;

    memset(display_ram, 0, sizeof(display_ram));
    initialized = true;

    return lcd_clear();
}

/* --- Write RAM --- */
int lcd_write_data(uint8_t addr, const uint8_t *data, size_t len)
{
    if (!initialized)
        return -ENODEV;
    if (addr + len > sizeof(display_ram))
        return -EINVAL;

    uint8_t buf[22];
    buf[0] = CMD_CONTINUE | CMD_LOAD_DP | (addr & 0x3F);
    buf[1] = CMD_LAST | CMD_DEVICE_SELECT | 0x00;
    memcpy(&buf[2], data, len);

    return lcd_i2c_write(buf, len + 2);
}

/* --- Clear display --- */
int lcd_clear(void)
{
    memset(display_ram, 0, sizeof(display_ram));
    return lcd_write_data(0, display_ram, sizeof(display_ram));
}

/* --- Test patterns --- */
int lcd_test_all_segments(void)
{
    memset(display_ram, 0xFF, sizeof(display_ram));
    return lcd_write_data(0, display_ram, sizeof(display_ram));
}

void lcd_test_walking_segments(void)
{
    if (!initialized)
        return;

    memset(display_ram, 0x00, sizeof(display_ram));
    for (int i = 10; i < 20; i++)
        display_ram[i] = 0xFF;

    int ret = lcd_write_data(0, display_ram, sizeof(display_ram));
    if (ret == 0)
        LOG_INF("Active segments (S20–S39) ON");
    else
        LOG_ERR("Failed to update active segment range: %d", ret);
}

void lcd_test_walking_active_segments(void)
{
    if (!initialized)
        return;

    LOG_INF("Starting walking segment test (S20–S39)...");
    memset(display_ram, 0x00, sizeof(display_ram));

    for (int s = 10; s < 20; s++) {
        for (int com = 0; com < 4; com++) {
            memset(display_ram, 0x00, sizeof(display_ram));
            display_ram[s] = (1 << com);
            int ret = lcd_write_data(0, display_ram, sizeof(display_ram));
            if (ret < 0) {
                LOG_ERR("Write failed at S=%d COM=%d: %d", s, com, ret);
                return;
            }
            LOG_INF("Lighting S%d (COM%d)", s, com);
            k_msleep(300);
        }
    }
    LOG_INF("Walking segment test done.");
}

void lcd_test_mapping(void)
{
    if (!initialized)
        return;

    LOG_INF("Starting full mapping test (S0–S39, COM0–3)...");
    memset(display_ram, 0x00, sizeof(display_ram));

    for (int s = 0; s < 40; s++) {
        for (int com = 0; com < 4; com++) {
            memset(display_ram, 0x00, sizeof(display_ram));
            display_ram[s] = (1 << com);
            lcd_write_data(0, display_ram, sizeof(display_ram));
            LOG_INF("Lighting S%d COM%d", s, com);
            k_msleep(400);
        }
    }
    LOG_INF("Mapping test complete.");
}

void lcd_test_walk_by_S(void)
{
    if (!initialized)
        return;

    LOG_INF("Walking by S-line (lighting all COMs per S)...");
    memset(display_ram, 0x00, sizeof(display_ram));

    for (int s = 0; s < 40; s++) {
        memset(display_ram, 0x00, sizeof(display_ram));
        display_ram[s] = 0xFF;
        lcd_write_data(0, display_ram, sizeof(display_ram));
        LOG_INF("Lighting S%d (COM0–3)", s);
        k_msleep(400);
    }
    LOG_INF("Walk-by-S complete.");
}

/* --- Display test --- */
int lcd_test_active_segments(void)
{
    memset(display_ram, 0x00, sizeof(display_ram));
    display_ram[17] &= ~(SEG_A | SEG_F | SEG_E | SEG_D);
    display_ram[18] = (SEG_B | SEG_C);
    return lcd_write_data(0, display_ram, sizeof(display_ram));
}

/* --- Character patterns --- */
typedef struct {
    uint8_t low;
    uint8_t high;
} seg_pattern_t;

static const seg_pattern_t seg_map[36] = {
    /* 0–9 */
    {SEG_A | SEG_F | SEG_E | SEG_D, SEG_B | SEG_C},             // 0
    {0, SEG_B | SEG_C},                                         // 1
    {SEG_A | SEG_E | SEG_D, SEG_B | SEG_G},                     // 2
    {SEG_A | SEG_D, SEG_B | SEG_G | SEG_C},                     // 3
    {SEG_F, SEG_B | SEG_G | SEG_C},                             // 4
    {SEG_A | SEG_F | SEG_D, SEG_G | SEG_C},                     // 5
    {SEG_A | SEG_F | SEG_E | SEG_D, SEG_G | SEG_C},             // 6
    {SEG_A, SEG_B | SEG_C},                                     // 7
    {SEG_A | SEG_F | SEG_E | SEG_D, SEG_B | SEG_G | SEG_C},     // 8
    {SEG_A | SEG_F | SEG_D, SEG_B | SEG_G | SEG_C},             // 9

    /* A–Z */
    {SEG_A | SEG_F | SEG_E, SEG_B | SEG_G | SEG_C},             // A
    {SEG_F | SEG_E | SEG_D, SEG_G | SEG_C},                     // b
    {SEG_A | SEG_F | SEG_E | SEG_D, 0},                         // C
    {SEG_E | SEG_D, SEG_B | SEG_G | SEG_C},                     // d
    {SEG_A | SEG_F | SEG_E | SEG_D, SEG_G},                     // E
    {SEG_A | SEG_F | SEG_E, SEG_G},                             // F
    {SEG_A | SEG_F | SEG_E | SEG_D, SEG_C | SEG_G},             // G
    {SEG_F | SEG_E, SEG_B | SEG_G | SEG_C},                     // H
    {0, SEG_B | SEG_C},                                         // I (same as 1)
    {SEG_E | SEG_D, SEG_B | SEG_C},                             // J
    {SEG_F | SEG_E | SEG_D, SEG_G | SEG_C},                     // K (approximation)
    {SEG_F | SEG_E | SEG_D, 0},                                 // L
    {SEG_E, SEG_B | SEG_C},                                     // M (approximation)
    {SEG_E | SEG_G, SEG_C},                                     // N (approximation)
    {SEG_A | SEG_F | SEG_E | SEG_D, SEG_B | SEG_C},             // O
    {SEG_A | SEG_F | SEG_E, SEG_B | SEG_G},                     // P
    {SEG_A | SEG_F | SEG_E | SEG_D, SEG_B | SEG_C | SEG_G},     // Q
    {SEG_E | SEG_G, SEG_B | SEG_C},                             // R (approximation)
    {SEG_A | SEG_F | SEG_D, SEG_G | SEG_C},                     // S
    {0, SEG_B | SEG_G},                                         // T (approximation)
    {SEG_F | SEG_E | SEG_D, SEG_B | SEG_C},                     // U
    {SEG_E | SEG_D, SEG_C},                                     // V (approximation)
    {SEG_E | SEG_D, SEG_B | SEG_C | SEG_G},                     // W (approximation)
    {SEG_F | SEG_E, SEG_B | SEG_C | SEG_G},                     // X (approximation)
    {SEG_F | SEG_D, SEG_B | SEG_G | SEG_C},                     // Y
    {SEG_A | SEG_E | SEG_D, SEG_B | SEG_G}                      // Z
};

/* --- Display character by position --- */

int showCharacterByPositionChar(int pos, char ch)
{
    uint8_t value;

    if (ch >= '0' && ch <= '9') {
        value = ch - '0';
    } else if (ch >= 'A' && ch <= 'F') {
        value = 10 + (ch - 'A');
    } else if (ch >= 'a' && ch <= 'f') {
        value = 10 + (ch - 'a');
    } else {
        // unsupported → clear the position
        uint8_t low_index  = 18 - pos;
        uint8_t high_index = 19 - pos;
        const uint8_t LOW_MASK  = SEG_A | SEG_F | SEG_E | SEG_D;
        const uint8_t HIGH_MASK = SEG_B | SEG_G | SEG_C;
        display_ram[low_index]  &= ~LOW_MASK;
        display_ram[high_index] &= ~HIGH_MASK;
        return lcd_write_data(0, display_ram, sizeof(display_ram));
    }

    return showCharacterByPosition(pos, value);
}


int showCharacterByPosition(int pos, uint8_t value)
{
    if (!initialized)
        return -ENODEV;

    if (pos < 0 || pos > 6 || value > 0xF)
        return -EINVAL;

    uint8_t low_index  = 11 + pos;   // AFED
    uint8_t high_index = 12 + pos;   // BGC

    // Masks for this digit’s segment bits
    const uint8_t LOW_MASK  = SEG_A | SEG_F | SEG_E | SEG_D;
    const uint8_t HIGH_MASK = SEG_B | SEG_G | SEG_C;

    // 1️⃣ Clear only the bits that belong to this digit
    display_ram[low_index]  &= ~LOW_MASK;
    display_ram[high_index] &= ~HIGH_MASK;

    // 2️⃣ Set new bits for this value (don’t affect others)
    display_ram[low_index]  |= seg_map[value].low;
    display_ram[high_index] |= seg_map[value].high;

    // 3️⃣ Push to LCD
    return lcd_write_data(0, display_ram, sizeof(display_ram));
}

/* --- Decimal points --- */
int showDecimalPoints(int pos)
{
    display_ram[12 + pos] |= (1 << 4);
    return lcd_write_data(0, display_ram, sizeof(display_ram));
}

/* --- Unit indicators --- */
int showUnitKPound(void)
{
    display_ram[18] |= (1 << 2);
    showUnitPound();
    return lcd_write_data(0, display_ram, sizeof(display_ram));
}

int showUnitKGram(void)
{
    display_ram[18] |= (1 << 1);
    showUnitGram();
    return lcd_write_data(0, display_ram, sizeof(display_ram));
}

int showUnitKNewton(void)
{
    display_ram[18] |= (1 << 3);
    showUnitGram();
    return lcd_write_data(0, display_ram, sizeof(display_ram));
}

int showUnitTon(void)
{
    display_ram[18] |= (1 << 0);
    return lcd_write_data(0, display_ram, sizeof(display_ram));
}

int showUnitNewton(void)
{
    display_ram[19] |= (1 << 7);
    return lcd_write_data(0, display_ram, sizeof(display_ram));
}

int showUnitPound(void)
{
    display_ram[19] |= (1 << 6);
    return lcd_write_data(0, display_ram, sizeof(display_ram));
}

int showUnitGram(void)
{
    display_ram[19] |= (1 << 5);
    return lcd_write_data(0, display_ram, sizeof(display_ram));
}

int showUnitThreeDots(void)
{
    display_ram[19] |= (1 << 4);
    return lcd_write_data(0, display_ram, sizeof(display_ram));
}
int lcd_show_string(const char *str)
{
    if (!initialized)
        return -ENODEV;

    size_t len = strlen(str);

    /* We only have 6 digits — fill from right to left (LSB on rightmost) */
    for (int i = 0; i < 6; i++) {
        int pos = i;
        char ch = ' ';  // blank if shorter than 6

        /* show right-aligned text by default */
        if (len - i - 1 >= 0) {
            ch = str[len - i - 1];
        }

        showCharacterByPositionChar(pos, ch);
    }

    return 0;
}

/* --- Corrected: display numeric value directly, no scaling --- */
/* --- Corrected: display numeric value directly, no scaling --- */
int lcd_show_number(double value, int decimal_pos)
{
    if (!initialized)
        return -ENODEV;

    printf("\n[lcd_show_number] value=%.6f, decimal_pos=%d\n", value, decimal_pos);

    int negative = (value < 0);
    if (negative)
        value = -value;

    long scaled = (long)(value + 0.5); // no scaling, just rounding
    printf("[lcd_show_number] scaled=%ld\n", scaled);

    int digits[6] = {0};

    // Extract last 6 digits (right-aligned)
    for (int i = 5; i >= 0; i--) {
        digits[i] = scaled % 10;
        scaled /= 10;
    }

    printf("[lcd_show_number] extracted digits: ");
    for (int i = 0; i < 6; i++)
        printf("%d", digits[i]);
    printf("\n");

    // Display left→right
    for (int pos = 1; pos <= 6; pos++) {
        printf("[lcd_show_number] pos=%d → digit=%d\n", pos, digits[pos - 1]);
        showCharacterByPosition(pos, digits[pos - 1]);
    }

    if (negative)
        showCharacterByPosition(1, '-');

    // Just show decimal point where requested
    showDecimalPoints(decimal_pos);
    printf("[lcd_show_number] showing decimal point at pos=%d\n", decimal_pos);

    return 0;
}
