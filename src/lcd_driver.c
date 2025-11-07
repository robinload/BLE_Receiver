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

#define MODE_ENABLE         (1 << 3)   /* Display enable */
#define MODE_BIAS_1_3       (0 << 2)
#define MODE_DRIVE_1_4      (0x00)
#define LCD_RAM_SIZE 20

/* --- Internal state --- */
static const struct device *i2c_dev;
static uint8_t display_ram[LCD_RAM_SIZE];
static bool initialized = false;

/* --- Internal helper --- */
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

    /* MODE_SET + LOAD_DP + DEVICE_SELECT (all in one transaction) */
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

/* --- Test pattern --- */
int lcd_test_all_segments(void)
{
    memset(display_ram, 0xFF, sizeof(display_ram));
    return lcd_write_data(0, display_ram, sizeof(display_ram));
}


void lcd_test_walking_segments(void)
{
    if (!initialized)
        return -ENODEV;

    memset(display_ram, 0x00, sizeof(display_ram));

    // Light up only S20–S39 (RAM bytes 10–19)
    for (int i = 10; i < 20; i++) {
        display_ram[i] = 0xFF;
    }

    int ret = lcd_write_data(0, display_ram, sizeof(display_ram));
    if (ret == 0) {
        LOG_INF("Active segments (S20–S39) ON");
    } else {
        LOG_ERR("Failed to update active segment range: %d", ret);
    }

    return ret;
}

void lcd_test_walking_active_segments(void)
{
    if (!initialized)
        return;

    LOG_INF("Starting walking segment test (S20–S39)...");

    memset(display_ram, 0x00, sizeof(display_ram));

    // Walk through S20–S39 (RAM bytes 10–19)
    for (int s = 10; s < 20; s++) {
        for (int com = 0; com < 4; com++) {
            memset(display_ram, 0x00, sizeof(display_ram));

            // Light one COM line of one segment
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
    if (!initialized) return;

    LOG_INF("Walking by S-line (lighting all COMs per S)...");
    memset(display_ram, 0x00, sizeof(display_ram));

    for (int s = 0; s < 40; s++) {
        memset(display_ram, 0x00, sizeof(display_ram));
        display_ram[s] = 0xFF;  // all COMs for this S
        lcd_write_data(0, display_ram, sizeof(display_ram));
        LOG_INF("Lighting S%d (COM0–3)", s);
        k_msleep(400);
    }

    LOG_INF("Walk-by-S complete.");
}

int lcd_test_active_segments(void)
{
    if (!initialized)
        return -ENODEV;

    memset(display_ram, 0x00, sizeof(display_ram));

    // Light up segments S20–S39 (the active range you identified)
    // Using all COMs to ensure complete segment activation
    for (int s = 0; s < 40; s++) {
        display_ram[s] = 0x0F;  // Light COM0-COM3 (bits 0-3)
    }

    // Alternative: Test specific range
    // for (int s = 10; s < 20; s++) {
    //     display_ram[s] = 0x0F;
    // }

    return lcd_write_data(0, display_ram, sizeof(display_ram));
}