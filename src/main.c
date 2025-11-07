#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "lcd_driver.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

void main(void)
{
    LOG_INF("=== PCF8576 LCD Demo ===");

    int ret = lcd_init();
    if (ret) {
        LOG_ERR("LCD init failed: %d", ret);
        return;
    }

    k_msleep(200);
    int ret2 = lcd_test_active_segments();
    LOG_INF("All segments ON");
}
