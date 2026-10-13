#include "test_hal.h"

#include <string.h>

vhw_t test_vhw;
uint32_t test_now_ms;
static int32_t i2s[8192][2];
static size_t i2s_head, i2s_tail;

void test_hal_reset(void)
{
    vhw_reset(&test_vhw);
    test_now_ms = 0;
    i2s_head = i2s_tail = 0;
}

void test_i2s_queue(const int32_t *iq, size_t frames)
{
    for (size_t k = 0; k < frames && i2s_head < 8192; k++, i2s_head++) {
        i2s[i2s_head][0] = iq[2 * k];
        i2s[i2s_head][1] = iq[2 * k + 1];
    }
}

int hal_i2c_write(uint8_t addr, uint8_t reg, const uint8_t *data, size_t len)
{
    return vhw_i2c_write(&test_vhw, addr, reg, data, len);
}

int hal_i2c_read(uint8_t addr, uint8_t reg, uint8_t *data, size_t len)
{
    return vhw_i2c_read(&test_vhw, addr, reg, data, len);
}

void hal_gpio_set(hal_gpio_t pin, bool level) { vhw_gpio_set(&test_vhw, pin, level); }
bool hal_gpio_get(hal_gpio_t pin) { return test_vhw.gpio[pin]; }
uint32_t hal_adc_mv(hal_adc_t ch) { return test_vhw.adc_mv[ch]; }

size_t hal_i2s_read(int32_t *iq, size_t max_frames)
{
    size_t n = 0;
    while (n < max_frames && i2s_tail < i2s_head) {
        iq[2 * n] = i2s[i2s_tail][0];
        iq[2 * n + 1] = i2s[i2s_tail][1];
        n++;
        i2s_tail++;
    }
    return n;
}

uint32_t hal_millis(void) { return test_now_ms; }
void hal_delay_ms(uint32_t ms) { test_now_ms += ms; }
