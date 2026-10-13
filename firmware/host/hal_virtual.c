#include "hal_virtual.h"

#include <time.h>

#include "hal.h"

#define I2S_QUEUE_FRAMES 16384u             /* power of two, ≈ 170 ms */

vhw_t vboard;
static int32_t i2s[I2S_QUEUE_FRAMES][2];
static uint32_t i2s_head, i2s_tail;
static struct timespec t0;

void hal_virtual_init(void)
{
    vhw_reset(&vboard);
    i2s_head = i2s_tail = 0;
    clock_gettime(CLOCK_MONOTONIC, &t0);
}

uint64_t hal_virtual_now_ms(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint64_t)(t.tv_sec - t0.tv_sec) * 1000u + (uint64_t)((t.tv_nsec - t0.tv_nsec) / 1000000);
}

void hal_virtual_i2s_push(const int32_t *iq, size_t frames)
{
    for (size_t k = 0; k < frames; k++) {
        if (i2s_head - i2s_tail >= I2S_QUEUE_FRAMES) i2s_tail++;   /* the ADC never waits: oldest frame lost */
        i2s[i2s_head % I2S_QUEUE_FRAMES][0] = iq[2 * k];
        i2s[i2s_head % I2S_QUEUE_FRAMES][1] = iq[2 * k + 1];
        i2s_head++;
    }
}

int hal_i2c_write(uint8_t addr, uint8_t reg, const uint8_t *data, size_t len) { return vhw_i2c_write(&vboard, addr, reg, data, len); }
int hal_i2c_read(uint8_t addr, uint8_t reg, uint8_t *data, size_t len) { return vhw_i2c_read(&vboard, addr, reg, data, len); }
void hal_gpio_set(hal_gpio_t pin, bool level) { vhw_gpio_set(&vboard, pin, level); }
bool hal_gpio_get(hal_gpio_t pin) { return vboard.gpio[pin]; }
uint32_t hal_adc_mv(hal_adc_t ch) { return vboard.adc_mv[ch]; }

size_t hal_i2s_read(int32_t *iq, size_t max_frames)
{
    size_t n = 0;
    while (n < max_frames && i2s_tail != i2s_head) {
        iq[2 * n] = i2s[i2s_tail % I2S_QUEUE_FRAMES][0];
        iq[2 * n + 1] = i2s[i2s_tail % I2S_QUEUE_FRAMES][1];
        i2s_tail++;
        n++;
    }
    return n;
}

uint32_t hal_millis(void) { return (uint32_t)hal_virtual_now_ms(); }

/* TinyUSB's time source (OPT_OS_NONE) */
uint32_t tusb_time_millis_api(void);
uint32_t tusb_time_millis_api(void) { return hal_millis(); }

void hal_delay_ms(uint32_t ms)
{
    struct timespec d = {.tv_sec = ms / 1000, .tv_nsec = (long)(ms % 1000) * 1000000L};
    nanosleep(&d, NULL);
}
