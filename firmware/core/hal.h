/* The firmware's only hardware interface. target/esp32s3 implements it on the chip;
 * host/hal_virtual.c implements it as the virtual board. Pin and channel names follow the
 * GPIO map in hardware/characterized/controller.json. */
#ifndef ELF_HAL_H
#define ELF_HAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    HAL_GPIO_LO_PATH,       /* 0 = direct quadrature, 1 = ÷4 path */
    HAL_GPIO_SR_SER,
    HAL_GPIO_SR_CLK,
    HAL_GPIO_SR_LATCH,
    HAL_GPIO_SR_OE,         /* active low: 0 = relay drivers enabled */
    HAL_GPIO_BIAS_EN,
    HAL_GPIO_STATUS_LED,
    HAL_GPIO_ADC_RST,       /* active low */
    HAL_GPIO_ADC_MS,
    HAL_GPIO_ADC_FMT,
    HAL_GPIO_ADC_OVFL,      /* input, active low */
    HAL_GPIO_COUNT
} hal_gpio_t;

typedef enum {
    HAL_ADC_SENSE_VBUS,     /* 100 k / 100 k divider */
    HAL_ADC_SENSE_EXT,
    HAL_ADC_SENSE_MAIN,
    HAL_ADC_BIAS_ISENSE,    /* bias-T current mirror */
    HAL_ADC_CC1,
    HAL_ADC_CC2,
    HAL_ADC_COUNT
} hal_adc_t;

int hal_i2c_write(uint8_t addr, uint8_t reg, const uint8_t *data, size_t len);   /* 0 = ACK */
int hal_i2c_read(uint8_t addr, uint8_t reg, uint8_t *data, size_t len);
void hal_gpio_set(hal_gpio_t pin, bool level);
bool hal_gpio_get(hal_gpio_t pin);
uint32_t hal_adc_mv(hal_adc_t ch);
/* Up to max_frames I2S frames, interleaved I, Q as signed 24-bit values in int32; returns the
 * number read (0 when none are ready). */
size_t hal_i2s_read(int32_t *iq, size_t max_frames);
uint32_t hal_millis(void);
void hal_delay_ms(uint32_t ms);

#endif
