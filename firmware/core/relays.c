#include "relays.h"

#include "hal.h"
#include "tables_gen.h"

uint32_t relays_word(int filter, bool atten, bool preamp)
{
    uint32_t w = 0;
    if (filter >= 1 && filter <= TABLES_FILTER_COUNT) {
        int in_bit = RELAY_BIT_BANK1_IN + 2 * (filter - 1);
        w |= 1u << in_bit;
        w |= 1u << (in_bit + 1);                     /* the filter's output relay follows its input relay */
    }
    if (atten) w |= 1u << RELAY_BIT_ATTEN;
    if (preamp) w |= 1u << RELAY_BIT_PREAMP;
    return w;
}

static void shift_latch(uint32_t word)
{
    for (int k = TABLES_RELAY_COUNT - 1; k >= 0; k--) {
        hal_gpio_set(HAL_GPIO_SR_SER, (word >> k) & 1u);
        hal_gpio_set(HAL_GPIO_SR_CLK, true);
        hal_gpio_set(HAL_GPIO_SR_CLK, false);
    }
    hal_gpio_set(HAL_GPIO_SR_LATCH, true);
    hal_gpio_set(HAL_GPIO_SR_LATCH, false);
}

void relays_write(uint32_t word)
{
    shift_latch(word);
    hal_gpio_set(HAL_GPIO_SR_OE, false);
}

void relays_init(void)
{
    hal_gpio_set(HAL_GPIO_SR_OE, true);
    hal_gpio_set(HAL_GPIO_SR_CLK, false);
    hal_gpio_set(HAL_GPIO_SR_LATCH, false);
    relays_write(0);
}
