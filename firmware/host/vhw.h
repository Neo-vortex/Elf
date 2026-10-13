/* Virtual board hardware: the parts the firmware drives, modelled at their pins and registers.
 * Used by the unit tests and by elf-virtual. The models follow the datasheets, not the
 * firmware: the 74HC595 chain shifts on SRCLK rising edges and latches on RCLK rising edges;
 * the Si5351C is a 256-byte register file with auto-incrementing writes. */
#ifndef ELF_VHW_H
#define ELF_VHW_H

#include <stdbool.h>
#include <stdint.h>

#include "hal.h"

#define VHW_CHAIN_BITS 24

typedef struct {
    uint8_t si5351[256];
    bool si5351_present, lm75_present;
    int16_t lm75_reg0;                 /* LM75A temperature register (11-bit value << 5) */
    bool gpio[HAL_GPIO_COUNT];
    uint32_t adc_mv[HAL_ADC_COUNT];
    uint32_t chain_shift;              /* shift-register stages, bit k = chain bit k */
    uint32_t chain_latched;            /* storage registers (the Q outputs when /OE is low) */
    uint32_t si5351_writes;            /* I2C write transactions to the Si5351 */
} vhw_t;

void vhw_reset(vhw_t *v);
int vhw_i2c_write(vhw_t *v, uint8_t addr, uint8_t reg, const uint8_t *data, size_t len);
int vhw_i2c_read(vhw_t *v, uint8_t addr, uint8_t reg, uint8_t *data, size_t len);
void vhw_gpio_set(vhw_t *v, hal_gpio_t pin, bool level);
/* The relay coils energised now: latched bits while /OE is low, none while it is high. */
uint32_t vhw_relay_outputs(const vhw_t *v);
void vhw_set_temp_mc(vhw_t *v, int32_t milli_c);

#endif
