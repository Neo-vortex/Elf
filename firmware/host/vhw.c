#include "vhw.h"

#include <string.h>

#include "si5351.h"
#include "tables_gen.h"

void vhw_reset(vhw_t *v)
{
    memset(v, 0, sizeof *v);
    v->si5351_present = v->lm75_present = true;
    /* power-up register defaults the firmware relies on being overwritten: outputs disabled */
    v->si5351[SI5351_REG_OE] = 0xFF;
    for (int k = 0; k < 8; k++) v->si5351[SI5351_REG_CLK_CTRL + k] = SI5351_CLK_PDN;
    /* board pull resistors: /OE high (outputs off), bias EN low, ADC RST low, LO path low */
    v->gpio[HAL_GPIO_SR_OE] = true;
    v->gpio[HAL_GPIO_ADC_OVFL] = true;
    vhw_set_temp_mc(v, 25000);
}

int vhw_i2c_write(vhw_t *v, uint8_t addr, uint8_t reg, const uint8_t *data, size_t len)
{
    if (addr == TABLES_I2C_SI5351 && v->si5351_present) {
        for (size_t k = 0; k < len; k++) v->si5351[(uint8_t)(reg + k)] = data[k];
        v->si5351_writes++;
        return 0;
    }
    if (addr == TABLES_I2C_LM75A && v->lm75_present) return 0;
    return -1;                                       /* no ACK */
}

int vhw_i2c_read(vhw_t *v, uint8_t addr, uint8_t reg, uint8_t *data, size_t len)
{
    if (addr == TABLES_I2C_SI5351 && v->si5351_present) {
        for (size_t k = 0; k < len; k++) data[k] = v->si5351[(uint8_t)(reg + k)];
        return 0;
    }
    if (addr == TABLES_I2C_LM75A && v->lm75_present) {
        uint16_t t = (uint16_t)v->lm75_reg0;
        for (size_t k = 0; k < len; k++) data[k] = reg == 0 ? (uint8_t)(k % 2 ? t : t >> 8) : 0;
        return 0;
    }
    return -1;
}

void vhw_gpio_set(vhw_t *v, hal_gpio_t pin, bool level)
{
    bool rising = level && !v->gpio[pin];
    v->gpio[pin] = level;
    if (pin == HAL_GPIO_SR_CLK && rising) {
        /* SER enters the first register's QA; every stage moves one place along the chain */
        v->chain_shift = ((v->chain_shift << 1) | (v->gpio[HAL_GPIO_SR_SER] ? 1u : 0u)) & ((1u << VHW_CHAIN_BITS) - 1);
    }
    if (pin == HAL_GPIO_SR_LATCH && rising) v->chain_latched = v->chain_shift;
}

uint32_t vhw_relay_outputs(const vhw_t *v) { return v->gpio[HAL_GPIO_SR_OE] ? 0 : v->chain_latched; }

void vhw_set_temp_mc(vhw_t *v, int32_t milli_c) { v->lm75_reg0 = (int16_t)((milli_c / 125) << 5); }
