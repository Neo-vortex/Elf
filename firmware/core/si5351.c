#include "si5351.h"

#include <string.h>

#include "hal.h"

static uint8_t log2_u8(uint8_t v)
{
    uint8_t k = 0;
    while (v > 1) {
        v >>= 1;
        k++;
    }
    return k;
}

void si5351_params(frac_t x, uint8_t r_div, uint8_t out[8])
{
    uint32_t a = (uint32_t)(x.num / x.den), b = (uint32_t)(x.num % x.den), c = (uint32_t)x.den;
    uint32_t f = (uint32_t)(((uint64_t)128 * b) / c);
    uint32_t p1 = 128 * a + f - 512;
    uint32_t p2 = 128 * b - c * f;
    uint32_t p3 = c;
    out[0] = (uint8_t)(p3 >> 8);
    out[1] = (uint8_t)p3;
    out[2] = (uint8_t)((log2_u8(r_div) << 4) | ((p1 >> 16) & 0x03));
    out[3] = (uint8_t)(p1 >> 8);
    out[4] = (uint8_t)p1;
    out[5] = (uint8_t)(((p3 >> 12) & 0xF0) | ((p2 >> 16) & 0x0F));
    out[6] = (uint8_t)(p2 >> 8);
    out[7] = (uint8_t)p2;
}

void si5351_image(const lo_plan_t *p, uint8_t reg[SI5351_REG_COUNT])
{
    memset(reg, 0, SI5351_REG_COUNT);
    reg[SI5351_REG_OE] = 0xFF;
    reg[SI5351_REG_PLL_SRC] = 0x00;                  /* both PLLs from the crystal */
    for (int k = 0; k < 8; k++) reg[SI5351_REG_CLK_CTRL + k] = SI5351_CLK_PDN;
    for (int pll = 0; pll < 2; pll++)
        if (p->pll_used[pll]) si5351_params(p->pll_mult[pll], 1, &reg[pll ? SI5351_REG_PLLB : SI5351_REG_PLLA]);
    for (int clk = 0; clk < 4; clk++) {
        const lo_output_t *o = &p->out[clk];
        if (!o->used) continue;
        uint8_t ctrl = SI5351_CLK_SRC_MS | SI5351_CLK_DRIVE_8MA;
        if (o->ms.den == 1) ctrl |= SI5351_CLK_INT;
        if (o->pll == LO_PLL_B) ctrl |= SI5351_CLK_SRC_PLLB;
        reg[SI5351_REG_CLK_CTRL + clk] = ctrl;
        si5351_params(o->ms, o->r_div, &reg[SI5351_REG_MS0 + 8 * clk]);
        reg[SI5351_REG_PHOFF + clk] = o->phoff & 0x7F;
        reg[SI5351_REG_OE] &= (uint8_t)~(1u << clk);
    }
    reg[SI5351_REG_PLL_RESET] = SI5351_PLL_RESET_BOTH;
    reg[SI5351_REG_XTAL_LOAD] = SI5351_XTAL_LOAD_8PF;
}

static int put(const uint8_t *reg, uint8_t first, uint8_t count)
{
    return hal_i2c_write(SI5351_ADDR, first, &reg[first], count);
}

int si5351_program(const uint8_t reg[SI5351_REG_COUNT])
{
    static const uint8_t off = 0xFF;
    int e;
    if ((e = hal_i2c_write(SI5351_ADDR, SI5351_REG_OE, &off, 1))) return e;
    if ((e = put(reg, SI5351_REG_PLL_SRC, 1))) return e;
    if ((e = put(reg, SI5351_REG_CLK_CTRL, 8))) return e;
    if ((e = put(reg, SI5351_REG_PLLA, 16))) return e;
    if ((e = put(reg, SI5351_REG_MS0, 32))) return e;
    if ((e = put(reg, SI5351_REG_PHOFF, 4))) return e;
    if ((e = put(reg, SI5351_REG_XTAL_LOAD, 1))) return e;
    if ((e = put(reg, SI5351_REG_PLL_RESET, 1))) return e;
    return put(reg, SI5351_REG_OE, 1);
}
