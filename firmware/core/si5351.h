/* Si5351C register image and programming sequence for a frequency plan (Silicon Labs AN619).
 *
 * The image covers the registers the board uses: 3 (output enable), 15 (PLL source: crystal),
 * 16–19 (CLK0–CLK3 control; CLK4–7 stay powered down), 26–41 (PLLA/PLLB), 42–73 (MultiSynth
 * 0–3), 165–168 (CLK0–3 phase offset), 177 (PLL reset), 183 (crystal load 8 pF, the BOM crystal's CL). Unused
 * outputs are powered down, not only disabled; CLK3 (the calibration tone) runs only when the
 * plan has it. */
#ifndef ELF_SI5351_H
#define ELF_SI5351_H

#include <stdbool.h>
#include <stdint.h>

#include "lo_plan.h"

#define SI5351_ADDR 0x60
#define SI5351_REG_COUNT 256

enum {
    SI5351_REG_OE = 3,
    SI5351_REG_PLL_SRC = 15,
    SI5351_REG_CLK_CTRL = 16,
    SI5351_REG_PLLA = 26,
    SI5351_REG_PLLB = 34,
    SI5351_REG_MS0 = 42,
    SI5351_REG_PHOFF = 165,
    SI5351_REG_PLL_RESET = 177,
    SI5351_REG_XTAL_LOAD = 183,
};

#define SI5351_CLK_PDN 0x80
#define SI5351_CLK_INT 0x40
#define SI5351_CLK_SRC_PLLB 0x20
#define SI5351_CLK_SRC_MS 0x0C
#define SI5351_CLK_DRIVE_8MA 0x03
#define SI5351_XTAL_LOAD_8PF 0x92       /* XTAL_CL = 10 (8 pF, the BOM crystal's CL), low bits 010010 as AN619 requires */
#define SI5351_PLL_RESET_BOTH 0xA0

/* Fills `reg` (all 256 registers, untouched ones 0) for the plan. */
void si5351_image(const lo_plan_t *p, uint8_t reg[SI5351_REG_COUNT]);

/* Encodes a + b/c as AN619's P1, P2, P3 into 8 bytes at `out` (PLL or MultiSynth layout;
 * `r_div` only applies to MultiSynth; pass 1 for a PLL). */
void si5351_params(frac_t x, uint8_t r_div, uint8_t out[8]);

/* Writes the image through hal_i2c_write: outputs off, PLL and MultiSynth parameters, phase
 * offsets, crystal load, PLL reset, outputs on. Returns 0 or the first I2C error. */
int si5351_program(const uint8_t reg[SI5351_REG_COUNT]);

#endif
