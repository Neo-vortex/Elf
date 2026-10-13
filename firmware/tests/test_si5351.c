#include <string.h>

#include "check.h"
#include "si5351.h"
#include "test_hal.h"

static void params_follow_an619(void)
{
    uint8_t b[8];
    /* 36 exactly: P1 = 128·36 − 512 = 4096, P2 = 0, P3 = 1 */
    si5351_params((frac_t){36, 1}, 1, b);
    const uint8_t want_int[8] = {0x00, 0x01, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00};
    CHECK(memcmp(b, want_int, 8) == 0);
    /* 35 + 5/7, R = 8: floor(128·5/7) = 91; P1 = 4480 + 91 − 512 = 4059; P2 = 640 − 637 = 3; P3 = 7 */
    si5351_params((frac_t){250, 7}, 8, b);
    const uint8_t want_frac[8] = {0x00, 0x07, 0x30, 0x0F, 0xDB, 0x00, 0x00, 0x03};
    CHECK(memcmp(b, want_frac, 8) == 0);
    /* 20-bit P3 and P2 high nibbles share byte 5 */
    si5351_params((frac_t){15LL * 1048575 + 1, 1048575}, 1, b);
    CHECK(b[5] == 0xF0 && b[0] == 0xFF && b[1] == 0xFF);
}

static void integer_outputs_set_ms_int_and_fractional_ones_do_not(void)
{
    lo_plan_t p;
    uint8_t reg[SI5351_REG_COUNT];
    CHECK(lo_plan(3550000, true, 10000, &p) == LO_OK);   /* ÷4 path: CLK2 integer, CLK3 fractional */
    CHECK(p.out[2].ms.den == 1 && p.out[3].ms.den != 1);
    si5351_image(&p, reg);
    CHECK(reg[SI5351_REG_CLK_CTRL + 2] & SI5351_CLK_INT);
    CHECK(!(reg[SI5351_REG_CLK_CTRL + 3] & SI5351_CLK_INT));
    CHECK(reg[SI5351_REG_CLK_CTRL + 2] & SI5351_CLK_SRC_PLLB);
    CHECK((reg[SI5351_REG_MS0 + 8 * 2 + 2] >> 4) == __builtin_ctz(p.out[2].r_div));
}

static void unused_outputs_are_powered_down_and_disabled(void)
{
    lo_plan_t p;
    uint8_t reg[SI5351_REG_COUNT];
    CHECK(lo_plan(14050000, false, 0, &p) == LO_OK);
    si5351_image(&p, reg);
    CHECK(reg[SI5351_REG_OE] == 0xFC);                   /* CLK0, CLK1 on */
    for (int k = 2; k < 8; k++) CHECK(reg[SI5351_REG_CLK_CTRL + k] == SI5351_CLK_PDN);
    CHECK(reg[SI5351_REG_CLK_CTRL] == (SI5351_CLK_INT | SI5351_CLK_SRC_MS | SI5351_CLK_DRIVE_8MA));
    CHECK(reg[SI5351_REG_PHOFF + 1] == (uint8_t)p.out[0].ms.num && reg[SI5351_REG_PHOFF] == 0);
    CHECK(reg[SI5351_REG_XTAL_LOAD] == 0x92 && reg[SI5351_REG_PLL_SRC] == 0);   /* 8 pF: the BOM crystal's CL */
}

static void program_turns_outputs_off_first_and_on_last(void)
{
    lo_plan_t p;
    uint8_t reg[SI5351_REG_COUNT];
    lo_plan(14050000, false, 0, &p);
    si5351_image(&p, reg);
    CHECK(si5351_program(reg) == 0);
    CHECK(test_vhw.si5351[SI5351_REG_OE] == 0xFC);
    CHECK(memcmp(&test_vhw.si5351[SI5351_REG_PLLA], &reg[SI5351_REG_PLLA], 48) == 0);
    test_vhw.si5351_present = false;
    CHECK(si5351_program(reg) != 0);
}

int main(void)
{
    RUN(params_follow_an619);
    RUN(integer_outputs_set_ms_int_and_fractional_ones_do_not);
    RUN(unused_outputs_are_powered_down_and_disabled);
    RUN(program_turns_outputs_off_first_and_on_last);
    return CHECK_DONE();
}
