#include <string.h>

#include "check.h"
#include "radio.h"
#include "relays.h"
#include "si5351.h"
#include "test_hal.h"

static void filter_edges_follow_the_bank_table(void)
{
    CHECK(radio_filter_for(100000) == 1);
    CHECK(radio_filter_for(499999) == 1);
    CHECK(radio_filter_for(500000) == 2);
    CHECK(radio_filter_for(14050000) == 7);
    CHECK(radio_filter_for(54000000) == 11);
    CHECK(radio_filter_for(99999) == 0);
    CHECK(radio_filter_for(54000001) == 0);
}

static void tune_direct_programs_the_si5351_path_and_filter(void)
{
    radio_init();
    CHECK(radio_tune(14050000) == RADIO_OK);
    lo_plan_t p;
    uint8_t want[SI5351_REG_COUNT];
    CHECK(lo_plan(14050000, false, 0, &p) == LO_OK);
    si5351_image(&p, want);
    CHECK(memcmp(&test_vhw.si5351[SI5351_REG_PLLA], &want[SI5351_REG_PLLA], 48) == 0);
    CHECK(test_vhw.si5351[SI5351_REG_OE] == want[SI5351_REG_OE]);
    CHECK(test_vhw.si5351[SI5351_REG_PHOFF + 1] == p.out[1].phoff);
    CHECK(test_vhw.gpio[HAL_GPIO_LO_PATH] == false);
    CHECK(vhw_relay_outputs(&test_vhw) == relays_word(7, false, false));
    CHECK(radio_state()->lo_hz == 14050000 && radio_state()->path == LO_PATH_DIRECT);
}

static void tune_low_selects_the_divide_by_four_path(void)
{
    radio_init();
    CHECK(radio_tune(3550000) == RADIO_OK);
    CHECK(test_vhw.gpio[HAL_GPIO_LO_PATH] == true);
    CHECK((test_vhw.si5351[SI5351_REG_OE] & 0x04) == 0);        /* CLK2 on */
    CHECK((test_vhw.si5351[SI5351_REG_OE] & 0x03) == 0x03);     /* CLK0/1 off */
    CHECK(vhw_relay_outputs(&test_vhw) == relays_word(4, false, false));
}

static void out_of_range_leaves_the_radio_as_it_was(void)
{
    radio_init();
    radio_tune(7030000);
    uint32_t before = vhw_relay_outputs(&test_vhw);
    CHECK(radio_tune(60000000) == RADIO_ERR_RANGE);
    CHECK(radio_tune(50000) == RADIO_ERR_RANGE);
    CHECK(radio_state()->lo_hz == 7030000 && vhw_relay_outputs(&test_vhw) == before);
}

static void missing_si5351_is_an_i2c_error(void)
{
    radio_init();
    test_vhw.si5351_present = false;
    CHECK(radio_tune(14050000) == RADIO_ERR_I2C);
    CHECK(radio_state()->lo_hz == 0);
}

static void frontend_keeps_the_filter(void)
{
    radio_init();
    radio_tune(21050000);
    radio_set_frontend(true, true);
    CHECK(vhw_relay_outputs(&test_vhw) == relays_word(8, true, true));
    uint32_t t0 = test_now_ms;
    radio_set_frontend(false, true);
    CHECK(test_now_ms - t0 >= RADIO_RELAY_SETTLE_MS);           /* waits for the relays */
}

static void cal_tone_runs_clk3_only_while_on(void)
{
    radio_init();
    radio_tune(14050000);
    CHECK(test_vhw.si5351[SI5351_REG_CLK_CTRL + 3] & SI5351_CLK_PDN);
    CHECK(radio_set_cal_tone(true, 10000) == RADIO_OK);
    CHECK(!(test_vhw.si5351[SI5351_REG_CLK_CTRL + 3] & SI5351_CLK_PDN));
    CHECK((test_vhw.si5351[SI5351_REG_OE] & 0x08) == 0);
    CHECK(radio_set_cal_tone(false, 0) == RADIO_OK);
    CHECK(test_vhw.si5351[SI5351_REG_CLK_CTRL + 3] & SI5351_CLK_PDN);   /* powered down, not only disabled */
}

int main(void)
{
    RUN(filter_edges_follow_the_bank_table);
    RUN(tune_direct_programs_the_si5351_path_and_filter);
    RUN(tune_low_selects_the_divide_by_four_path);
    RUN(out_of_range_leaves_the_radio_as_it_was);
    RUN(missing_si5351_is_an_i2c_error);
    RUN(frontend_keeps_the_filter);
    RUN(cal_tone_runs_clk3_only_while_on);
    return CHECK_DONE();
}
