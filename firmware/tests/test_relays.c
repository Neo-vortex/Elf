#include "check.h"
#include "relays.h"
#include "tables_gen.h"
#include "test_hal.h"

static void word_selects_both_relays_of_one_filter(void)
{
    CHECK(relays_word(7, false, false) == ((1u << RELAY_BIT_BANK7_IN) | (1u << RELAY_BIT_BANK7_OUT)));
    CHECK(relays_word(1, true, true) == (0x3u | (1u << RELAY_BIT_ATTEN) | (1u << RELAY_BIT_PREAMP)));
    CHECK(relays_word(0, false, false) == 0);           /* muted bank */
    CHECK(relays_word(12, false, false) == 0);          /* no such filter: muted, not a stray bit */
}

static void write_lands_every_bit_on_its_own_output(void)
{
    for (int k = 0; k < TABLES_RELAY_COUNT; k++) {
        relays_write(1u << k);
        CHECK(vhw_relay_outputs(&test_vhw) == (1u << k));
    }
    relays_write(0xA5C3E1u);
    CHECK(vhw_relay_outputs(&test_vhw) == 0xA5C3E1u);
}

static void init_enables_the_outputs_only_after_latching_all_released(void)
{
    test_vhw.chain_latched = 0xFFFFFFu;                 /* random power-up storage contents */
    relays_init();
    CHECK(vhw_relay_outputs(&test_vhw) == 0);
    CHECK(test_vhw.gpio[HAL_GPIO_SR_OE] == false);
}

int main(void)
{
    RUN(word_selects_both_relays_of_one_filter);
    RUN(write_lands_every_bit_on_its_own_output);
    RUN(init_enables_the_outputs_only_after_latching_all_released);
    return CHECK_DONE();
}
