#include "check.h"
#include "supervisor.h"
#include "tables_gen.h"
#include "test_hal.h"

static uint32_t sense_mv_for(uint32_t ua)
{
    /* inverse of the table, by search: the smallest mV that reads at least ua */
    uint32_t mv = 0;
    while (supervisor_bias_ua(mv) < ua && mv < 3300) mv++;
    return mv;
}

static void usb_supply(uint32_t vbus_mv, uint32_t cc_mv)
{
    test_vhw.adc_mv[HAL_ADC_SENSE_VBUS] = vbus_mv / 2;
    test_vhw.adc_mv[HAL_ADC_SENSE_MAIN] = (vbus_mv - 100) / 2;
    test_vhw.adc_mv[HAL_ADC_SENSE_EXT] = 0;
    test_vhw.adc_mv[HAL_ADC_CC1] = cc_mv;
}

static void sense_table_reads_back_its_own_points(void)
{
    for (int k = 0; k < TABLES_BIAS_SENSE_POINTS; k++)
        CHECK(supervisor_bias_ua(TABLES_BIAS_SENSE_MV[k]) == TABLES_BIAS_SENSE_UA[k]);
    CHECK(supervisor_bias_ua(0) == 0 && supervisor_bias_ua(500) == 0);
    uint32_t prev = 0;
    for (uint32_t mv = 600; mv <= 2000; mv += 10) {
        uint32_t ua = supervisor_bias_ua(mv);
        CHECK(ua >= prev);
        prev = ua;
    }
}

static void cc_voltage_maps_to_the_advertised_current(void)
{
    CHECK(supervisor_cc_from_mv(0) == SUP_CC_NONE);
    CHECK(supervisor_cc_from_mv(410) == SUP_CC_DEFAULT);
    CHECK(supervisor_cc_from_mv(920) == SUP_CC_1A5);
    CHECK(supervisor_cc_from_mv(1680) == SUP_CC_3A);
}

static void bias_needs_the_jack_or_a_1a5_source(void)
{
    supervisor_init();
    usb_supply(5000, 410);
    CHECK(!supervisor_bias_enable(true));
    CHECK(test_vhw.gpio[HAL_GPIO_BIAS_EN] == false);
    usb_supply(5000, 920);
    CHECK(supervisor_bias_enable(true));
    CHECK(test_vhw.gpio[HAL_GPIO_BIAS_EN] == true);
    supervisor_bias_enable(false);
    usb_supply(5000, 410);
    test_vhw.adc_mv[HAL_ADC_SENSE_EXT] = 2500;          /* 5.0 V on the jack */
    CHECK(supervisor_bias_enable(true));
}

static void over_current_cuts_the_feed(void)
{
    supervisor_init();
    usb_supply(5000, 1680);
    CHECK(supervisor_bias_enable(true));
    test_vhw.adc_mv[HAL_ADC_BIAS_ISENSE] = sense_mv_for(140000);
    CHECK(supervisor_poll()->flags & SUP_FLAG_BIAS_ON);
    test_vhw.adc_mv[HAL_ADC_BIAS_ISENSE] = sense_mv_for(160000);
    const sup_status_t *s = supervisor_poll();
    CHECK(!(s->flags & SUP_FLAG_BIAS_ON) && (s->flags & SUP_FLAG_BIAS_TRIPPED));
    CHECK(test_vhw.gpio[HAL_GPIO_BIAS_EN] == false);
    test_vhw.adc_mv[HAL_ADC_BIAS_ISENSE] = sense_mv_for(50000);   /* fault cleared */
    CHECK(supervisor_bias_enable(true));                 /* re-enabling clears the trip */
    s = supervisor_poll();
    CHECK((s->flags & SUP_FLAG_BIAS_ON) && !(s->flags & SUP_FLAG_BIAS_TRIPPED));
}

static void low_usb_is_flagged_and_temperature_read(void)
{
    supervisor_init();
    usb_supply(4600, 410);
    vhw_set_temp_mc(&test_vhw, 41500);
    const sup_status_t *s = supervisor_poll();
    CHECK(s->flags & SUP_FLAG_VBUS_LOW);
    CHECK(s->vbus_mv == 4600);
    CHECK(s->temp_mc == 41500);
    test_vhw.lm75_present = false;
    CHECK(supervisor_poll()->flags & SUP_FLAG_TEMP_FAIL);
}

int main(void)
{
    RUN(sense_table_reads_back_its_own_points);
    RUN(cc_voltage_maps_to_the_advertised_current);
    RUN(bias_needs_the_jack_or_a_1a5_source);
    RUN(over_current_cuts_the_feed);
    RUN(low_usb_is_flagged_and_temperature_read);
    return CHECK_DONE();
}
