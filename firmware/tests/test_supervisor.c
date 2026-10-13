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

/* the board's monitors as the dividers present them (ratios from power.json) */
static void supplies(uint32_t jack_mv, uint32_t main_mv, uint32_t vbus_mv)
{
    test_vhw.adc_mv[HAL_ADC_SENSE_EXT] = (uint32_t)((uint64_t)jack_mv * 1000 / TABLES_SENSE_DIV_EXT_X1000);
    test_vhw.adc_mv[HAL_ADC_SENSE_MAIN] = (uint32_t)((uint64_t)main_mv * 1000 / TABLES_SENSE_DIV_MAIN_X1000);
    test_vhw.adc_mv[HAL_ADC_SENSE_VBUS] = (uint32_t)((uint64_t)vbus_mv * 1000 / TABLES_SENSE_DIV_VBUS_X1000);
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

static void monitors_read_through_the_dividers(void)
{
    supervisor_init();
    supplies(12000, 5480, 5000);
    const sup_status_t *s = supervisor_poll();
    CHECK(s->ext_mv >= 11980 && s->ext_mv <= 12020);              /* 100 k / 10 k: 30 V fits the ADC */
    CHECK(s->main_mv >= 5470 && s->main_mv <= 5490);
    CHECK((s->flags & SUP_FLAG_EXT) && (s->flags & SUP_FLAG_HOST) && !(s->flags & SUP_FLAG_INPUT_LOW));
    supplies(7500, 5480, 0);                                       /* input sagging, no host */
    s = supervisor_poll();
    CHECK((s->flags & SUP_FLAG_INPUT_LOW) && !(s->flags & SUP_FLAG_HOST));
}

static void bias_needs_5v_main_in_regulation(void)
{
    supervisor_init();
    supplies(12000, 4800, 5000);                                   /* buck not up yet */
    CHECK(!supervisor_bias_enable(true));
    CHECK(test_vhw.gpio[HAL_GPIO_BIAS_EN] == false);
    supplies(12000, 5480, 0);                                      /* no USB host needed: the jack powers it */
    CHECK(supervisor_bias_enable(true));
    CHECK(test_vhw.gpio[HAL_GPIO_BIAS_EN] == true);
}

static void bias_is_allowed_across_the_main_rail_tolerance(void)
{
    /* LM2576 Vref 1.193–1.267 V and 1 % divider: 5V_MAIN may sit near 5.2 V on a good board */
    supervisor_init();
    supplies(12000, 5200, 5000);
    CHECK(supervisor_bias_enable(true));
    CHECK(supervisor_poll()->flags & SUP_FLAG_BIAS_ON);
}

static void bias_drops_when_main_falls(void)
{
    supervisor_init();
    supplies(12000, 5480, 5000);
    CHECK(supervisor_bias_enable(true));
    supplies(6000, 4600, 5000);                                    /* the jack pulled: main collapsing */
    const sup_status_t *s = supervisor_poll();
    CHECK(!(s->flags & SUP_FLAG_BIAS_ON) && test_vhw.gpio[HAL_GPIO_BIAS_EN] == false);
}

static void over_current_cuts_the_feed(void)
{
    supervisor_init();
    supplies(12000, 5480, 5000);
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

static void sense_zero_is_measured_with_the_feed_off(void)
{
    /* a board whose mirror reads 0.70 V at zero current (the table's zero is 0.628 V) */
    supervisor_init();
    supplies(12000, 5480, 5000);
    test_vhw.adc_mv[HAL_ADC_BIAS_ISENSE] = 700;       /* feed off: the zero reading */
    CHECK(supervisor_bias_enable(true));
    CHECK(supervisor_poll()->bias_ua < 1000);           /* still no current: reads ≈ 0, not ≈ 14 mA */
    test_vhw.adc_mv[HAL_ADC_BIAS_ISENSE] = sense_mv_for(100000) + (700 - TABLES_BIAS_SENSE_MV[0]);
    uint32_t ua = supervisor_poll()->bias_ua;
    CHECK(ua > 98000 && ua < 102000);
}

static void temperature_is_read_and_a_missing_sensor_flagged(void)
{
    supervisor_init();
    supplies(12000, 5480, 5000);
    vhw_set_temp_mc(&test_vhw, 41500);
    CHECK(supervisor_poll()->temp_mc == 41500);
    test_vhw.lm75_present = false;
    CHECK(supervisor_poll()->flags & SUP_FLAG_TEMP_FAIL);
}

int main(void)
{
    RUN(sense_table_reads_back_its_own_points);
    RUN(monitors_read_through_the_dividers);
    RUN(bias_needs_5v_main_in_regulation);
    RUN(bias_is_allowed_across_the_main_rail_tolerance);
    RUN(bias_drops_when_main_falls);
    RUN(over_current_cuts_the_feed);
    RUN(sense_zero_is_measured_with_the_feed_off);
    RUN(temperature_is_read_and_a_missing_sensor_flagged);
    return CHECK_DONE();
}
