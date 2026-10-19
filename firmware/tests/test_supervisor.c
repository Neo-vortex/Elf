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
    supervisor_init();                                              /* China's INA180 table until a zero says otherwise */
    for (int k = 0; k < TABLES_BIAS_SENSE_INA180_POINTS; k++)
        CHECK(supervisor_bias_ua(TABLES_BIAS_SENSE_INA180_MV[k]) == TABLES_BIAS_SENSE_INA180_UA[k]);
    CHECK(supervisor_bias_ua(0) == 0);
    uint32_t prev = 0;
    for (uint32_t mv = 10; mv <= 3200; mv += 10) {
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
    CHECK(s->ext_mv >= 11980 && s->ext_mv <= 12020);              /* 100 k / 10 k: 20 V fits the ADC */
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
    test_vhw.adc_mv[HAL_ADC_BIAS_ISENSE] = 3;                       /* feed off: the zero again */
    CHECK(supervisor_bias_enable(true));                 /* re-enabling clears the trip */
    test_vhw.adc_mv[HAL_ADC_BIAS_ISENSE] = sense_mv_for(50000);   /* fault cleared */
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
    test_vhw.adc_mv[HAL_ADC_BIAS_ISENSE] = sense_mv_for(100000) + (700 - TABLES_BIAS_SENSE_MIRROR_MV[0]);
    uint32_t ua = supervisor_poll()->bias_ua;
    CHECK(ua > 98000 && ua < 102000);
}

static void bias_sense_table_follows_the_zero_reading(void)
{
    /* mirror zero ≈ 630 mV picks the mirror table; INA180 zero ≈ 0 mV picks the INA180 table */
    supplies(12000, 5480, 5000);
    supervisor_init();
    test_vhw.adc_mv[HAL_ADC_BIAS_ISENSE] = 630;
    CHECK(supervisor_bias_enable(true));
    CHECK(supervisor_bias_ua(TABLES_BIAS_SENSE_MIRROR_MV[3]) == TABLES_BIAS_SENSE_MIRROR_UA[3]);
    CHECK(supervisor_bias_enable(false));
    test_vhw.adc_mv[HAL_ADC_BIAS_ISENSE] = 2;
    CHECK(supervisor_bias_enable(true));
    CHECK(supervisor_bias_ua(TABLES_BIAS_SENSE_INA180_MV[3]) == TABLES_BIAS_SENSE_INA180_UA[3]);
    CHECK(!(supervisor_poll()->flags & SUP_FLAG_BIAS_SENSE_FAULT));
}

static void ina180_reading_is_referred_to_its_own_zero(void)
{
    supplies(12000, 5480, 5000);
    supervisor_init();
    test_vhw.adc_mv[HAL_ADC_BIAS_ISENSE] = 20;                       /* this board's zero is 20 mV: still an INA180A1 */
    CHECK(supervisor_bias_enable(true));
    test_vhw.adc_mv[HAL_ADC_BIAS_ISENSE] = 20 + TABLES_BIAS_SENSE_INA180_MV[3] - TABLES_BIAS_SENSE_INA180_MV[0];
    uint32_t ua = supervisor_poll()->bias_ua;
    CHECK(ua > TABLES_BIAS_SENSE_INA180_UA[3] - 1500 && ua < TABLES_BIAS_SENSE_INA180_UA[3] + 1500);
}

static void a_zero_between_the_two_circuits_is_a_sense_fault(void)
{
    supplies(12000, 5480, 5000);
    supervisor_init();
    test_vhw.adc_mv[HAL_ADC_BIAS_ISENSE] = 300;
    CHECK(!supervisor_bias_enable(true));                            /* neither circuit reads 300 mV at zero */
    CHECK(test_vhw.gpio[HAL_GPIO_BIAS_EN] == false);
    CHECK(supervisor_poll()->flags & SUP_FLAG_BIAS_SENSE_FAULT);
    CHECK(!(supervisor_poll()->flags & SUP_FLAG_BIAS_ON));
    test_vhw.adc_mv[HAL_ADC_BIAS_ISENSE] = 630;                      /* the fault clears with a sane zero */
    CHECK(supervisor_bias_enable(true));
    CHECK(!(supervisor_poll()->flags & SUP_FLAG_BIAS_SENSE_FAULT));
}

static void a_zero_with_nothing_fitted_or_a_shorted_pin_is_a_sense_fault(void)
{
    supplies(12000, 5480, 5000);
    supervisor_init();
    test_vhw.adc_mv[HAL_ADC_BIAS_ISENSE] = 3300;                     /* the 1 MOhm pull-up alone: no sense circuit fitted */
    CHECK(!supervisor_bias_enable(true));
    CHECK(test_vhw.gpio[HAL_GPIO_BIAS_EN] == false);
    CHECK(supervisor_poll()->flags & SUP_FLAG_BIAS_SENSE_FAULT);
    test_vhw.adc_mv[HAL_ADC_BIAS_ISENSE] = 901;
    CHECK(!supervisor_bias_enable(true));
    test_vhw.adc_mv[HAL_ADC_BIAS_ISENSE] = 900;                      /* mirror zero tolerance still passes */
    CHECK(supervisor_bias_enable(true));
}

static void enabling_again_with_the_feed_on_keeps_the_zero_and_table(void)
{
    supplies(12000, 5480, 5000);
    supervisor_init();
    test_vhw.adc_mv[HAL_ADC_BIAS_ISENSE] = 2;
    CHECK(supervisor_bias_enable(true));
    test_vhw.adc_mv[HAL_ADC_BIAS_ISENSE] = sense_mv_for(100000);     /* 100 mA flowing: reads ≈ 2 V */
    CHECK(supervisor_bias_enable(true));                             /* already on: nothing is re-measured */
    uint32_t ua = supervisor_poll()->bias_ua;
    CHECK(ua > 98000 && ua < 102000);
    CHECK(supervisor_poll()->flags & SUP_FLAG_BIAS_ON);
    CHECK(supervisor_bias_ua(TABLES_BIAS_SENSE_INA180_MV[3]) == TABLES_BIAS_SENSE_INA180_UA[3]);
}

static void a_saturated_ina180_reading_trips_the_cutoff(void)
{
    supplies(12000, 5480, 5000);
    supervisor_init();
    test_vhw.adc_mv[HAL_ADC_BIAS_ISENSE] = 3;
    CHECK(supervisor_bias_enable(true));
    test_vhw.adc_mv[HAL_ADC_BIAS_ISENSE] = TABLES_BIAS_SENSE_INA180_MV[TABLES_BIAS_SENSE_INA180_POINTS - 1];
    const sup_status_t *s = supervisor_poll();                      /* the clamp, 3277 mV */
    CHECK((s->flags & SUP_FLAG_BIAS_TRIPPED) && !(s->flags & SUP_FLAG_BIAS_ON));
    CHECK(s->bias_ua > TABLES_BIAS_CUTOFF_UA);
}

static void the_virtual_board_reads_the_fitted_circuits_zero(void)
{
    vhw_t v;
    vhw_reset(&v);
    CHECK(vhw_population(&v) == VHW_POP_CHINA);                     /* China is the default */
    CHECK(v.adc_mv[HAL_ADC_BIAS_ISENSE] == TABLES_BIAS_SENSE_INA180_MV[0]);
    CHECK(vhw_set_population(&v, "iran") == 0 && vhw_population(&v) == VHW_POP_IRAN);
    CHECK(v.adc_mv[HAL_ADC_BIAS_ISENSE] == TABLES_BIAS_SENSE_MIRROR_MV[0]);
    CHECK(vhw_set_population(&v, "china") == 0 && v.adc_mv[HAL_ADC_BIAS_ISENSE] == TABLES_BIAS_SENSE_INA180_MV[0]);
    CHECK(vhw_set_population(&v, "mars") != 0 && vhw_population(&v) == VHW_POP_CHINA);
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
    RUN(bias_sense_table_follows_the_zero_reading);
    RUN(ina180_reading_is_referred_to_its_own_zero);
    RUN(a_zero_between_the_two_circuits_is_a_sense_fault);
    RUN(a_zero_with_nothing_fitted_or_a_shorted_pin_is_a_sense_fault);
    RUN(enabling_again_with_the_feed_on_keeps_the_zero_and_table);
    RUN(a_saturated_ina180_reading_trips_the_cutoff);
    RUN(the_virtual_board_reads_the_fitted_circuits_zero);
    RUN(temperature_is_read_and_a_missing_sensor_flagged);
    return CHECK_DONE();
}
