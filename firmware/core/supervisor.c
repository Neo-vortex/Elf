#include "supervisor.h"

#include "hal.h"
#include "tables_gen.h"

static sup_status_t st;

uint32_t supervisor_bias_ua(uint32_t mv)
{
    const uint32_t *x = TABLES_BIAS_SENSE_MV, *y = TABLES_BIAS_SENSE_UA;
    const int n = TABLES_BIAS_SENSE_POINTS;
    if (mv <= x[0]) return 0;
    int k = 1;
    while (k < n - 1 && mv > x[k]) k++;
    /* linear between points k-1 and k (extrapolated beyond the last segment) */
    int64_t num = (int64_t)(mv - x[k - 1]) * (int64_t)(y[k] - y[k - 1]);
    int64_t v = (int64_t)y[k - 1] + num / (int64_t)(x[k] - x[k - 1]);
    return v < 0 ? 0 : (uint32_t)v;
}

sup_cc_t supervisor_cc_from_mv(uint32_t mv)
{
    if (mv < 200) return SUP_CC_NONE;
    if (mv < SUP_CC_1A5_MIN_MV) return SUP_CC_DEFAULT;
    if (mv < SUP_CC_3A_MIN_MV) return SUP_CC_1A5;
    return SUP_CC_3A;
}

void supervisor_init(void)
{
    st = (sup_status_t){0};
    hal_gpio_set(HAL_GPIO_BIAS_EN, false);
}

static void read_temp(void)
{
    uint8_t b[2];
    if (hal_i2c_read(TABLES_I2C_LM75A, 0x00, b, 2)) {
        st.flags |= SUP_FLAG_TEMP_FAIL;
        return;
    }
    st.flags &= ~SUP_FLAG_TEMP_FAIL;
    int16_t raw = (int16_t)((b[0] << 8) | b[1]) >> 5;       /* 11-bit, 0.125 °C per LSB */
    st.temp_mc = raw * 125;
}

const sup_status_t *supervisor_poll(void)
{
    st.vbus_mv = hal_adc_mv(HAL_ADC_SENSE_VBUS) * SUP_DIVIDER;
    st.ext_mv = hal_adc_mv(HAL_ADC_SENSE_EXT) * SUP_DIVIDER;
    st.main_mv = hal_adc_mv(HAL_ADC_SENSE_MAIN) * SUP_DIVIDER;
    uint32_t cc1 = hal_adc_mv(HAL_ADC_CC1), cc2 = hal_adc_mv(HAL_ADC_CC2);
    st.cc = supervisor_cc_from_mv(cc1 > cc2 ? cc1 : cc2);
    st.flags = (st.flags & (SUP_FLAG_BIAS_ON | SUP_FLAG_BIAS_TRIPPED | SUP_FLAG_TEMP_FAIL));
    if (st.ext_mv >= SUP_EXT_PRESENT_MV) st.flags |= SUP_FLAG_EXT;
    else if (st.vbus_mv < SUP_VBUS_LOW_MV) st.flags |= SUP_FLAG_VBUS_LOW;
    st.bias_ua = (st.flags & SUP_FLAG_BIAS_ON) ? supervisor_bias_ua(hal_adc_mv(HAL_ADC_BIAS_ISENSE)) : 0;
    if ((st.flags & SUP_FLAG_BIAS_ON) && st.bias_ua > TABLES_BIAS_CUTOFF_UA) {
        hal_gpio_set(HAL_GPIO_BIAS_EN, false);
        st.flags = (st.flags & ~SUP_FLAG_BIAS_ON) | SUP_FLAG_BIAS_TRIPPED;
    }
    read_temp();
    return &st;
}

bool supervisor_bias_enable(bool on)
{
    if (!on) {
        hal_gpio_set(HAL_GPIO_BIAS_EN, false);
        st.flags &= ~SUP_FLAG_BIAS_ON;
        return true;
    }
    supervisor_poll();
    if (!(st.flags & SUP_FLAG_EXT) && st.cc != SUP_CC_1A5 && st.cc != SUP_CC_3A) return false;
    st.flags &= ~SUP_FLAG_BIAS_TRIPPED;
    st.flags |= SUP_FLAG_BIAS_ON;
    hal_gpio_set(HAL_GPIO_BIAS_EN, true);
    return true;
}
