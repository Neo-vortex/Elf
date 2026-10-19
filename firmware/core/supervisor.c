#include "supervisor.h"

#include "hal.h"
#include "tables_gen.h"

static sup_status_t st;
static uint32_t zero_mv = 0;          /* the sense circuit's own reading with the feed off (0: the table's) */

/* the table of the sense circuit fitted: INA180A1 (China) until a zero reading says it is the mirror (Iran) */
static const uint32_t *tab_mv = TABLES_BIAS_SENSE_INA180_MV, *tab_ua = TABLES_BIAS_SENSE_INA180_UA;
static int tab_n = TABLES_BIAS_SENSE_INA180_POINTS;

static void choose_table(uint32_t zero)
{
    bool mirror = zero >= TABLES_BIAS_ZERO_SPLIT_MV;
    tab_mv = mirror ? TABLES_BIAS_SENSE_MIRROR_MV : TABLES_BIAS_SENSE_INA180_MV;
    tab_ua = mirror ? TABLES_BIAS_SENSE_MIRROR_UA : TABLES_BIAS_SENSE_INA180_UA;
    tab_n = mirror ? TABLES_BIAS_SENSE_MIRROR_POINTS : TABLES_BIAS_SENSE_INA180_POINTS;
    if (zero >= SUP_SENSE_FAULT_LO_MV && zero <= SUP_SENSE_FAULT_HI_MV) st.flags |= SUP_FLAG_BIAS_SENSE_FAULT;
    else st.flags &= ~SUP_FLAG_BIAS_SENSE_FAULT;
}

/* the sense voltage referred to the simulated circuit: this board's zero becomes the table's */
static uint32_t referred_mv(uint32_t mv)
{
    if (!zero_mv) return mv;
    int64_t v = (int64_t)mv - zero_mv + tab_mv[0];
    return v < 0 ? 0 : (uint32_t)v;
}

uint32_t supervisor_bias_ua(uint32_t mv)
{
    const uint32_t *x = tab_mv, *y = tab_ua;
    const int n = tab_n;
    if (mv <= x[0]) return 0;
    int k = 1;
    while (k < n - 1 && mv > x[k]) k++;
    /* linear between points k-1 and k (extrapolated beyond the last segment) */
    int64_t num = (int64_t)(mv - x[k - 1]) * (int64_t)(y[k] - y[k - 1]);
    int64_t v = (int64_t)y[k - 1] + num / (int64_t)(x[k] - x[k - 1]);
    return v < 0 ? 0 : (uint32_t)v;
}

void supervisor_init(void)
{
    st = (sup_status_t){0};
    zero_mv = 0;
    choose_table(0);
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

static uint32_t divided(hal_adc_t ch, uint32_t ratio_x1000)
{
    return (uint32_t)((uint64_t)hal_adc_mv(ch) * ratio_x1000 / 1000u);
}

static void bias_off(uint32_t why)
{
    hal_gpio_set(HAL_GPIO_BIAS_EN, false);
    st.flags = (st.flags & ~SUP_FLAG_BIAS_ON) | why;
}

const sup_status_t *supervisor_poll(void)
{
    st.vbus_mv = divided(HAL_ADC_SENSE_VBUS, TABLES_SENSE_DIV_VBUS_X1000);
    st.ext_mv = divided(HAL_ADC_SENSE_EXT, TABLES_SENSE_DIV_EXT_X1000);
    st.main_mv = divided(HAL_ADC_SENSE_MAIN, TABLES_SENSE_DIV_MAIN_X1000);
    st.flags &= SUP_FLAG_BIAS_ON | SUP_FLAG_BIAS_TRIPPED | SUP_FLAG_TEMP_FAIL | SUP_FLAG_BIAS_SENSE_FAULT;
    if (st.ext_mv >= SUP_EXT_PRESENT_MV) st.flags |= SUP_FLAG_EXT;
    if (st.ext_mv < SUP_INPUT_LOW_MV) st.flags |= SUP_FLAG_INPUT_LOW;
    if (st.vbus_mv >= SUP_HOST_MV) st.flags |= SUP_FLAG_HOST;
    if (st.main_mv < SUP_MAIN_OK_MV) st.flags |= SUP_FLAG_MAIN_LOW;
    st.bias_ua = (st.flags & SUP_FLAG_BIAS_ON) ? supervisor_bias_ua(referred_mv(hal_adc_mv(HAL_ADC_BIAS_ISENSE))) : 0;
    if ((st.flags & SUP_FLAG_BIAS_ON) && st.bias_ua > TABLES_BIAS_CUTOFF_UA) bias_off(SUP_FLAG_BIAS_TRIPPED);
    if ((st.flags & SUP_FLAG_BIAS_ON) && (st.flags & SUP_FLAG_MAIN_LOW)) bias_off(0);
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
    if (st.flags & SUP_FLAG_MAIN_LOW) return false;
    zero_mv = hal_adc_mv(HAL_ADC_BIAS_ISENSE);   /* the feed is still off: this board's zero */
    choose_table(zero_mv);
    if (st.flags & SUP_FLAG_BIAS_SENSE_FAULT) return false;
    st.flags &= ~SUP_FLAG_BIAS_TRIPPED;
    st.flags |= SUP_FLAG_BIAS_ON;
    hal_gpio_set(HAL_GPIO_BIAS_EN, true);
    return true;
}
