#include "radio.h"

#include "hal.h"
#include "relays.h"
#include "si5351.h"
#include "tables_gen.h"

static radio_state_t st;

void radio_init(void)
{
    st = (radio_state_t){0};
    relays_init();
    hal_gpio_set(HAL_GPIO_LO_PATH, false);
}

int radio_filter_for(uint32_t lo_hz)
{
    for (int k = 0; k < TABLES_FILTER_COUNT; k++)
        if (TABLES_FILTER_LO_HZ[k] <= lo_hz && lo_hz < TABLES_FILTER_HI_HZ[k]) return k + 1;
    if (lo_hz == TABLES_FILTER_HI_HZ[TABLES_FILTER_COUNT - 1]) return TABLES_FILTER_COUNT;
    return 0;
}

static radio_status_t apply(uint32_t lo_hz, bool cal, int32_t cal_offset)
{
    lo_plan_t p;
    lo_status_t s = lo_plan(lo_hz, cal, cal_offset, &p);
    if (s == LO_ERR_RANGE) return RADIO_ERR_RANGE;
    if (s != LO_OK) return RADIO_ERR_PLAN;
    uint8_t reg[SI5351_REG_COUNT];
    si5351_image(&p, reg);
    if (si5351_program(reg)) return RADIO_ERR_I2C;
    hal_gpio_set(HAL_GPIO_LO_PATH, p.path == LO_PATH_DIV4);
    st.lo_hz = lo_hz;
    st.path = p.path;
    st.cal_tone = cal;
    st.cal_offset_hz = cal_offset;
    int filter = radio_filter_for(lo_hz);
    if (filter != st.filter) {
        st.filter = filter;
        relays_write(relays_word(st.filter, st.atten, st.preamp));
        hal_delay_ms(RADIO_RELAY_SETTLE_MS);
    }
    return RADIO_OK;
}

radio_status_t radio_tune(uint32_t lo_hz) { return apply(lo_hz, st.cal_tone, st.cal_offset_hz); }

radio_status_t radio_set_frontend(bool atten, bool preamp)
{
    st.atten = atten;
    st.preamp = preamp;
    relays_write(relays_word(st.filter, st.atten, st.preamp));
    hal_delay_ms(RADIO_RELAY_SETTLE_MS);
    return RADIO_OK;
}

radio_status_t radio_set_cal_tone(bool on, int32_t offset_hz)
{
    if (!st.lo_hz) return RADIO_ERR_RANGE;
    return apply(st.lo_hz, on, offset_hz);
}

const radio_state_t *radio_state(void) { return &st; }
