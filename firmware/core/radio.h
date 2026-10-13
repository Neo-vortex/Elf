/* Tuning: LO plan → Si5351, LO path select, filter, attenuator and preamp relays. */
#ifndef ELF_RADIO_H
#define ELF_RADIO_H

#include <stdbool.h>
#include <stdint.h>

#include "lo_plan.h"

#define RADIO_RELAY_SETTLE_MS 5      /* G6K-2F operate time 3 ms max, plus margin */

typedef enum { RADIO_OK = 0, RADIO_ERR_RANGE = -1, RADIO_ERR_PLAN = -2, RADIO_ERR_I2C = -3 } radio_status_t;

typedef struct {
    uint32_t lo_hz;          /* 0 until the first successful tune */
    int filter;              /* 1–11, 0 = muted */
    bool atten, preamp;
    bool cal_tone;
    int32_t cal_offset_hz;
    uint8_t path;            /* LO_PATH_DIRECT or LO_PATH_DIV4 */
} radio_state_t;

void radio_init(void);
/* Filter for an LO: filter n covers lo_n <= f < hi_n, the top edge selects the last; 0 outside. */
int radio_filter_for(uint32_t lo_hz);
radio_status_t radio_tune(uint32_t lo_hz);
radio_status_t radio_set_frontend(bool atten, bool preamp);
/* Calibration tone on CLK3 at LO + offset (off outside calibration: CLK3 powered down). */
radio_status_t radio_set_cal_tone(bool on, int32_t offset_hz);
const radio_state_t *radio_state(void);

#endif
