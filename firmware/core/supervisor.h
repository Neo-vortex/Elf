/* Power and protection supervision (phase 3 spec §3.2, plan 3.4: the 9–30 V jack is the only
 * supply, USB is data only): input and 5V_MAIN monitoring, host presence on VBUS, the bias-T
 * over-current cut-off (hardware limit 0.39 A, firmware 150 mA) and board temperature. */
#ifndef ELF_SUPERVISOR_H
#define ELF_SUPERVISOR_H

#include <stdbool.h>
#include <stdint.h>

#define SUP_EXT_PRESENT_MV 3000u       /* any input on the jack side of the board */
#define SUP_INPUT_LOW_MV 8000u         /* after the fuse, NTC and SS36: a 9 V adapter sagging below ≈ 8.5 V */
#define SUP_MAIN_OK_MV 5300u           /* 5V_MAIN in regulation (5.5 V nominal) */
#define SUP_HOST_MV 4000u              /* VBUS from a USB host */

enum {
    SUP_FLAG_EXT = 1u << 0,            /* the jack supplies the board */
    SUP_FLAG_INPUT_LOW = 1u << 1,      /* input below the 9 V specification */
    SUP_FLAG_BIAS_ON = 1u << 2,
    SUP_FLAG_BIAS_TRIPPED = 1u << 3,   /* cut off for over-current; cleared by the next enable */
    SUP_FLAG_TEMP_FAIL = 1u << 4,      /* LM75A not answering */
    SUP_FLAG_HOST = 1u << 5,           /* a USB host is connected (VBUS present) */
    SUP_FLAG_MAIN_LOW = 1u << 6,       /* 5V_MAIN out of regulation */
};

typedef struct {
    uint32_t vbus_mv, ext_mv, main_mv;
    uint32_t bias_ua;
    int32_t temp_mc;                   /* milli-°C */
    uint32_t flags;
} sup_status_t;

void supervisor_init(void);
/* Reads every monitor and applies the bias-T cut-offs (over-current, 5V_MAIN out of regulation);
 * call every few ms. */
const sup_status_t *supervisor_poll(void);
/* Bias-T on only with 5V_MAIN in regulation; returns false when refused. */
bool supervisor_bias_enable(bool on);
/* Current from the sense voltage (piecewise-linear table of the simulated mirror). Readings are
 * first referred to the table: this board's zero, measured with the feed off before each enable,
 * is subtracted and the table's zero added. */
uint32_t supervisor_bias_ua(uint32_t sense_mv);

#endif
