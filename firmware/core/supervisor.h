/* Power and protection supervision: input source, USB-C current advertisement, rails, bias-T
 * over-current cut-off (hardware limit 0.39 A, firmware 150 mA) and board temperature. */
#ifndef ELF_SUPERVISOR_H
#define ELF_SUPERVISOR_H

#include <stdbool.h>
#include <stdint.h>

#define SUP_DIVIDER 2u                 /* 100 k / 100 k */
#define SUP_EXT_PRESENT_MV 4000u
#define SUP_VBUS_LOW_MV 4750u
/* USB-C sink with 5.1 kΩ Rd: CC voltage per source advertisement (USB Type-C spec, vRd ranges) */
#define SUP_CC_1A5_MIN_MV 660u
#define SUP_CC_3A_MIN_MV 1230u

typedef enum { SUP_CC_NONE, SUP_CC_DEFAULT, SUP_CC_1A5, SUP_CC_3A } sup_cc_t;

enum {
    SUP_FLAG_EXT = 1u << 0,            /* external supply in use */
    SUP_FLAG_VBUS_LOW = 1u << 1,       /* USB below 4.75 V */
    SUP_FLAG_BIAS_ON = 1u << 2,
    SUP_FLAG_BIAS_TRIPPED = 1u << 3,   /* cut off for over-current; cleared by the next enable */
    SUP_FLAG_TEMP_FAIL = 1u << 4,      /* LM75A not answering */
};

typedef struct {
    uint32_t vbus_mv, ext_mv, main_mv;
    uint32_t bias_ua;
    int32_t temp_mc;                   /* milli-°C */
    sup_cc_t cc;
    uint32_t flags;
} sup_status_t;

void supervisor_init(void);
/* Reads every monitor and applies the bias-T cut-off; call every few ms. */
const sup_status_t *supervisor_poll(void);
/* Bias-T on only with the external supply or a USB-C source offering ≥ 1.5 A (an active antenna
 * takes up to 100 mA beyond the USB 2.0 budget); returns false when refused. */
bool supervisor_bias_enable(bool on);
/* Current from the sense voltage (piecewise-linear table of the simulated mirror). */
uint32_t supervisor_bias_ua(uint32_t sense_mv);
sup_cc_t supervisor_cc_from_mv(uint32_t cc_mv);

#endif
