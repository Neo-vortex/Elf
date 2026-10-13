/* Si5351C frequency planner: the C port of hardware/neohw/lo_plan.py (the reference it is
 * tested against on the whole 1 kHz grid). Exact rational arithmetic, no floating point.
 *
 * Outputs (Si5351C-B-GM1): CLK0/CLK1 = direct I/Q on PLLA; CLK2 = 4·LO on PLLB for the ÷4
 * path; CLK3 = calibration tone on PLLB. */
#ifndef ELF_LO_PLAN_H
#define ELF_LO_PLAN_H

#include <stdbool.h>
#include <stdint.h>

#define LO_XTAL_HZ 25000000LL
#define LO_VCO_MIN_HZ 600000000LL
#define LO_VCO_MAX_HZ 900000000LL
#define LO_PLL_MULT_MIN 15
#define LO_PLL_MULT_MAX 90
#define LO_DENOM_MAX 1048575LL
#define LO_MS_INT_MIN 6
#define LO_MS_FRAC_MIN 8
#define LO_MS_MAX 1800
#define LO_PHOFF_MAX 127
#define LO_DIRECT_MIN_HZ 4800000UL
#define LO_MIN_HZ 100000UL
#define LO_MAX_HZ 54000000UL

typedef struct {
    int64_t num, den;               /* den > 0, reduced */
} frac_t;

enum { LO_PLL_A = 0, LO_PLL_B = 1 };
enum { LO_PATH_DIRECT = 0, LO_PATH_DIV4 = 1 };

typedef enum {
    LO_OK = 0,
    LO_ERR_RANGE = -1,              /* LO outside 0.1–54 MHz */
    LO_ERR_NO_PLAN = -2,            /* no divider / multiplier combination */
} lo_status_t;

typedef struct {
    bool used;
    uint8_t pll;                    /* LO_PLL_A or LO_PLL_B */
    frac_t ms;                      /* MultiSynth divider */
    uint8_t r_div;                  /* 1, 2, 4 … 128 */
    uint8_t phoff;                  /* CLKx_PHOFF, quarter VCO periods */
    frac_t actual_hz;
} lo_output_t;

typedef struct {
    uint32_t f_lo_hz;
    uint8_t path;                   /* LO_PATH_DIRECT or LO_PATH_DIV4 */
    bool pll_used[2];
    frac_t pll_mult[2];             /* feedback multiplier a + b/c */
    frac_t vco_hz[2];
    lo_output_t out[4];             /* CLK0 … CLK3 */
} lo_plan_t;

/* Plans the LO; with `with_cal` also the calibration tone at f_lo + cal_offset_hz on CLK3. */
lo_status_t lo_plan(uint32_t f_lo_hz, bool with_cal, int32_t cal_offset_hz, lo_plan_t *p);

/* Python's Fraction.limit_denominator, exactly (same candidate and tie-break). */
frac_t frac_limit_denominator(frac_t x, int64_t max_den);
frac_t frac_make(int64_t num, int64_t den);

#endif
