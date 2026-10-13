#include "check.h"
#include "lo_plan.h"
#include "test_hal.h"

/* actual_hz must be exact for any Hz the CAT accepts, not only the 1 kHz grid (the ÷4 path with
 * a cal tone off the grid gives a non-integer VCO and products near 10^21) */
static void off_grid_cal_plans_keep_exact_frequencies(void)
{
    const uint32_t los[] = {4781131, 1369097, 100001, 4799999, 3550123};
    const int32_t cals[] = {10000, 12345, 1, 999999};
    for (unsigned i = 0; i < sizeof los / sizeof los[0]; i++)
        for (unsigned j = 0; j < sizeof cals / sizeof cals[0]; j++) {
            lo_plan_t p;
            if (lo_plan(los[i], true, cals[j], &p) != LO_OK) continue;
            double cal = (double)p.out[3].actual_hz.num / (double)p.out[3].actual_hz.den;
            CHECK(p.out[3].actual_hz.den > 0);
            CHECK(cal > (double)los[i] + cals[j] - 1.0 && cal < (double)los[i] + cals[j] + 1.0);
        }
}

/* a prime step lands off the grid everywhere: any overflow fails the test under UBSan */
static void off_grid_sweep_has_no_undefined_behaviour(void)
{
    const int32_t cals[] = {10000, 12345, -7777};
    unsigned planned = 0;
    for (uint32_t f = LO_MIN_HZ; f <= LO_MAX_HZ; f += 997)
        for (unsigned j = 0; j < 3; j++) {
            lo_plan_t p;
            planned += lo_plan(f, true, cals[j], &p) == LO_OK;
        }
    CHECK(planned > 3 * 50000);
}

int main(void)
{
    RUN(off_grid_sweep_has_no_undefined_behaviour);
    RUN(off_grid_cal_plans_keep_exact_frequencies);
    return CHECK_DONE();
}
