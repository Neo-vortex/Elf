/* Dumps the plan for every LO on a grid, one line each, in the format of
 * tests/lo_plan_reference.py, so the two can be compared byte for byte. */
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

#include "lo_plan.h"

static void frac(const frac_t *f) { printf(" %" PRId64 "/%" PRId64, f->num, f->den); }

int main(int argc, char **argv)
{
    if (argc != 5) {
        fprintf(stderr, "usage: lo_plan_dump START STOP STEP CAL_OFFSET\n");
        return 2;
    }
    uint32_t start = (uint32_t)strtoul(argv[1], 0, 10), stop = (uint32_t)strtoul(argv[2], 0, 10);
    uint32_t step = (uint32_t)strtoul(argv[3], 0, 10);
    int32_t cal = (int32_t)strtol(argv[4], 0, 10);
    for (uint32_t f = start; f <= stop; f += step) {
        lo_plan_t p;
        lo_status_t s = lo_plan(f, true, cal, &p);
        printf("%" PRIu32, f);
        if (s != LO_OK) {
            printf(" error\n");
            continue;
        }
        printf(" %s", p.path == LO_PATH_DIRECT ? "direct" : "div4");
        for (int k = 0; k < 2; k++) {
            if (p.pll_used[k]) frac(&p.pll_mult[k]);
            else printf(" -");
        }
        for (int c = 0; c < 4; c++) {
            const lo_output_t *o = &p.out[c];
            if (!o->used) { printf(" -"); continue; }
            printf(" %c", o->pll == LO_PLL_A ? 'A' : 'B');
            frac(&o->ms);
            printf(" %u %u", o->r_div, o->phoff);
        }
        printf("\n");
    }
    return 0;
}
