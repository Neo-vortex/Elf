#include "lo_plan.h"

#include <string.h>

static const uint8_t R_DIVS[] = {1, 2, 4, 8, 16, 32, 64, 128};

static int64_t gcd64(int64_t a, int64_t b)
{
    if (a < 0) a = -a;
    if (b < 0) b = -b;
    while (b) {
        int64_t t = a % b;
        a = b;
        b = t;
    }
    return a;
}

frac_t frac_make(int64_t num, int64_t den)
{
    if (den < 0) {
        num = -num;
        den = -den;
    }
    int64_t g = gcd64(num, den);
    frac_t f = {g ? num / g : 0, g ? den / g : 1};
    return f;
}

static int64_t floordiv(int64_t a, int64_t b)
{
    int64_t q = a / b;
    if ((a % b != 0) && ((a < 0) != (b < 0))) q--;
    return q;
}

frac_t frac_limit_denominator(frac_t x, int64_t max_den)
{
    if (x.den <= max_den) return x;
    int64_t p0 = 0, q0 = 1, p1 = 1, q1 = 0;
    int64_t n = x.num, d = x.den;
    for (;;) {
        int64_t a = floordiv(n, d);
        int64_t q2 = q0 + a * q1;
        if (q2 > max_den) break;
        int64_t p2 = p0 + a * p1;
        p0 = p1;
        q0 = q1;
        p1 = p2;
        q1 = q2;
        int64_t t = n - a * d;
        n = d;
        d = t;
    }
    int64_t k = (max_den - q0) / q1;
    if ((__int128)2 * d * (q0 + k * q1) <= (__int128)x.den) return frac_make(p1, q1);
    return frac_make(p0 + k * p1, q0 + k * q1);
}

/* a <= b for fractions (denominators positive) */
static bool frac_le(frac_t a, frac_t b) { return (__int128)a.num * b.den <= (__int128)b.num * a.den; }
static frac_t frac_int(int64_t v) { return (frac_t){v, 1}; }

static frac_t frac_mul(frac_t a, frac_t b)
{
    int64_t g1 = gcd64(a.num, b.den), g2 = gcd64(b.num, a.den);
    if (!g1) g1 = 1;
    if (!g2) g2 = 1;
    return frac_make((a.num / g1) * (b.num / g2), (a.den / g2) * (b.den / g1));
}

static frac_t frac_div(frac_t a, frac_t b) { return frac_mul(a, (frac_t){b.den, b.num}); }

/* Feedback multiplier for a VCO frequency (20-bit denominator) and the VCO it produces. */
static bool pll_for(frac_t vco, frac_t *mult, frac_t *vco_out)
{
    frac_t m = frac_limit_denominator(frac_div(vco, frac_int(LO_XTAL_HZ)), LO_DENOM_MAX);
    if (!frac_le(frac_int(LO_PLL_MULT_MIN), m) || !frac_le(m, frac_int(LO_PLL_MULT_MAX))) return false;
    *mult = m;
    *vco_out = frac_mul(m, frac_int(LO_XTAL_HZ));
    return true;
}

/* Largest even integer divider N (with an R divider when f is too low) with N·R·f in the VCO
 * range; f is an integer frequency. */
static bool integer_plan(int64_t f, int n_max, int *n_out, int *r_out, int64_t *vco_out)
{
    for (unsigned i = 0; i < sizeof R_DIVS; i++) {
        int64_t r = R_DIVS[i];
        int64_t top = LO_VCO_MAX_HZ / (f * r);
        if (top > n_max) top = n_max;
        int64_t n = top - (top % 2);
        if (n >= LO_MS_INT_MIN && n * r * f >= LO_VCO_MIN_HZ) {
            *n_out = (int)n;
            *r_out = (int)r;
            *vco_out = n * r * f;
            return true;
        }
    }
    return false;
}

static bool fractional_ms(frac_t vco, int64_t f, frac_t *ms_out, int *r_out)
{
    for (unsigned i = 0; i < sizeof R_DIVS; i++) {
        int64_t r = R_DIVS[i];
        frac_t ms = frac_limit_denominator(frac_div(vco, frac_int(f * r)), LO_DENOM_MAX);
        if (frac_le(frac_int(LO_MS_FRAC_MIN), ms) && frac_le(ms, frac_int(LO_MS_MAX))) {
            *ms_out = ms;
            *r_out = (int)r;
            return true;
        }
    }
    return false;
}

static void set_out(lo_plan_t *p, int clk, int pll, frac_t ms, int r, int phoff)
{
    lo_output_t *o = &p->out[clk];
    o->used = true;
    o->pll = (uint8_t)pll;
    o->ms = ms;
    o->r_div = (uint8_t)r;
    o->phoff = (uint8_t)phoff;
    o->actual_hz = frac_div(p->vco_hz[pll], frac_mul(ms, frac_int(r)));
}

lo_status_t lo_plan(uint32_t f_lo_hz, bool with_cal, int32_t cal_offset_hz, lo_plan_t *p)
{
    memset(p, 0, sizeof *p);
    if (f_lo_hz < LO_MIN_HZ || f_lo_hz > LO_MAX_HZ) return LO_ERR_RANGE;
    p->f_lo_hz = f_lo_hz;
    p->path = f_lo_hz >= LO_DIRECT_MIN_HZ ? LO_PATH_DIRECT : LO_PATH_DIV4;
    int n, r;
    int64_t vco;
    if (p->path == LO_PATH_DIRECT) {
        if (!integer_plan(f_lo_hz, LO_PHOFF_MAX, &n, &r, &vco) || r != 1) return LO_ERR_NO_PLAN;
        if (!pll_for(frac_int(vco), &p->pll_mult[LO_PLL_A], &p->vco_hz[LO_PLL_A])) return LO_ERR_NO_PLAN;
        p->pll_used[LO_PLL_A] = true;
        set_out(p, 0, LO_PLL_A, frac_int(n), 1, 0);
        set_out(p, 1, LO_PLL_A, frac_int(n), 1, n);
    } else {
        if (!integer_plan(4LL * f_lo_hz, LO_MS_MAX, &n, &r, &vco)) return LO_ERR_NO_PLAN;
        if (!pll_for(frac_int(vco), &p->pll_mult[LO_PLL_B], &p->vco_hz[LO_PLL_B])) return LO_ERR_NO_PLAN;
        p->pll_used[LO_PLL_B] = true;
        set_out(p, 2, LO_PLL_B, frac_int(n), r, 0);
    }
    if (with_cal) {
        int64_t f_cal = (int64_t)f_lo_hz + cal_offset_hz;
        if (!p->pll_used[LO_PLL_B]) {        /* direct path: PLLB is free for the tone */
            if (!integer_plan(f_cal, LO_MS_MAX, &n, &r, &vco)) return LO_ERR_NO_PLAN;
            if (!pll_for(frac_int(vco), &p->pll_mult[LO_PLL_B], &p->vco_hz[LO_PLL_B])) return LO_ERR_NO_PLAN;
            p->pll_used[LO_PLL_B] = true;
            set_out(p, 3, LO_PLL_B, frac_int(n), r, 0);
        } else {
            frac_t ms;
            if (!fractional_ms(p->vco_hz[LO_PLL_B], f_cal, &ms, &r)) return LO_ERR_NO_PLAN;
            set_out(p, 3, LO_PLL_B, ms, r, 0);
        }
    }
    return LO_OK;
}
