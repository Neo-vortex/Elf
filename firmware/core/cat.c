#include "cat.h"

#include <inttypes.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "radio.h"
#include "supervisor.h"

#define CAT_MAX 64

static cat_out_fn out;
static char buf[CAT_MAX];
static size_t len;
static bool overflow;
static uint32_t vfo_b;
static int mode = 2;              /* USB */

void cat_init(cat_out_fn fn)
{
    out = fn;
    len = 0;
    overflow = false;
    vfo_b = 0;
    mode = 2;
}

static void say(const char *s) { out(s, strlen(s)); }

static void sayf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void sayf(const char *fmt, ...)
{
    char s[CAT_MAX];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(s, sizeof s, fmt, ap);
    va_end(ap);
    say(s);
}

/* digits only, 1..max characters */
static bool number(const char *s, size_t n, size_t max, uint32_t *v)
{
    if (n == 0 || n > max) return false;
    uint64_t acc = 0;
    for (size_t k = 0; k < n; k++) {
        if (s[k] < '0' || s[k] > '9') return false;
        acc = acc * 10 + (uint64_t)(s[k] - '0');
    }
    if (acc > UINT32_MAX) return false;
    *v = (uint32_t)acc;
    return true;
}

static void command(const char *c, size_t n)
{
    const radio_state_t *r = radio_state();
    const char *arg = c + 2;
    size_t an = n - 2;
    uint32_t v;
    if (n < 2) { say("?;"); return; }
    if (!strncmp(c, "FA", 2)) {
        if (an == 0) { sayf("FA%011" PRIu32 ";", r->lo_hz); return; }
        if (!number(arg, an, 11, &v) || radio_tune(v) != RADIO_OK) { say("?;"); return; }
        return;
    }
    if (!strncmp(c, "FB", 2)) {
        if (an == 0) { sayf("FB%011" PRIu32 ";", vfo_b); return; }
        if (!number(arg, an, 11, &v)) { say("?;"); return; }
        vfo_b = v;
        return;
    }
    if (!strncmp(c, "IF", 2) && an == 0) {
        /* P1 freq(11) P2 step(5) P3 RIT(5) P4 RIT P5 XIT P6-7 memory(3) P8 RX P9 mode P10 VFO
         * P11 scan P12 split P13 tone P14 tone no.(2) P15 shift */
        sayf("IF%011" PRIu32 "     +0000000000%d0000000;", r->lo_hz, mode);
        return;
    }
    if (!strncmp(c, "ID", 2) && an == 0) { say("ID019;"); return; }
    if (!strncmp(c, "MD", 2)) {
        if (an == 0) { sayf("MD%d;", mode); return; }
        if (!number(arg, an, 1, &v) || v < 1 || v > 9) { say("?;"); return; }
        mode = (int)v;
        return;
    }
    if (!strncmp(c, "PA", 2)) {
        if (an == 0) { sayf("PA%d0;", r->preamp ? 1 : 0); return; }
        if (!number(arg, an, 1, &v) || v > 1) { say("?;"); return; }
        radio_set_frontend(r->atten, v == 1);
        return;
    }
    if (!strncmp(c, "RA", 2)) {
        if (an == 0) { sayf("RA%02d00;", r->atten ? 1 : 0); return; }
        if (!number(arg, an, 2, &v) || v > 1) { say("?;"); return; }
        radio_set_frontend(v == 1, r->preamp);
        return;
    }
    if (!strncmp(c, "AI", 2)) { if (an == 0) say("AI0;"); return; }
    if (!strncmp(c, "PS", 2)) { if (an == 0) say("PS1;"); return; }
    if (!strncmp(c, "RX", 2) && an == 0) { say("RX0;"); return; }
    if (!strncmp(c, "ZB", 2)) {
        const sup_status_t *s = supervisor_poll();
        if (an == 0) { sayf("ZB%d;", (s->flags & SUP_FLAG_BIAS_ON) ? 1 : 0); return; }
        if (!number(arg, an, 1, &v) || v > 1 || !supervisor_bias_enable(v == 1)) { say("?;"); return; }
        return;
    }
    if (!strncmp(c, "ZT", 2) && an == 0) {
        const sup_status_t *s = supervisor_poll();
        sayf("ZT%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRId32 ",%" PRIu32 ";", s->vbus_mv, s->ext_mv,
             s->main_mv, s->bias_ua, s->temp_mc, s->flags);
        return;
    }
    if (!strncmp(c, "ZC", 2)) {
        if (an == 0) { sayf("ZC%d%" PRId32 ";", r->cal_tone ? 1 : 0, r->cal_offset_hz); return; }
        if (arg[0] == '0' && an == 1) { radio_set_cal_tone(false, 0); return; }
        if (arg[0] == '1' && number(arg + 1, an - 1, 6, &v) && radio_set_cal_tone(true, (int32_t)v) == RADIO_OK) return;
        say("?;");
        return;
    }
    say("?;");
}

void cat_input(const uint8_t *data, size_t n)
{
    for (size_t k = 0; k < n; k++) {
        char ch = (char)data[k];
        if (ch == '\r' || ch == '\n') continue;
        if (ch == ';') {
            if (overflow) say("?;");
            else command(buf, len);
            len = 0;
            overflow = false;
            continue;
        }
        if (len < CAT_MAX - 1) buf[len++] = ch;
        else overflow = true;
    }
}
