# Phase 4.1: Firmware Core Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The firmware's hardware-independent modules in C (LO planner, Si5351 registers, relay chain, radio, supervisor, CAT, audio ring), unit-tested on Linux against pin-level models of the board's parts, and cross-checked against the design's Python references.

**Architecture:** `firmware/core` holds one `.c/.h` pair per module behind `hal.h`; `tools/gen_tables.py` generates the filter, relay, GPIO and sense tables from `hardware/characterized/` at build time. `host/vhw.c` models the Si5351C register file, the 74HC595 chain (shifting on SRCLK edges), the LM75A and the ADC inputs; the C tests run the modules against it. Python tests compare the C planner with `lo_plan.py` on the whole grid and decode every register image with an AN619 decoder written separately from the encoder (`sim/neosim/si5351_decode.py`).

**Tech Stack:** C11, CMake ≥ 3.20 + Ninja, gcc, Python 3 (pytest, numpy), the existing `neohw`/`neosim` packages.

**Spec:** `docs/superpowers/specs/2026-10-13-phase4-firmware-virtual-board-design.md` (§1 items 1–2, §2, §6).

## Global Constraints

- C11, `-Wall -Wextra -Werror -Wshadow -Wconversion`; no ESP-IDF headers in `core/`.
- Hardware access only through `core/hal.h`.
- Tables come from `hardware/characterized/` (bpf_bank.json, controller.json) at build time; no hand-copied numbers.
- The LO planner must equal `hardware/neohw/lo_plan.py` exactly (same rational approximations, same tie-breaks).
- Outside calibration CLK3 is powered down (phase 3 spec §3.1 firmware contract).
- Power-up: relay outputs enabled only after an all-released word is latched; bias-T off; the bias-T is enabled only with the jack or a ≥ 1.5 A USB-C source; cut-off at 150 mA.

## Review Focus

- Planner equality on the whole 1 kHz grid, including `limit_denominator` tie-breaks (test: `test_c_planner_equals_the_python_planner_on_the_whole_grid`).
- Register images decoded independently (test: `test_register_images_decode_to_the_requested_lo_and_tone`) and the AN619 byte layout checked by hand-computed values (`params_follow_an619`).
- Relay bit order end to end through the 74HC595 model (`write_lands_every_bit_on_its_own_output`).
- Bias-T protection: refusal without power, trip at 150 mA, re-enable clears the trip (`test_supervisor.c`).
- CAT parser robustness: split input, CR/LF, overlong commands, invalid numbers (`test_cat.c`).

---

### Task 1: Firmware core library and its unit tests

**Files:**
- Create: `firmware/CMakeLists.txt`, `firmware/tools/gen_tables.py`, `firmware/core/{hal.h,lo_plan.{c,h},si5351.{c,h},relays.{c,h},radio.{c,h},supervisor.{c,h},cat.{c,h},audio.{c,h}}`, `firmware/host/vhw.{c,h}`, `firmware/tests/{check.h,test_hal.{c,h}}`
- Test: `firmware/tests/test_{si5351,relays,radio,supervisor,cat,audio}.c`
- Modify: `Makefile` (`fw`, `fw-test`)

**Interfaces:**
- Produces: `lo_plan(f, with_cal, cal_offset, *plan)`, `frac_limit_denominator`, `si5351_image(plan, reg[256])`, `si5351_params`, `si5351_program(reg)`, `relays_word(filter, atten, preamp)`, `relays_write`, `relays_init`, `radio_init/tune/set_frontend/set_cal_tone/state/filter_for`, `supervisor_init/poll/bias_enable/bias_ua/cc_from_mv`, `cat_init(out)/cat_input`, `audio_init/pump/packet/stats`, `vhw_*`; CMake targets `elf_core`, `elf_vhw`; the generated `tables_gen.h` names (`TABLES_FILTER_*`, `RELAY_BIT_*`, `GPIO_PAD_*`, `TABLES_BIAS_*`, `TABLES_I2C_*`).

- [ ] **Step 1: Write the failing tests**

`firmware/tests/check.h`:

```c
/* Minimal unit-test harness: CHECK records a failure and continues; RUN calls a test. */
#ifndef ELF_CHECK_H
#define ELF_CHECK_H

#include <stdio.h>

static int check_failures;

#define CHECK(cond)                                                                   \
    do {                                                                              \
        if (!(cond)) {                                                                \
            fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond);  \
            check_failures++;                                                         \
        }                                                                             \
    } while (0)

#define RUN(test)                    \
    do {                             \
        test_hal_reset();            \
        test();                      \
    } while (0)

#define CHECK_DONE() (check_failures ? (fprintf(stderr, "%d failure(s)\n", check_failures), 1) : 0)

#endif
```

`firmware/tests/test_hal.h`:

```c
/* hal.h over the virtual hardware, with manual time and a scripted I2S source. */
#ifndef ELF_TEST_HAL_H
#define ELF_TEST_HAL_H

#include "vhw.h"

extern vhw_t test_vhw;
extern uint32_t test_now_ms;

void test_hal_reset(void);
/* Queues frames the next hal_i2s_read calls return (interleaved I, Q). */
void test_i2s_queue(const int32_t *iq, size_t frames);

#endif
```

`firmware/tests/test_hal.c`:

```c
#include "test_hal.h"

#include <string.h>

vhw_t test_vhw;
uint32_t test_now_ms;
static int32_t i2s[8192][2];
static size_t i2s_head, i2s_tail;

void test_hal_reset(void)
{
    vhw_reset(&test_vhw);
    test_now_ms = 0;
    i2s_head = i2s_tail = 0;
}

void test_i2s_queue(const int32_t *iq, size_t frames)
{
    for (size_t k = 0; k < frames && i2s_head < 8192; k++, i2s_head++) {
        i2s[i2s_head][0] = iq[2 * k];
        i2s[i2s_head][1] = iq[2 * k + 1];
    }
}

int hal_i2c_write(uint8_t addr, uint8_t reg, const uint8_t *data, size_t len)
{
    return vhw_i2c_write(&test_vhw, addr, reg, data, len);
}

int hal_i2c_read(uint8_t addr, uint8_t reg, uint8_t *data, size_t len)
{
    return vhw_i2c_read(&test_vhw, addr, reg, data, len);
}

void hal_gpio_set(hal_gpio_t pin, bool level) { vhw_gpio_set(&test_vhw, pin, level); }
bool hal_gpio_get(hal_gpio_t pin) { return test_vhw.gpio[pin]; }
uint32_t hal_adc_mv(hal_adc_t ch) { return test_vhw.adc_mv[ch]; }

size_t hal_i2s_read(int32_t *iq, size_t max_frames)
{
    size_t n = 0;
    while (n < max_frames && i2s_tail < i2s_head) {
        iq[2 * n] = i2s[i2s_tail][0];
        iq[2 * n + 1] = i2s[i2s_tail][1];
        n++;
        i2s_tail++;
    }
    return n;
}

uint32_t hal_millis(void) { return test_now_ms; }
void hal_delay_ms(uint32_t ms) { test_now_ms += ms; }
```

`firmware/tests/test_si5351.c`:

```c
#include <string.h>

#include "check.h"
#include "si5351.h"
#include "test_hal.h"

static void params_follow_an619(void)
{
    uint8_t b[8];
    /* 36 exactly: P1 = 128·36 − 512 = 4096, P2 = 0, P3 = 1 */
    si5351_params((frac_t){36, 1}, 1, b);
    const uint8_t want_int[8] = {0x00, 0x01, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00};
    CHECK(memcmp(b, want_int, 8) == 0);
    /* 35 + 5/7, R = 8: floor(128·5/7) = 91; P1 = 4480 + 91 − 512 = 4059; P2 = 640 − 637 = 3; P3 = 7 */
    si5351_params((frac_t){250, 7}, 8, b);
    const uint8_t want_frac[8] = {0x00, 0x07, 0x30, 0x0F, 0xDB, 0x00, 0x00, 0x03};
    CHECK(memcmp(b, want_frac, 8) == 0);
    /* 20-bit P3 and P2 high nibbles share byte 5 */
    si5351_params((frac_t){15LL * 1048575 + 1, 1048575}, 1, b);
    CHECK(b[5] == 0xF0 && b[0] == 0xFF && b[1] == 0xFF);
}

static void integer_outputs_set_ms_int_and_fractional_ones_do_not(void)
{
    lo_plan_t p;
    uint8_t reg[SI5351_REG_COUNT];
    CHECK(lo_plan(3550000, true, 10000, &p) == LO_OK);   /* ÷4 path: CLK2 integer, CLK3 fractional */
    CHECK(p.out[2].ms.den == 1 && p.out[3].ms.den != 1);
    si5351_image(&p, reg);
    CHECK(reg[SI5351_REG_CLK_CTRL + 2] & SI5351_CLK_INT);
    CHECK(!(reg[SI5351_REG_CLK_CTRL + 3] & SI5351_CLK_INT));
    CHECK(reg[SI5351_REG_CLK_CTRL + 2] & SI5351_CLK_SRC_PLLB);
    CHECK((reg[SI5351_REG_MS0 + 8 * 2 + 2] >> 4) == __builtin_ctz(p.out[2].r_div));
}

static void unused_outputs_are_powered_down_and_disabled(void)
{
    lo_plan_t p;
    uint8_t reg[SI5351_REG_COUNT];
    CHECK(lo_plan(14050000, false, 0, &p) == LO_OK);
    si5351_image(&p, reg);
    CHECK(reg[SI5351_REG_OE] == 0xFC);                   /* CLK0, CLK1 on */
    for (int k = 2; k < 8; k++) CHECK(reg[SI5351_REG_CLK_CTRL + k] == SI5351_CLK_PDN);
    CHECK(reg[SI5351_REG_CLK_CTRL] == (SI5351_CLK_INT | SI5351_CLK_SRC_MS | SI5351_CLK_DRIVE_8MA));
    CHECK(reg[SI5351_REG_PHOFF + 1] == (uint8_t)p.out[0].ms.num && reg[SI5351_REG_PHOFF] == 0);
    CHECK(reg[SI5351_REG_XTAL_LOAD] == 0xD2 && reg[SI5351_REG_PLL_SRC] == 0);
}

static void program_turns_outputs_off_first_and_on_last(void)
{
    lo_plan_t p;
    uint8_t reg[SI5351_REG_COUNT];
    lo_plan(14050000, false, 0, &p);
    si5351_image(&p, reg);
    CHECK(si5351_program(reg) == 0);
    CHECK(test_vhw.si5351[SI5351_REG_OE] == 0xFC);
    CHECK(memcmp(&test_vhw.si5351[SI5351_REG_PLLA], &reg[SI5351_REG_PLLA], 48) == 0);
    test_vhw.si5351_present = false;
    CHECK(si5351_program(reg) != 0);
}

int main(void)
{
    RUN(params_follow_an619);
    RUN(integer_outputs_set_ms_int_and_fractional_ones_do_not);
    RUN(unused_outputs_are_powered_down_and_disabled);
    RUN(program_turns_outputs_off_first_and_on_last);
    return CHECK_DONE();
}
```

`firmware/tests/test_relays.c`:

```c
#include "check.h"
#include "relays.h"
#include "tables_gen.h"
#include "test_hal.h"

static void word_selects_both_relays_of_one_filter(void)
{
    CHECK(relays_word(7, false, false) == ((1u << RELAY_BIT_BANK7_IN) | (1u << RELAY_BIT_BANK7_OUT)));
    CHECK(relays_word(1, true, true) == (0x3u | (1u << RELAY_BIT_ATTEN) | (1u << RELAY_BIT_PREAMP)));
    CHECK(relays_word(0, false, false) == 0);           /* muted bank */
    CHECK(relays_word(12, false, false) == 0);          /* no such filter: muted, not a stray bit */
}

static void write_lands_every_bit_on_its_own_output(void)
{
    for (int k = 0; k < TABLES_RELAY_COUNT; k++) {
        relays_write(1u << k);
        CHECK(vhw_relay_outputs(&test_vhw) == (1u << k));
    }
    relays_write(0xA5C3E1u);
    CHECK(vhw_relay_outputs(&test_vhw) == 0xA5C3E1u);
}

static void init_enables_the_outputs_only_after_latching_all_released(void)
{
    test_vhw.chain_latched = 0xFFFFFFu;                 /* random power-up storage contents */
    relays_init();
    CHECK(vhw_relay_outputs(&test_vhw) == 0);
    CHECK(test_vhw.gpio[HAL_GPIO_SR_OE] == false);
}

int main(void)
{
    RUN(word_selects_both_relays_of_one_filter);
    RUN(write_lands_every_bit_on_its_own_output);
    RUN(init_enables_the_outputs_only_after_latching_all_released);
    return CHECK_DONE();
}
```

`firmware/tests/test_radio.c`:

```c
#include <string.h>

#include "check.h"
#include "radio.h"
#include "relays.h"
#include "si5351.h"
#include "test_hal.h"

static void filter_edges_follow_the_bank_table(void)
{
    CHECK(radio_filter_for(100000) == 1);
    CHECK(radio_filter_for(499999) == 1);
    CHECK(radio_filter_for(500000) == 2);
    CHECK(radio_filter_for(14050000) == 7);
    CHECK(radio_filter_for(54000000) == 11);
    CHECK(radio_filter_for(99999) == 0);
    CHECK(radio_filter_for(54000001) == 0);
}

static void tune_direct_programs_the_si5351_path_and_filter(void)
{
    radio_init();
    CHECK(radio_tune(14050000) == RADIO_OK);
    lo_plan_t p;
    uint8_t want[SI5351_REG_COUNT];
    CHECK(lo_plan(14050000, false, 0, &p) == LO_OK);
    si5351_image(&p, want);
    CHECK(memcmp(&test_vhw.si5351[SI5351_REG_PLLA], &want[SI5351_REG_PLLA], 48) == 0);
    CHECK(test_vhw.si5351[SI5351_REG_OE] == want[SI5351_REG_OE]);
    CHECK(test_vhw.si5351[SI5351_REG_PHOFF + 1] == p.out[1].phoff);
    CHECK(test_vhw.gpio[HAL_GPIO_LO_PATH] == false);
    CHECK(vhw_relay_outputs(&test_vhw) == relays_word(7, false, false));
    CHECK(radio_state()->lo_hz == 14050000 && radio_state()->path == LO_PATH_DIRECT);
}

static void tune_low_selects_the_divide_by_four_path(void)
{
    radio_init();
    CHECK(radio_tune(3550000) == RADIO_OK);
    CHECK(test_vhw.gpio[HAL_GPIO_LO_PATH] == true);
    CHECK((test_vhw.si5351[SI5351_REG_OE] & 0x04) == 0);        /* CLK2 on */
    CHECK((test_vhw.si5351[SI5351_REG_OE] & 0x03) == 0x03);     /* CLK0/1 off */
    CHECK(vhw_relay_outputs(&test_vhw) == relays_word(4, false, false));
}

static void out_of_range_leaves_the_radio_as_it_was(void)
{
    radio_init();
    radio_tune(7030000);
    uint32_t before = vhw_relay_outputs(&test_vhw);
    CHECK(radio_tune(60000000) == RADIO_ERR_RANGE);
    CHECK(radio_tune(50000) == RADIO_ERR_RANGE);
    CHECK(radio_state()->lo_hz == 7030000 && vhw_relay_outputs(&test_vhw) == before);
}

static void missing_si5351_is_an_i2c_error(void)
{
    radio_init();
    test_vhw.si5351_present = false;
    CHECK(radio_tune(14050000) == RADIO_ERR_I2C);
    CHECK(radio_state()->lo_hz == 0);
}

static void frontend_keeps_the_filter(void)
{
    radio_init();
    radio_tune(21050000);
    radio_set_frontend(true, true);
    CHECK(vhw_relay_outputs(&test_vhw) == relays_word(8, true, true));
    uint32_t t0 = test_now_ms;
    radio_set_frontend(false, true);
    CHECK(test_now_ms - t0 >= RADIO_RELAY_SETTLE_MS);           /* waits for the relays */
}

static void cal_tone_runs_clk3_only_while_on(void)
{
    radio_init();
    radio_tune(14050000);
    CHECK(test_vhw.si5351[SI5351_REG_CLK_CTRL + 3] & SI5351_CLK_PDN);
    CHECK(radio_set_cal_tone(true, 10000) == RADIO_OK);
    CHECK(!(test_vhw.si5351[SI5351_REG_CLK_CTRL + 3] & SI5351_CLK_PDN));
    CHECK((test_vhw.si5351[SI5351_REG_OE] & 0x08) == 0);
    CHECK(radio_set_cal_tone(false, 0) == RADIO_OK);
    CHECK(test_vhw.si5351[SI5351_REG_CLK_CTRL + 3] & SI5351_CLK_PDN);   /* powered down, not only disabled */
}

int main(void)
{
    RUN(filter_edges_follow_the_bank_table);
    RUN(tune_direct_programs_the_si5351_path_and_filter);
    RUN(tune_low_selects_the_divide_by_four_path);
    RUN(out_of_range_leaves_the_radio_as_it_was);
    RUN(missing_si5351_is_an_i2c_error);
    RUN(frontend_keeps_the_filter);
    RUN(cal_tone_runs_clk3_only_while_on);
    return CHECK_DONE();
}
```

`firmware/tests/test_supervisor.c`:

```c
#include "check.h"
#include "supervisor.h"
#include "tables_gen.h"
#include "test_hal.h"

static uint32_t sense_mv_for(uint32_t ua)
{
    /* inverse of the table, by search: the smallest mV that reads at least ua */
    uint32_t mv = 0;
    while (supervisor_bias_ua(mv) < ua && mv < 3300) mv++;
    return mv;
}

static void usb_supply(uint32_t vbus_mv, uint32_t cc_mv)
{
    test_vhw.adc_mv[HAL_ADC_SENSE_VBUS] = vbus_mv / 2;
    test_vhw.adc_mv[HAL_ADC_SENSE_MAIN] = (vbus_mv - 100) / 2;
    test_vhw.adc_mv[HAL_ADC_SENSE_EXT] = 0;
    test_vhw.adc_mv[HAL_ADC_CC1] = cc_mv;
}

static void sense_table_reads_back_its_own_points(void)
{
    for (int k = 0; k < TABLES_BIAS_SENSE_POINTS; k++)
        CHECK(supervisor_bias_ua(TABLES_BIAS_SENSE_MV[k]) == TABLES_BIAS_SENSE_UA[k]);
    CHECK(supervisor_bias_ua(0) == 0 && supervisor_bias_ua(500) == 0);
    uint32_t prev = 0;
    for (uint32_t mv = 600; mv <= 2000; mv += 10) {
        uint32_t ua = supervisor_bias_ua(mv);
        CHECK(ua >= prev);
        prev = ua;
    }
}

static void cc_voltage_maps_to_the_advertised_current(void)
{
    CHECK(supervisor_cc_from_mv(0) == SUP_CC_NONE);
    CHECK(supervisor_cc_from_mv(410) == SUP_CC_DEFAULT);
    CHECK(supervisor_cc_from_mv(920) == SUP_CC_1A5);
    CHECK(supervisor_cc_from_mv(1680) == SUP_CC_3A);
}

static void bias_needs_the_jack_or_a_1a5_source(void)
{
    supervisor_init();
    usb_supply(5000, 410);
    CHECK(!supervisor_bias_enable(true));
    CHECK(test_vhw.gpio[HAL_GPIO_BIAS_EN] == false);
    usb_supply(5000, 920);
    CHECK(supervisor_bias_enable(true));
    CHECK(test_vhw.gpio[HAL_GPIO_BIAS_EN] == true);
    supervisor_bias_enable(false);
    usb_supply(5000, 410);
    test_vhw.adc_mv[HAL_ADC_SENSE_EXT] = 2500;          /* 5.0 V on the jack */
    CHECK(supervisor_bias_enable(true));
}

static void over_current_cuts_the_feed(void)
{
    supervisor_init();
    usb_supply(5000, 1680);
    CHECK(supervisor_bias_enable(true));
    test_vhw.adc_mv[HAL_ADC_BIAS_ISENSE] = sense_mv_for(140000);
    CHECK(supervisor_poll()->flags & SUP_FLAG_BIAS_ON);
    test_vhw.adc_mv[HAL_ADC_BIAS_ISENSE] = sense_mv_for(160000);
    const sup_status_t *s = supervisor_poll();
    CHECK(!(s->flags & SUP_FLAG_BIAS_ON) && (s->flags & SUP_FLAG_BIAS_TRIPPED));
    CHECK(test_vhw.gpio[HAL_GPIO_BIAS_EN] == false);
    test_vhw.adc_mv[HAL_ADC_BIAS_ISENSE] = sense_mv_for(50000);   /* fault cleared */
    CHECK(supervisor_bias_enable(true));                 /* re-enabling clears the trip */
    s = supervisor_poll();
    CHECK((s->flags & SUP_FLAG_BIAS_ON) && !(s->flags & SUP_FLAG_BIAS_TRIPPED));
}

static void low_usb_is_flagged_and_temperature_read(void)
{
    supervisor_init();
    usb_supply(4600, 410);
    vhw_set_temp_mc(&test_vhw, 41500);
    const sup_status_t *s = supervisor_poll();
    CHECK(s->flags & SUP_FLAG_VBUS_LOW);
    CHECK(s->vbus_mv == 4600);
    CHECK(s->temp_mc == 41500);
    test_vhw.lm75_present = false;
    CHECK(supervisor_poll()->flags & SUP_FLAG_TEMP_FAIL);
}

int main(void)
{
    RUN(sense_table_reads_back_its_own_points);
    RUN(cc_voltage_maps_to_the_advertised_current);
    RUN(bias_needs_the_jack_or_a_1a5_source);
    RUN(over_current_cuts_the_feed);
    RUN(low_usb_is_flagged_and_temperature_read);
    return CHECK_DONE();
}
```

`firmware/tests/test_cat.c`:

```c
#include <string.h>

#include "cat.h"
#include "check.h"
#include "radio.h"
#include "relays.h"
#include "supervisor.h"
#include "test_hal.h"

static char reply[512];

static void collect(const char *s, size_t n)
{
    size_t have = strlen(reply);
    if (have + n < sizeof reply) {
        memcpy(reply + have, s, n);
        reply[have + n] = 0;
    }
}

static const char *ask(const char *cmd)
{
    reply[0] = 0;
    cat_input((const uint8_t *)cmd, strlen(cmd));
    return reply;
}

static void start(void)
{
    radio_init();
    supervisor_init();
    cat_init(collect);
    test_vhw.adc_mv[HAL_ADC_SENSE_VBUS] = 2500;
    test_vhw.adc_mv[HAL_ADC_CC1] = 410;
}

static void fa_tunes_and_reads_back(void)
{
    start();
    CHECK(strcmp(ask("FA00014050000;"), "") == 0);
    CHECK(radio_state()->lo_hz == 14050000);
    CHECK(strcmp(ask("FA;"), "FA00014050000;") == 0);
    CHECK(strcmp(ask("FA00099000000;"), "?;") == 0);       /* outside 0.1–54 MHz */
    CHECK(radio_state()->lo_hz == 14050000);
    CHECK(strcmp(ask("FA1234x;"), "?;") == 0);
}

static void if_answer_is_38_characters_with_the_mode_at_29(void)
{
    start();
    ask("FA00007030000;MD3;");
    const char *r = ask("IF;");
    CHECK(strlen(r) == 38);
    CHECK(strncmp(r, "IF00007030000", 13) == 0);
    CHECK(r[29] == '3' && r[37] == ';');
}

static void identity_mode_and_unknown_commands(void)
{
    start();
    CHECK(strcmp(ask("ID;"), "ID019;") == 0);
    CHECK(strcmp(ask("MD;"), "MD2;") == 0);
    CHECK(strcmp(ask("MD0;"), "?;") == 0);
    CHECK(strcmp(ask("XX;"), "?;") == 0);
    CHECK(strcmp(ask(";"), "?;") == 0);
    CHECK(strcmp(ask("AI;PS;RX;"), "AI0;PS1;RX0;") == 0);
}

static void input_may_arrive_in_pieces_with_line_ends(void)
{
    start();
    ask("F");
    ask("A0001405");
    CHECK(strcmp(ask("0000;\r\n"), "") == 0);
    CHECK(radio_state()->lo_hz == 14050000);
    char longcmd[200];
    memset(longcmd, '1', sizeof longcmd - 2);
    longcmd[sizeof longcmd - 2] = ';';
    longcmd[sizeof longcmd - 1] = 0;
    CHECK(strcmp(ask(longcmd), "?;") == 0);                /* overlong: rejected, parser recovers */
    CHECK(strcmp(ask("ID;"), "ID019;") == 0);
}

static void preamp_and_attenuator_switch_relays(void)
{
    start();
    ask("FA00014050000;PA1;RA01;");
    CHECK(strcmp(ask("PA;RA;"), "PA10;RA0100;") == 0);
    CHECK(vhw_relay_outputs(&test_vhw) == relays_word(7, true, true));
    CHECK(strcmp(ask("PA2;"), "?;") == 0);
}

static void bias_t_needs_power_and_reports_telemetry(void)
{
    start();
    CHECK(strcmp(ask("ZB1;"), "?;") == 0);                 /* USB default current only */
    test_vhw.adc_mv[HAL_ADC_CC1] = 920;
    CHECK(strcmp(ask("ZB1;ZB;"), "ZB1;") == 0);
    const char *t = ask("ZT;");
    CHECK(strncmp(t, "ZT5000,0,", 9) == 0 && t[strlen(t) - 1] == ';');
}

static void calibration_tone_on_and_off(void)
{
    start();
    ask("FA00014050000;");
    CHECK(strcmp(ask("ZC110000;ZC;"), "ZC110000;") == 0);
    CHECK(strcmp(ask("ZC0;ZC;"), "ZC00;") == 0);
}

int main(void)
{
    RUN(fa_tunes_and_reads_back);
    RUN(if_answer_is_38_characters_with_the_mode_at_29);
    RUN(identity_mode_and_unknown_commands);
    RUN(input_may_arrive_in_pieces_with_line_ends);
    RUN(preamp_and_attenuator_switch_relays);
    RUN(bias_t_needs_power_and_reports_telemetry);
    RUN(calibration_tone_on_and_off);
    return CHECK_DONE();
}
```

`firmware/tests/test_audio.c`:

```c
#include <string.h>

#include "audio.h"
#include "check.h"
#include "test_hal.h"

static void packet_is_three_byte_little_endian_i_then_q(void)
{
    audio_init();
    int32_t iq[4] = {0x123456, -2, -8388608, 8388607};
    test_i2s_queue(iq, 2);
    audio_pump();
    uint8_t p[12];
    CHECK(audio_packet(p, 2) == 12);
    const uint8_t want[12] = {0x56, 0x34, 0x12, 0xFE, 0xFF, 0xFF, 0x00, 0x00, 0x80, 0xFF, 0xFF, 0x7F};
    CHECK(memcmp(p, want, 12) == 0);
    CHECK(audio_stats()->underruns == 0 && audio_stats()->level == 0);
}

static void a_short_ring_pads_with_silence_and_counts_one_underrun(void)
{
    audio_init();
    int32_t iq[2] = {1, 1};
    test_i2s_queue(iq, 1);
    audio_pump();
    uint8_t p[AUDIO_PACKET_BYTES];
    memset(p, 0xAA, sizeof p);
    audio_packet(p, AUDIO_FRAMES_PER_MS);
    CHECK(p[0] == 1 && p[6] == 0 && p[AUDIO_PACKET_BYTES - 1] == 0);
    CHECK(audio_stats()->underruns == 1);
}

static void a_full_ring_drops_and_counts_overruns(void)
{
    audio_init();
    static int32_t iq[2 * 5000];
    for (int k = 0; k < 5000; k++) iq[2 * k] = k;
    test_i2s_queue(iq, 5000);
    audio_pump();
    CHECK(audio_stats()->level == AUDIO_RING_FRAMES);
    CHECK(audio_stats()->overruns == 5000 - AUDIO_RING_FRAMES);
    uint8_t p[6];
    audio_packet(p, 1);
    CHECK(p[0] == 0 && p[1] == 0);                         /* oldest frame first */
}

static void packet_size_is_576_bytes_per_millisecond(void) { CHECK(AUDIO_PACKET_BYTES == 576); }

int main(void)
{
    RUN(packet_is_three_byte_little_endian_i_then_q);
    RUN(a_short_ring_pads_with_silence_and_counts_one_underrun);
    RUN(a_full_ring_drops_and_counts_overruns);
    RUN(packet_size_is_576_bytes_per_millisecond);
    return CHECK_DONE();
}
```

`firmware/CMakeLists.txt`:

```cmake
# Elf firmware, Linux host build: the hardware-independent modules, their unit tests and (from
# plan 4.2) the virtual board. The ESP32-S3 target build is a separate ESP-IDF project.
cmake_minimum_required(VERSION 3.20)
project(elf_firmware C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Werror -Wshadow -Wconversion -Wno-sign-conversion)

set(CHAR_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../hardware/characterized CACHE PATH "characterisation files")
set(GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/gen)
file(MAKE_DIRECTORY ${GEN_DIR})
find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_custom_command(
    OUTPUT ${GEN_DIR}/tables_gen.h ${GEN_DIR}/tables_gen.c
    COMMAND Python3::Interpreter ${CMAKE_CURRENT_SOURCE_DIR}/tools/gen_tables.py ${CHAR_DIR} ${GEN_DIR}
    DEPENDS tools/gen_tables.py ${CHAR_DIR}/bpf_bank.json ${CHAR_DIR}/controller.json
    COMMENT "Generating firmware tables from the characterisation files")

add_library(elf_core STATIC
    core/lo_plan.c core/si5351.c core/relays.c core/radio.c core/supervisor.c core/cat.c core/audio.c
    ${GEN_DIR}/tables_gen.c)
target_include_directories(elf_core PUBLIC core ${GEN_DIR})

add_library(elf_vhw STATIC host/vhw.c)
target_include_directories(elf_vhw PUBLIC host)
target_link_libraries(elf_vhw PUBLIC elf_core)

enable_testing()
foreach(t si5351 relays radio supervisor cat audio)
    add_executable(test_${t} tests/test_${t}.c tests/test_hal.c)
    target_include_directories(test_${t} PRIVATE tests)
    target_link_libraries(test_${t} PRIVATE elf_core elf_vhw)
    add_test(NAME ${t} COMMAND test_${t})
endforeach()
```

- [ ] **Step 2: Run them to verify they fail**

Run: `cmake -S firmware -B build/firmware -G Ninja && ninja -C build/firmware`
Expected: FAIL — CMake: `Cannot find source file: core/lo_plan.c`.

- [ ] **Step 3: Implement**

`firmware/tools/gen_tables.py`:

```python
"""Generates firmware/core/tables_gen.{h,c} from the committed characterisation files, so the
firmware's filter edges, relay bit order, GPIO pads and bias-sense table are the hardware
design's own numbers (phase 4 spec §2). Files are rewritten only when their text changes."""
from __future__ import annotations

import json
import sys
from pathlib import Path

HEADER = "/* Generated by firmware/tools/gen_tables.py from hardware/characterized/ — do not edit. */"


def _array(name: str, values) -> str:
    return f"const uint32_t {name}[] = {{" + ", ".join(f"{v}u" for v in values) + "};"


def render(char_dir: Path) -> tuple[str, str]:
    bank = json.loads((char_dir / "bpf_bank.json").read_text())
    ctl = json.loads((char_dir / "controller.json").read_text())
    filters = sorted(bank["filters"], key=lambda f: f["index"])
    order = ctl["relay_chain_order"]
    table = ctl["bias_sense_table_27c"]
    h = [HEADER, "#ifndef ELF_TABLES_GEN_H", "#define ELF_TABLES_GEN_H", "", "#include <stdint.h>", "",
         f"#define TABLES_FILTER_COUNT {len(filters)}",
         "/* filter n (1-based) covers lo_hz <= f < hi_hz; the top edge selects the last filter */",
         "extern const uint32_t TABLES_FILTER_LO_HZ[];", "extern const uint32_t TABLES_FILTER_HI_HZ[];",
         "", f"#define TABLES_RELAY_COUNT {len(order)}",
         "/* bit k of the shift-register chain (74HC595 #k/8, output Q(k%8)) drives relay order[k] */"]
    h += [f"#define RELAY_BIT_{name.upper()} {k}" for k, name in enumerate(order)]
    h += ["", "/* ESP32-S3 pads (target build) */"]
    h += [f"#define GPIO_PAD_{name.upper()} {int(pad.removeprefix('IO'))}" for name, pad in ctl["gpio"].items()]
    h += ["", "/* bias-T current mirror at 27 °C: sense mV -> current µA (monotonic) */",
          f"#define TABLES_BIAS_SENSE_POINTS {len(table)}",
          "extern const uint32_t TABLES_BIAS_SENSE_MV[];", "extern const uint32_t TABLES_BIAS_SENSE_UA[];",
          f"#define TABLES_BIAS_CUTOFF_UA {round(ctl['bias_cutoff_a'] * 1e6)}u",
          f"#define TABLES_I2C_SI5351 0x{ctl['i2c']['Si5351C']:02x}",
          f"#define TABLES_I2C_LM75A 0x{ctl['i2c']['LM75A']:02x}", "", "#endif", ""]
    c = [HEADER, '#include "tables_gen.h"', "",
         _array("TABLES_FILTER_LO_HZ", (int(f["lo_hz"]) for f in filters)),
         _array("TABLES_FILTER_HI_HZ", (int(f["hi_hz"]) for f in filters)),
         _array("TABLES_BIAS_SENSE_MV", (round(p["sense_v"] * 1000) for p in table)),
         _array("TABLES_BIAS_SENSE_UA", (max(0, round(p["i_a"] * 1e6)) for p in table)), ""]
    return "\n".join(h), "\n".join(c)


def _write(path: Path, text: str) -> None:
    if not path.exists() or path.read_text() != text:
        path.write_text(text)


if __name__ == "__main__":
    char_dir, out_dir = Path(sys.argv[1]), Path(sys.argv[2])
    h, c = render(char_dir)
    _write(out_dir / "tables_gen.h", h)
    _write(out_dir / "tables_gen.c", c)
```

`firmware/core/hal.h`:

```c
/* The firmware's only hardware interface. target/esp32s3 implements it on the chip;
 * host/hal_virtual.c implements it as the virtual board. Pin and channel names follow the
 * GPIO map in hardware/characterized/controller.json. */
#ifndef ELF_HAL_H
#define ELF_HAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    HAL_GPIO_LO_PATH,       /* 0 = direct quadrature, 1 = ÷4 path */
    HAL_GPIO_SR_SER,
    HAL_GPIO_SR_CLK,
    HAL_GPIO_SR_LATCH,
    HAL_GPIO_SR_OE,         /* active low: 0 = relay drivers enabled */
    HAL_GPIO_BIAS_EN,
    HAL_GPIO_STATUS_LED,
    HAL_GPIO_ADC_RST,       /* active low */
    HAL_GPIO_ADC_MS,
    HAL_GPIO_ADC_FMT,
    HAL_GPIO_ADC_OVFL,      /* input, active low */
    HAL_GPIO_COUNT
} hal_gpio_t;

typedef enum {
    HAL_ADC_SENSE_VBUS,     /* 100 k / 100 k divider */
    HAL_ADC_SENSE_EXT,
    HAL_ADC_SENSE_MAIN,
    HAL_ADC_BIAS_ISENSE,    /* bias-T current mirror */
    HAL_ADC_CC1,
    HAL_ADC_CC2,
    HAL_ADC_COUNT
} hal_adc_t;

int hal_i2c_write(uint8_t addr, uint8_t reg, const uint8_t *data, size_t len);   /* 0 = ACK */
int hal_i2c_read(uint8_t addr, uint8_t reg, uint8_t *data, size_t len);
void hal_gpio_set(hal_gpio_t pin, bool level);
bool hal_gpio_get(hal_gpio_t pin);
uint32_t hal_adc_mv(hal_adc_t ch);
/* Up to max_frames I2S frames, interleaved I, Q as signed 24-bit values in int32; returns the
 * number read (0 when none are ready). */
size_t hal_i2s_read(int32_t *iq, size_t max_frames);
uint32_t hal_millis(void);
void hal_delay_ms(uint32_t ms);

#endif
```

`firmware/core/lo_plan.h`:

```c
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
```

`firmware/core/lo_plan.c`:

```c
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
```

`firmware/core/si5351.h`:

```c
/* Si5351C register image and programming sequence for a frequency plan (Silicon Labs AN619).
 *
 * The image covers the registers the board uses: 3 (output enable), 15 (PLL source: crystal),
 * 16–19 (CLK0–CLK3 control; CLK4–7 stay powered down), 26–41 (PLLA/PLLB), 42–73 (MultiSynth
 * 0–3), 165–168 (CLK0–3 phase offset), 177 (PLL reset), 183 (crystal load 10 pF). Unused
 * outputs are powered down, not only disabled; CLK3 (the calibration tone) runs only when the
 * plan has it. */
#ifndef ELF_SI5351_H
#define ELF_SI5351_H

#include <stdbool.h>
#include <stdint.h>

#include "lo_plan.h"

#define SI5351_ADDR 0x60
#define SI5351_REG_COUNT 256

enum {
    SI5351_REG_OE = 3,
    SI5351_REG_PLL_SRC = 15,
    SI5351_REG_CLK_CTRL = 16,
    SI5351_REG_PLLA = 26,
    SI5351_REG_PLLB = 34,
    SI5351_REG_MS0 = 42,
    SI5351_REG_PHOFF = 165,
    SI5351_REG_PLL_RESET = 177,
    SI5351_REG_XTAL_LOAD = 183,
};

#define SI5351_CLK_PDN 0x80
#define SI5351_CLK_INT 0x40
#define SI5351_CLK_SRC_PLLB 0x20
#define SI5351_CLK_SRC_MS 0x0C
#define SI5351_CLK_DRIVE_8MA 0x03
#define SI5351_XTAL_LOAD_10PF 0xD2      /* XTAL_CL = 11, low bits 010010 as AN619 requires */
#define SI5351_PLL_RESET_BOTH 0xA0

/* Fills `reg` (all 256 registers, untouched ones 0) for the plan. */
void si5351_image(const lo_plan_t *p, uint8_t reg[SI5351_REG_COUNT]);

/* Encodes a + b/c as AN619's P1, P2, P3 into 8 bytes at `out` (PLL or MultiSynth layout;
 * `r_div` only applies to MultiSynth; pass 1 for a PLL). */
void si5351_params(frac_t x, uint8_t r_div, uint8_t out[8]);

/* Writes the image through hal_i2c_write: outputs off, PLL and MultiSynth parameters, phase
 * offsets, crystal load, PLL reset, outputs on. Returns 0 or the first I2C error. */
int si5351_program(const uint8_t reg[SI5351_REG_COUNT]);

#endif
```

`firmware/core/si5351.c`:

```c
#include "si5351.h"

#include <string.h>

#include "hal.h"

static uint8_t log2_u8(uint8_t v)
{
    uint8_t k = 0;
    while (v > 1) {
        v >>= 1;
        k++;
    }
    return k;
}

void si5351_params(frac_t x, uint8_t r_div, uint8_t out[8])
{
    uint32_t a = (uint32_t)(x.num / x.den), b = (uint32_t)(x.num % x.den), c = (uint32_t)x.den;
    uint32_t f = (uint32_t)(((uint64_t)128 * b) / c);
    uint32_t p1 = 128 * a + f - 512;
    uint32_t p2 = 128 * b - c * f;
    uint32_t p3 = c;
    out[0] = (uint8_t)(p3 >> 8);
    out[1] = (uint8_t)p3;
    out[2] = (uint8_t)((log2_u8(r_div) << 4) | ((p1 >> 16) & 0x03));
    out[3] = (uint8_t)(p1 >> 8);
    out[4] = (uint8_t)p1;
    out[5] = (uint8_t)(((p3 >> 12) & 0xF0) | ((p2 >> 16) & 0x0F));
    out[6] = (uint8_t)(p2 >> 8);
    out[7] = (uint8_t)p2;
}

void si5351_image(const lo_plan_t *p, uint8_t reg[SI5351_REG_COUNT])
{
    memset(reg, 0, SI5351_REG_COUNT);
    reg[SI5351_REG_OE] = 0xFF;
    reg[SI5351_REG_PLL_SRC] = 0x00;                  /* both PLLs from the crystal */
    for (int k = 0; k < 8; k++) reg[SI5351_REG_CLK_CTRL + k] = SI5351_CLK_PDN;
    for (int pll = 0; pll < 2; pll++)
        if (p->pll_used[pll]) si5351_params(p->pll_mult[pll], 1, &reg[pll ? SI5351_REG_PLLB : SI5351_REG_PLLA]);
    for (int clk = 0; clk < 4; clk++) {
        const lo_output_t *o = &p->out[clk];
        if (!o->used) continue;
        uint8_t ctrl = SI5351_CLK_SRC_MS | SI5351_CLK_DRIVE_8MA;
        if (o->ms.den == 1) ctrl |= SI5351_CLK_INT;
        if (o->pll == LO_PLL_B) ctrl |= SI5351_CLK_SRC_PLLB;
        reg[SI5351_REG_CLK_CTRL + clk] = ctrl;
        si5351_params(o->ms, o->r_div, &reg[SI5351_REG_MS0 + 8 * clk]);
        reg[SI5351_REG_PHOFF + clk] = o->phoff & 0x7F;
        reg[SI5351_REG_OE] &= (uint8_t)~(1u << clk);
    }
    reg[SI5351_REG_PLL_RESET] = SI5351_PLL_RESET_BOTH;
    reg[SI5351_REG_XTAL_LOAD] = SI5351_XTAL_LOAD_10PF;
}

static int put(const uint8_t *reg, uint8_t first, uint8_t count)
{
    return hal_i2c_write(SI5351_ADDR, first, &reg[first], count);
}

int si5351_program(const uint8_t reg[SI5351_REG_COUNT])
{
    static const uint8_t off = 0xFF;
    int e;
    if ((e = hal_i2c_write(SI5351_ADDR, SI5351_REG_OE, &off, 1))) return e;
    if ((e = put(reg, SI5351_REG_PLL_SRC, 1))) return e;
    if ((e = put(reg, SI5351_REG_CLK_CTRL, 8))) return e;
    if ((e = put(reg, SI5351_REG_PLLA, 16))) return e;
    if ((e = put(reg, SI5351_REG_MS0, 32))) return e;
    if ((e = put(reg, SI5351_REG_PHOFF, 4))) return e;
    if ((e = put(reg, SI5351_REG_XTAL_LOAD, 1))) return e;
    if ((e = put(reg, SI5351_REG_PLL_RESET, 1))) return e;
    return put(reg, SI5351_REG_OE, 1);
}
```

`firmware/core/relays.h`:

```c
/* Relay chain: 3 × 74HC595 → 3 × ULN2803 → 24 coils. Bit k of the word is chain bit k
 * (74HC595 #k/8, output Q(k%8)), in the order of controller.json (tables_gen.h). */
#ifndef ELF_RELAYS_H
#define ELF_RELAYS_H

#include <stdbool.h>
#include <stdint.h>

/* The word for a filter (1–11, 0 = bank muted: every filter released), attenuator and preamp. */
uint32_t relays_word(int filter, bool atten, bool preamp);

/* Shifts the word into the chain (bit 23 first, so bit 0 lands on the first register's QA),
 * latches it and enables the outputs (/OE low). */
void relays_write(uint32_t word);

/* Power-up: outputs disabled (/OE high, as the pull-up holds them) and an all-released word
 * latched before they are enabled. */
void relays_init(void);

#endif
```

`firmware/core/relays.c`:

```c
#include "relays.h"

#include "hal.h"
#include "tables_gen.h"

uint32_t relays_word(int filter, bool atten, bool preamp)
{
    uint32_t w = 0;
    if (filter >= 1 && filter <= TABLES_FILTER_COUNT) {
        int in_bit = RELAY_BIT_BANK1_IN + 2 * (filter - 1);
        w |= 1u << in_bit;
        w |= 1u << (in_bit + 1);                     /* the filter's output relay follows its input relay */
    }
    if (atten) w |= 1u << RELAY_BIT_ATTEN;
    if (preamp) w |= 1u << RELAY_BIT_PREAMP;
    return w;
}

static void shift_latch(uint32_t word)
{
    for (int k = TABLES_RELAY_COUNT - 1; k >= 0; k--) {
        hal_gpio_set(HAL_GPIO_SR_SER, (word >> k) & 1u);
        hal_gpio_set(HAL_GPIO_SR_CLK, true);
        hal_gpio_set(HAL_GPIO_SR_CLK, false);
    }
    hal_gpio_set(HAL_GPIO_SR_LATCH, true);
    hal_gpio_set(HAL_GPIO_SR_LATCH, false);
}

void relays_write(uint32_t word)
{
    shift_latch(word);
    hal_gpio_set(HAL_GPIO_SR_OE, false);
}

void relays_init(void)
{
    hal_gpio_set(HAL_GPIO_SR_OE, true);
    hal_gpio_set(HAL_GPIO_SR_CLK, false);
    hal_gpio_set(HAL_GPIO_SR_LATCH, false);
    relays_write(0);
}
```

`firmware/core/radio.h`:

```c
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
```

`firmware/core/radio.c`:

```c
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
```

`firmware/core/supervisor.h`:

```c
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
```

`firmware/core/supervisor.c`:

```c
#include "supervisor.h"

#include "hal.h"
#include "tables_gen.h"

static sup_status_t st;

uint32_t supervisor_bias_ua(uint32_t mv)
{
    const uint32_t *x = TABLES_BIAS_SENSE_MV, *y = TABLES_BIAS_SENSE_UA;
    const int n = TABLES_BIAS_SENSE_POINTS;
    if (mv <= x[0]) return 0;
    int k = 1;
    while (k < n - 1 && mv > x[k]) k++;
    /* linear between points k-1 and k (extrapolated beyond the last segment) */
    int64_t num = (int64_t)(mv - x[k - 1]) * (int64_t)(y[k] - y[k - 1]);
    int64_t v = (int64_t)y[k - 1] + num / (int64_t)(x[k] - x[k - 1]);
    return v < 0 ? 0 : (uint32_t)v;
}

sup_cc_t supervisor_cc_from_mv(uint32_t mv)
{
    if (mv < 200) return SUP_CC_NONE;
    if (mv < SUP_CC_1A5_MIN_MV) return SUP_CC_DEFAULT;
    if (mv < SUP_CC_3A_MIN_MV) return SUP_CC_1A5;
    return SUP_CC_3A;
}

void supervisor_init(void)
{
    st = (sup_status_t){0};
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

const sup_status_t *supervisor_poll(void)
{
    st.vbus_mv = hal_adc_mv(HAL_ADC_SENSE_VBUS) * SUP_DIVIDER;
    st.ext_mv = hal_adc_mv(HAL_ADC_SENSE_EXT) * SUP_DIVIDER;
    st.main_mv = hal_adc_mv(HAL_ADC_SENSE_MAIN) * SUP_DIVIDER;
    uint32_t cc1 = hal_adc_mv(HAL_ADC_CC1), cc2 = hal_adc_mv(HAL_ADC_CC2);
    st.cc = supervisor_cc_from_mv(cc1 > cc2 ? cc1 : cc2);
    st.flags = (st.flags & (SUP_FLAG_BIAS_ON | SUP_FLAG_BIAS_TRIPPED | SUP_FLAG_TEMP_FAIL));
    if (st.ext_mv >= SUP_EXT_PRESENT_MV) st.flags |= SUP_FLAG_EXT;
    else if (st.vbus_mv < SUP_VBUS_LOW_MV) st.flags |= SUP_FLAG_VBUS_LOW;
    st.bias_ua = (st.flags & SUP_FLAG_BIAS_ON) ? supervisor_bias_ua(hal_adc_mv(HAL_ADC_BIAS_ISENSE)) : 0;
    if ((st.flags & SUP_FLAG_BIAS_ON) && st.bias_ua > TABLES_BIAS_CUTOFF_UA) {
        hal_gpio_set(HAL_GPIO_BIAS_EN, false);
        st.flags = (st.flags & ~SUP_FLAG_BIAS_ON) | SUP_FLAG_BIAS_TRIPPED;
    }
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
    if (!(st.flags & SUP_FLAG_EXT) && st.cc != SUP_CC_1A5 && st.cc != SUP_CC_3A) return false;
    st.flags &= ~SUP_FLAG_BIAS_TRIPPED;
    st.flags |= SUP_FLAG_BIAS_ON;
    hal_gpio_set(HAL_GPIO_BIAS_EN, true);
    return true;
}
```

`firmware/core/cat.h`:

```c
/* CAT over CDC: a Kenwood TS-2000 subset (works with Hamlib's TS-2000 backend) plus Elf
 * extensions. Commands end with ';'; an unknown or invalid command answers "?;".
 *
 *   FA / FB   VFO A / B frequency in Hz, 11 digits (FA tunes the receiver; FB is stored only)
 *   IF        transceiver status (38 characters)
 *   ID        "ID019;" (TS-2000)
 *   MD        mode 1–9 (stored for the host's demodulator; the board is I/Q only)
 *   PA        preamp: PA0 / PA1 (answer "PAx0;")
 *   RA        attenuator: RA00 / RA01 (answer "RAxx00;")
 *   AI, PS, RX  accepted (AI0, PS1, receive)
 *   ZB        Elf: bias-T, ZB0 / ZB1 (refused with "?;" without ≥ 1.5 A or the jack)
 *   ZT        Elf: telemetry "ZT<vbus mV>,<ext mV>,<main mV>,<bias µA>,<temp m°C>,<flags>;"
 *   ZC        Elf: calibration tone, ZC0 / ZC1<offset Hz>, answer "ZC<0|1><offset>;" */
#ifndef ELF_CAT_H
#define ELF_CAT_H

#include <stddef.h>
#include <stdint.h>

typedef void (*cat_out_fn)(const char *s, size_t n);

void cat_init(cat_out_fn out);
void cat_input(const uint8_t *data, size_t n);

#endif
```

`firmware/core/cat.c`:

```c
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
```

`firmware/core/audio.h`:

```c
/* I/Q audio path: I2S frames from the CS5361 (24 bits in 32-bit slots, L = I, R = Q) into a
 * ring buffer, out as UAC2 packets of 3-byte little-endian samples, I then Q (96 frames per
 * 1 ms frame at 96 kHz = 576 bytes). */
#ifndef ELF_AUDIO_H
#define ELF_AUDIO_H

#include <stddef.h>
#include <stdint.h>

#define AUDIO_RATE_HZ 96000u
#define AUDIO_CHANNELS 2u
#define AUDIO_BYTES_PER_SAMPLE 3u
#define AUDIO_FRAME_BYTES (AUDIO_CHANNELS * AUDIO_BYTES_PER_SAMPLE)
#define AUDIO_FRAMES_PER_MS (AUDIO_RATE_HZ / 1000u)
#define AUDIO_PACKET_BYTES (AUDIO_FRAMES_PER_MS * AUDIO_FRAME_BYTES)
#define AUDIO_RING_FRAMES 4096u        /* power of two, ≈ 43 ms */

typedef struct {
    uint32_t overruns;                 /* frames dropped: the ring was full */
    uint32_t underruns;                /* packets padded with silence: the ring ran dry */
    uint32_t level;                    /* frames waiting */
} audio_stats_t;

void audio_init(void);
/* Moves every I2S frame the HAL has ready into the ring. */
void audio_pump(void);
/* Fills `out` with `frames` frames (silence for any the ring lacks); returns bytes written. */
size_t audio_packet(uint8_t *out, size_t frames);
const audio_stats_t *audio_stats(void);

#endif
```

`firmware/core/audio.c`:

```c
#include "audio.h"

#include <string.h>

#include "hal.h"

static int32_t ring[AUDIO_RING_FRAMES][2];
static uint32_t head, tail;              /* free-running; frames = head - tail */
static audio_stats_t stats;

void audio_init(void)
{
    head = tail = 0;
    memset(&stats, 0, sizeof stats);
}

void audio_pump(void)
{
    int32_t chunk[64][2];
    size_t n;
    while ((n = hal_i2s_read(&chunk[0][0], 64)) > 0) {
        for (size_t k = 0; k < n; k++) {
            if (head - tail >= AUDIO_RING_FRAMES) {
                stats.overruns++;
                continue;
            }
            ring[head % AUDIO_RING_FRAMES][0] = chunk[k][0];
            ring[head % AUDIO_RING_FRAMES][1] = chunk[k][1];
            head++;
        }
    }
    stats.level = head - tail;
}

static void put24(uint8_t *p, int32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
}

size_t audio_packet(uint8_t *out, size_t frames)
{
    int short_packet = 0;
    for (size_t k = 0; k < frames; k++) {
        uint8_t *p = out + k * AUDIO_FRAME_BYTES;
        if (head == tail) {
            memset(p, 0, AUDIO_FRAME_BYTES);
            short_packet = 1;
            continue;
        }
        put24(p, ring[tail % AUDIO_RING_FRAMES][0]);
        put24(p + 3, ring[tail % AUDIO_RING_FRAMES][1]);
        tail++;
    }
    stats.underruns += (uint32_t)short_packet;
    stats.level = head - tail;
    return frames * AUDIO_FRAME_BYTES;
}

const audio_stats_t *audio_stats(void) { return &stats; }
```

`firmware/host/vhw.h`:

```c
/* Virtual board hardware: the parts the firmware drives, modelled at their pins and registers.
 * Used by the unit tests and by elf-virtual. The models follow the datasheets, not the
 * firmware: the 74HC595 chain shifts on SRCLK rising edges and latches on RCLK rising edges;
 * the Si5351C is a 256-byte register file with auto-incrementing writes. */
#ifndef ELF_VHW_H
#define ELF_VHW_H

#include <stdbool.h>
#include <stdint.h>

#include "hal.h"

#define VHW_CHAIN_BITS 24

typedef struct {
    uint8_t si5351[256];
    bool si5351_present, lm75_present;
    int16_t lm75_reg0;                 /* LM75A temperature register (11-bit value << 5) */
    bool gpio[HAL_GPIO_COUNT];
    uint32_t adc_mv[HAL_ADC_COUNT];
    uint32_t chain_shift;              /* shift-register stages, bit k = chain bit k */
    uint32_t chain_latched;            /* storage registers (the Q outputs when /OE is low) */
    uint32_t si5351_writes;            /* I2C write transactions to the Si5351 */
} vhw_t;

void vhw_reset(vhw_t *v);
int vhw_i2c_write(vhw_t *v, uint8_t addr, uint8_t reg, const uint8_t *data, size_t len);
int vhw_i2c_read(vhw_t *v, uint8_t addr, uint8_t reg, uint8_t *data, size_t len);
void vhw_gpio_set(vhw_t *v, hal_gpio_t pin, bool level);
/* The relay coils energised now: latched bits while /OE is low, none while it is high. */
uint32_t vhw_relay_outputs(const vhw_t *v);
void vhw_set_temp_mc(vhw_t *v, int32_t milli_c);

#endif
```

`firmware/host/vhw.c`:

```c
#include "vhw.h"

#include <string.h>

#include "si5351.h"
#include "tables_gen.h"

void vhw_reset(vhw_t *v)
{
    memset(v, 0, sizeof *v);
    v->si5351_present = v->lm75_present = true;
    /* power-up register defaults the firmware relies on being overwritten: outputs disabled */
    v->si5351[SI5351_REG_OE] = 0xFF;
    for (int k = 0; k < 8; k++) v->si5351[SI5351_REG_CLK_CTRL + k] = SI5351_CLK_PDN;
    /* board pull resistors: /OE high (outputs off), bias EN low, ADC RST low, LO path low */
    v->gpio[HAL_GPIO_SR_OE] = true;
    v->gpio[HAL_GPIO_ADC_OVFL] = true;
    vhw_set_temp_mc(v, 25000);
}

int vhw_i2c_write(vhw_t *v, uint8_t addr, uint8_t reg, const uint8_t *data, size_t len)
{
    if (addr == TABLES_I2C_SI5351 && v->si5351_present) {
        for (size_t k = 0; k < len; k++) v->si5351[(uint8_t)(reg + k)] = data[k];
        v->si5351_writes++;
        return 0;
    }
    if (addr == TABLES_I2C_LM75A && v->lm75_present) return 0;
    return -1;                                       /* no ACK */
}

int vhw_i2c_read(vhw_t *v, uint8_t addr, uint8_t reg, uint8_t *data, size_t len)
{
    if (addr == TABLES_I2C_SI5351 && v->si5351_present) {
        for (size_t k = 0; k < len; k++) data[k] = v->si5351[(uint8_t)(reg + k)];
        return 0;
    }
    if (addr == TABLES_I2C_LM75A && v->lm75_present) {
        uint16_t t = (uint16_t)v->lm75_reg0;
        for (size_t k = 0; k < len; k++) data[k] = reg == 0 ? (uint8_t)(k % 2 ? t : t >> 8) : 0;
        return 0;
    }
    return -1;
}

void vhw_gpio_set(vhw_t *v, hal_gpio_t pin, bool level)
{
    bool rising = level && !v->gpio[pin];
    v->gpio[pin] = level;
    if (pin == HAL_GPIO_SR_CLK && rising) {
        /* SER enters the first register's QA; every stage moves one place along the chain */
        v->chain_shift = ((v->chain_shift << 1) | (v->gpio[HAL_GPIO_SR_SER] ? 1u : 0u)) & ((1u << VHW_CHAIN_BITS) - 1);
    }
    if (pin == HAL_GPIO_SR_LATCH && rising) v->chain_latched = v->chain_shift;
}

uint32_t vhw_relay_outputs(const vhw_t *v) { return v->gpio[HAL_GPIO_SR_OE] ? 0 : v->chain_latched; }

void vhw_set_temp_mc(vhw_t *v, int32_t milli_c) { v->lm75_reg0 = (int16_t)((milli_c / 125) << 5); }
```

`Makefile (append before `clean:`; add `fw fw-test` to .PHONY; `test` runs `$(MAKE) fw-test` after the hardware tests)`:

```make
FW_BUILD := build/firmware

fw:
	cmake -S firmware -B $(FW_BUILD) -G Ninja -DCMAKE_BUILD_TYPE=Release
	ninja -C $(FW_BUILD)

# firmware unit tests, C-vs-Python cross-checks, the USB device over USB/IP and the five-band CW
# acceptance through the user-space USB/IP client (no root needed; the acceptance needs `make dsp`)
fw-test: fw
	ctest --test-dir $(FW_BUILD) --output-on-failure
```

- [ ] **Step 4: Run them to verify they pass**

Run: `make fw-test`
Expected: `100% tests passed, 0 tests failed out of 6`.

Mutation check (each must turn a suite red, then restore): reverse the loop in `shift_latch`; make `lo_path` follow `LO_PATH_DIRECT`; shift byte 2 of `put24` by 15; double the cut-off; drop the relay settle delay; never set `SI5351_CLK_INT`.

- [ ] **Step 5: Commit**

```bash
git add firmware Makefile
git commit -m "feat(fw): firmware core: LO planner, Si5351, relays, radio, supervisor, CAT, audio, with pin-level unit tests"
```

### Task 2: Cross-checks against the Python references

**Files:**
- Create: `sim/neosim/si5351_decode.py`, `firmware/tests/lo_plan_dump.c`, `firmware/tests/si5351_dump.c`
- Test: `sim/tests/test_si5351_decode.py`, `firmware/tests/test_crosscheck.py`
- Modify: `firmware/CMakeLists.txt` (dump tools), `Makefile` (`fw-test` runs pytest)

**Interfaces:**
- Consumes: Task 1 `lo_plan`, `si5351_image`.
- Produces: `si5351_decode.ratio(regs, base)`, `clocks(regs)`, `lo_state(regs, lo_path_high) -> LoState(lo_hz, path, cal_hz, reason)`; executables `lo_plan_dump START STOP STEP CAL`, `si5351_dump START STOP STEP CAL`.

- [ ] **Step 1: Write the failing tests**

`sim/tests/test_si5351_decode.py`:

```python
from fractions import Fraction

from neosim import si5351_decode as d


def _params(x: Fraction, rdiv_log2: int = 0) -> list[int]:
    a, b, c = x.numerator // x.denominator, x.numerator % x.denominator, x.denominator
    f = 128 * b // c
    p1, p2, p3 = 128 * a + f - 512, 128 * b - c * f, c
    return [p3 >> 8 & 0xFF, p3 & 0xFF, rdiv_log2 << 4 | p1 >> 16 & 3, p1 >> 8 & 0xFF, p1 & 0xFF,
            (p3 >> 12 & 0xF0) | (p2 >> 16 & 0x0F), p2 >> 8 & 0xFF, p2 & 0xFF]


def _direct(ms: int = 50, phoff: int = 50, pll=Fraction(28, 1)) -> bytearray:
    """14 MHz direct quadrature: PLLA 700 MHz, MS0 = MS1 = 50."""
    r = bytearray(256)
    r[3] = 0xFC
    r[16:24] = [0x4F, 0x4F] + [0x80] * 6
    r[26:34] = _params(pll)
    r[42:50] = _params(Fraction(ms))
    r[50:58] = _params(Fraction(ms))
    r[166] = phoff
    return r


def test_direct_quadrature_decodes_to_the_lo():
    s = d.lo_state(bytes(_direct()), lo_path_high=False)
    assert s.lo_hz == 14_000_000 and s.path == "direct" and s.cal_hz is None


def test_a_wrong_phase_offset_gives_no_lo():
    s = d.lo_state(bytes(_direct(phoff=49)), lo_path_high=False)
    assert s.lo_hz is None and "quarter period" in s.reason


def test_the_mux_select_must_match_the_running_outputs():
    s = d.lo_state(bytes(_direct()), lo_path_high=True)      # ÷4 selected, CLK2 off
    assert s.lo_hz is None and "CLK2" in s.reason


def test_a_disabled_output_gives_no_lo():
    r = _direct()
    r[3] = 0xFE                                              # CLK1 disabled
    assert d.lo_state(bytes(r), lo_path_high=False).lo_hz is None


def test_r_divider_and_fractional_multisynth():
    r = bytearray(256)
    r[3] = 0xFB                                              # CLK2 on
    r[16:24] = [0x80, 0x80, 0x6F, 0x80, 0x80, 0x80, 0x80, 0x80]   # CLK2: PLLB, integer
    r[34:42] = _params(Fraction(30))                         # 750 MHz
    r[58:66] = _params(Fraction(1500, 7), rdiv_log2=3)       # ÷(1500/7) ÷8
    s = d.lo_state(bytes(r), lo_path_high=True)
    assert s.lo_hz == Fraction(750_000_000 * 7, 1500 * 8 * 4)
```

`firmware/tests/test_crosscheck.py`:

```python
"""The firmware against the design's Python references (phase 4 spec §1 item 2): the C LO
planner equals hardware/neohw/lo_plan.py on the whole 1 kHz grid, and every Si5351 register
image the firmware builds decodes, through the independent AN619 decoder in sim/neosim, to
exactly the LO and calibration tone asked for."""
from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
BUILD = Path(os.environ.get("ELF_FW_BUILD", ROOT / "build" / "firmware"))
sys.path[:0] = [str(ROOT / "hardware"), str(ROOT / "sim")]

from neohw import lo_plan  # noqa: E402
from neosim import si5351_decode  # noqa: E402

GRID = ("100000", "54000000", "1000")


def _run(tool: str, *args: str) -> list[str]:
    exe = BUILD / tool
    if not exe.exists():
        pytest.skip(f"{exe} not built (make fw)")
    return subprocess.run([str(exe), *args], check=True, capture_output=True, text=True).stdout.splitlines()


def _fr(x) -> str:
    return f"{x.numerator}/{x.denominator}"


def _python_line(f: int, cal: int) -> str:
    try:
        p = lo_plan.plan(f, cal)
    except ValueError:
        return f"{f} error"
    parts = [str(f), p.path] + [_fr(p.pll_mult[k]) if k in p.pll_mult else "-" for k in ("A", "B")]
    for c in ("CLK0", "CLK1", "CLK2", "CLK3"):
        o = p.outputs.get(c)
        parts += ["-"] if o is None else [o.pll, _fr(o.ms), str(o.r_div), str(o.phoff)]
    return " ".join(parts)


def test_c_planner_equals_the_python_planner_on_the_whole_grid():
    c_lines = _run("lo_plan_dump", *GRID, "10000")
    start, stop, step = (int(x) for x in GRID)
    py_lines = [_python_line(f, 10000) for f in range(start, stop + 1, step)]
    assert len(c_lines) == len(py_lines) == 53901
    diff = [(c, p) for c, p in zip(c_lines, py_lines) if c != p]
    assert diff == [], diff[:3]


@pytest.mark.parametrize("cal", [0, 10000])
def test_register_images_decode_to_the_requested_lo_and_tone(cal):
    bad = []
    for line in _run("si5351_dump", *GRID, str(cal)):
        f, path, hx = line.split()
        s = si5351_decode.lo_state(bytes.fromhex(hx), path == "1")
        want_cal = int(f) + cal if cal else None
        if s.lo_hz != int(f) or s.cal_hz != want_cal:
            bad.append((f, s))
    assert bad == [], bad[:3]
```

- [ ] **Step 2: Run them to verify they fail**

Run: `cd sim && python3 -m pytest -q tests/test_si5351_decode.py`
Expected: FAIL — `ImportError: cannot import name 'si5351_decode' from 'neosim'`.

- [ ] **Step 3: Implement**

`sim/neosim/si5351_decode.py`:

```python
"""Si5351C register decoder for the virtual board (phase 4 spec §3): written from AN619
independently of the firmware's encoder, so a wrong register shows up as a wrong frequency.

a + b/c = (P1 + 512 + P2 / P3) / 128 for both the PLL feedback and the MultiSynth dividers;
f_out = f_xtal · PLL / (MS · R)."""
from __future__ import annotations

from dataclasses import dataclass
from fractions import Fraction

XTAL_HZ = 25_000_000
PLL_BASE = {"A": 26, "B": 34}
MS_BASE = 42
CLK_CTRL, OE, PHOFF = 16, 3, 165


def _p123(r: bytes, base: int) -> tuple[int, int, int]:
    p3 = (r[base] << 8) | r[base + 1] | ((r[base + 5] >> 4) << 16)
    p1 = ((r[base + 2] & 0x03) << 16) | (r[base + 3] << 8) | r[base + 4]
    p2 = ((r[base + 5] & 0x0F) << 16) | (r[base + 6] << 8) | r[base + 7]
    return p1, p2, p3


def ratio(r: bytes, base: int) -> Fraction:
    p1, p2, p3 = _p123(r, base)
    return (p1 + 512 + Fraction(p2, p3)) / 128


@dataclass(frozen=True)
class Clock:
    enabled: bool
    pll: str
    hz: Fraction
    ms: Fraction
    phoff: int


def clocks(r: bytes) -> dict[int, Clock]:
    out = {}
    for k in range(4):
        ctrl = r[CLK_CTRL + k]
        on = not (ctrl & 0x80) and not (r[OE] >> k) & 1 and (ctrl & 0x0C) == 0x0C
        pll = "B" if ctrl & 0x20 else "A"
        base = MS_BASE + 8 * k
        if not on:
            out[k] = Clock(False, pll, Fraction(0), Fraction(0), 0)
            continue
        ms = ratio(r, base)
        rdiv = 1 << ((r[base + 2] >> 4) & 0x07)
        hz = XTAL_HZ * ratio(r, PLL_BASE[pll]) / (ms * rdiv)
        out[k] = Clock(True, pll, hz, ms, r[PHOFF + k] & 0x7F)
    return out


@dataclass(frozen=True)
class LoState:
    lo_hz: Fraction | None        # None: no valid quadrature LO
    path: str
    cal_hz: Fraction | None
    reason: str


def lo_state(r: bytes, lo_path_high: bool) -> LoState:
    """What the mixer sees: direct path = CLK0 and CLK1 a quarter period apart; ÷4 path =
    CLK2 / 4 through the Johnson counter. The mux select must match."""
    c = clocks(r)
    cal = c[3].hz if c[3].enabled else None
    if lo_path_high:
        if not c[2].enabled:
            return LoState(None, "div4", cal, "÷4 path selected but CLK2 is off")
        return LoState(c[2].hz / 4, "div4", cal, "")
    a, b = c[0], c[1]
    if not (a.enabled and b.enabled):
        return LoState(None, "direct", cal, "direct path selected but CLK0/CLK1 are off")
    if a.pll != b.pll or a.ms != b.ms or a.ms.denominator != 1:
        return LoState(None, "direct", cal, "CLK0/CLK1 not the same integer divider on one PLL")
    if (b.phoff - a.phoff) != a.ms:
        return LoState(None, "direct", cal, f"phase offset {b.phoff - a.phoff} is not a quarter period ({a.ms})")
    return LoState(a.hz, "direct", cal, "")
```

`firmware/tests/lo_plan_dump.c`:

```c
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
```

`firmware/tests/si5351_dump.c`:

```c
/* Prints the Si5351 register image for every LO on a grid: "<f> <path> <256 hex bytes>". */
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

#include "si5351.h"

int hal_i2c_write(uint8_t addr, uint8_t reg, const uint8_t *data, size_t len)
{
    (void)addr, (void)reg, (void)data, (void)len;
    return 0;
}

int main(int argc, char **argv)
{
    if (argc != 5) {
        fprintf(stderr, "usage: si5351_dump START STOP STEP CAL_OFFSET\n");
        return 2;
    }
    uint32_t start = (uint32_t)strtoul(argv[1], 0, 10), stop = (uint32_t)strtoul(argv[2], 0, 10);
    uint32_t step = (uint32_t)strtoul(argv[3], 0, 10);
    int32_t cal = (int32_t)strtol(argv[4], 0, 10);
    for (uint32_t f = start; f <= stop; f += step) {
        lo_plan_t p;
        uint8_t reg[SI5351_REG_COUNT];
        if (lo_plan(f, cal != 0, cal, &p) != LO_OK) return 1;
        si5351_image(&p, reg);
        printf("%" PRIu32 " %d ", f, p.path);
        for (int k = 0; k < SI5351_REG_COUNT; k++) printf("%02x", reg[k]);
        printf("\n");
    }
    return 0;
}
```

`firmware/CMakeLists.txt (append)`:

```cmake
add_executable(lo_plan_dump tests/lo_plan_dump.c)
target_link_libraries(lo_plan_dump PRIVATE elf_core)
add_executable(si5351_dump tests/si5351_dump.c)
target_link_libraries(si5351_dump PRIVATE elf_core)
```

`Makefile (`fw-test`: add the pytest line)`:

```make
	cd firmware && $(PYTHON) -m pytest -q tests
```

- [ ] **Step 4: Run them to verify they pass**

Run: `cd sim && python3 -m pytest -q tests/test_si5351_decode.py && cd .. && make fw-test`
Expected: `5 passed`; then ctest `6/6` and pytest `3 passed` (the grid comparison and both register runs: 53 901 plans each, 0 mismatches).

- [ ] **Step 5: Commit**

```bash
git add sim/neosim/si5351_decode.py sim/tests/test_si5351_decode.py firmware Makefile
git commit -m "test(fw): C planner equals lo_plan.py on the grid; register images decode independently"
```

### Task 3: Verify and README

- [ ] **Step 1:** `make test` (includes `fw-test`). Expected: all pass.
- [ ] **Step 2:** README: tick "Host build of the firmware modules with unit tests"; add the feature line for the firmware core and its cross-checks. Commit.
