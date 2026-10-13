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
    test_vhw.adc_mv[HAL_ADC_SENSE_VBUS] = 2500;               /* host present */
    test_vhw.adc_mv[HAL_ADC_SENSE_EXT] = 1091;               /* 12 V jack through 100 k / 10 k */
    test_vhw.adc_mv[HAL_ADC_SENSE_MAIN] = 2000;              /* 4.0 V: the buck not up yet */
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
    CHECK(strcmp(ask("ZB1;"), "?;") == 0);                 /* 5V_MAIN not in regulation */
    test_vhw.adc_mv[HAL_ADC_SENSE_MAIN] = 2740;              /* 5.48 V */
    CHECK(strcmp(ask("ZB1;ZB;"), "ZB1;") == 0);
    const char *t = ask("ZT;");
    CHECK(strncmp(t, "ZT5000,12001,5480,", 18) == 0 && t[strlen(t) - 1] == ';');
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
