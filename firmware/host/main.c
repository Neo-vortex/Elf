/* elf-virtual: the Elf firmware running as a Linux process, its USB device served over USB/IP.
 *
 *   elf-virtual [--port N] [--twin SOCKET | --tone HZ] [--seconds S] [--population china|iran]
 *
 * --twin takes the ADC's I/Q from the digital twin (python -m neosim vboard STATIONS --socket
 * SOCKET), rendered for whatever the firmware set the hardware to; --tone feeds a complex tone at
 * HZ instead (I = cos, Q = sin, −6 dBFS; the default, 1 kHz); --seconds exits after S seconds
 * (tests); --population builds the virtual board as China (INA180A1 bias sense, the default) or
 * Iran (BC857B mirror). Attach with
 *   sudo modprobe vhci-hcd && sudo usbip attach -r 127.0.0.1 -b 1-1 */
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "audio.h"
#include "hal_virtual.h"
#include "radio.h"
#include "supervisor.h"
#include "tables_gen.h"
#include "tusb.h"
#include "twin_link.h"
#include "usb_app.h"
#include "usbip_server.h"

static volatile sig_atomic_t stop;

static void on_signal(int sig)
{
    (void)sig;
    stop = 1;
}

/* board state the supervisor reads: a 12 V adapter on the jack, 5V_MAIN in regulation, a USB
 * host connected (the monitors see the node voltages through the dividers in power.json); the bias sense
 * reads the zero of the circuit the population fits (vhw_set_population) */
static void set_supplies(void)
{
    vboard.adc_mv[HAL_ADC_SENSE_EXT] = (uint32_t)(11600ull * 1000 / TABLES_SENSE_DIV_EXT_X1000);
    vboard.adc_mv[HAL_ADC_SENSE_MAIN] = (uint32_t)(5480ull * 1000 / TABLES_SENSE_DIV_MAIN_X1000);
    vboard.adc_mv[HAL_ADC_SENSE_VBUS] = (uint32_t)(5000ull * 1000 / TABLES_SENSE_DIV_VBUS_X1000);
}

static void tone_frame(double hz, uint64_t frame)
{
    int32_t iq[2 * AUDIO_FRAMES_PER_MS];
    const double a = 4194304.0;                   /* 2^22: −6 dBFS of 24 bits */
    for (unsigned k = 0; k < AUDIO_FRAMES_PER_MS; k++) {
        double t = (double)(frame * AUDIO_FRAMES_PER_MS + k) / AUDIO_RATE_HZ;
        iq[2 * k] = (int32_t)lround(a * cos(2 * M_PI * hz * t));
        iq[2 * k + 1] = (int32_t)lround(a * sin(2 * M_PI * hz * t));
    }
    hal_virtual_i2s_push(iq, AUDIO_FRAMES_PER_MS);
}

int main(int argc, char **argv)
{
    unsigned port = USBIP_PORT;
    double tone_hz = 1000.0, seconds = 0;
    const char *twin = NULL, *population = "china";
    for (int k = 1; k < argc; k++) {
        if (!strcmp(argv[k], "--port") && k + 1 < argc) port = (unsigned)atoi(argv[++k]);
        else if (!strcmp(argv[k], "--tone") && k + 1 < argc) tone_hz = atof(argv[++k]);
        else if (!strcmp(argv[k], "--seconds") && k + 1 < argc) seconds = atof(argv[++k]);
        else if (!strcmp(argv[k], "--twin") && k + 1 < argc) twin = argv[++k];
        else if (!strcmp(argv[k], "--population") && k + 1 < argc) population = argv[++k];
        else {
            fprintf(stderr, "usage: elf-virtual [--port N] [--twin SOCKET | --tone HZ] [--seconds S] "
                            "[--population china|iran]\n");
            return 2;
        }
    }
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    signal(SIGPIPE, SIG_IGN);

    hal_virtual_init();
    if (vhw_set_population(&vboard, population)) {
        fprintf(stderr, "elf-virtual: unknown population %s (china or iran)\n", population);
        return 2;
    }
    set_supplies();
    radio_init();
    supervisor_init();
    audio_init();
    usb_app_init();
    tusb_rhport_init_t dev_init = {.role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_FULL};
    tusb_init(0, &dev_init);
    if (twin && twin_link_open(twin)) {
        fprintf(stderr, "elf-virtual: cannot reach the twin at %s\n", twin);
        return 1;
    }
    if (usbip_server_start((uint16_t)port)) {
        perror("elf-virtual: listen");
        return 1;
    }
    printf("elf-virtual: USB/IP on 127.0.0.1:%u, bus id 1-1\n", port);
    fflush(stdout);

    uint64_t frames = 0;
    while (!stop) {
        uint64_t now = hal_virtual_now_ms();
        if (seconds > 0 && now >= (uint64_t)(seconds * 1000)) break;
        while (frames < now) {                     /* catch up after a slow iteration */
            if (twin) twin_link_frame();
            else tone_frame(tone_hz, frames);
            audio_pump();
            usb_app_frame();
            usbip_frame();
            if (frames % 10 == 0) supervisor_poll();
            frames++;
        }
        usbip_poll(1);
        tud_task();
    }
    const usbip_stats_t *s = usbip_stats();
    printf("elf-virtual: imports %u, urbs %u/%u, unlinks %u, iso packets %u (%u empty), audio underruns %u, "
           "twin silence frames %u\n", s->imports, s->urbs_completed, s->urbs_received, s->unlinks, s->iso_packets,
           s->iso_empty_packets, audio_stats()->underruns, twin_link_short_frames());
    return 0;
}
