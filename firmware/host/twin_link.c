#include "twin_link.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "audio.h"
#include "hal_virtual.h"

#define FIFO_FRAMES 65536u                /* power of two */
#define REQUEST_FRAMES 4800u              /* 50 ms per request */
#define LOW_WATER 9600u                   /* keep ≈ 100 ms queued */

static int fd = -1;
static int32_t fifo[FIFO_FRAMES][2];
static uint32_t head, tail, requested, short_frames;
static uint8_t rx[16 + 8 * REQUEST_FRAMES];
static size_t rx_len;
static uint32_t last_writes = UINT32_MAX, last_relays = UINT32_MAX;
static int last_path = -1;

int twin_link_open(const char *path)
{
    struct sockaddr_un a = {.sun_family = AF_UNIX};
    if (strlen(path) >= sizeof a.sun_path) return -1;
    strcpy(a.sun_path, path);
    fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    if (connect(fd, (struct sockaddr *)&a, sizeof a)) {
        close(fd);
        fd = -1;
        return -1;
    }
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK);
    return 0;
}

bool twin_link_up(void) { return fd >= 0; }
uint32_t twin_link_short_frames(void) { return short_frames; }

static void put32le(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24); }
static uint32_t get32le(const uint8_t *p) { return p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24; }

static void lost(void)
{
    fprintf(stderr, "elf-virtual: twin link lost\n");
    close(fd);
    fd = -1;
}

/* small control messages: block until sent (the socket buffer always has room for them) */
static void send_all(const uint8_t *p, size_t n)
{
    while (n && fd >= 0) {
        ssize_t w = send(fd, p, n, MSG_NOSIGNAL);
        if (w < 0 && (errno == EAGAIN || errno == EINTR)) continue;
        if (w <= 0) {
            lost();
            return;
        }
        p += w;
        n -= (size_t)w;
    }
}

static void send_state(void)
{
    uint8_t m[1 + 256 + 5];
    m[0] = 'S';
    memcpy(m + 1, vboard.si5351, 256);
    put32le(m + 257, vhw_relay_outputs(&vboard));
    m[261] = vboard.gpio[HAL_GPIO_LO_PATH] ? 1 : 0;
    send_all(m, sizeof m);
    /* the new state's I/Q replaces whatever is queued for the old one */
    tail = head;
}

static void receive(void)
{
    for (;;) {
        ssize_t r = recv(fd, rx + rx_len, sizeof rx - rx_len, 0);
        if (r < 0 && (errno == EAGAIN || errno == EINTR)) break;
        if (r <= 0) {
            lost();
            return;
        }
        rx_len += (size_t)r;
        /* complete D messages */
        while (rx_len >= 5 && rx[0] == 'D') {
            uint32_t n = get32le(rx + 1);
            size_t need = 5 + 8 * (size_t)n;
            if (rx_len < need) break;
            for (uint32_t k = 0; k < n; k++) {
                if (head - tail < FIFO_FRAMES) {
                    fifo[head % FIFO_FRAMES][0] = (int32_t)get32le(rx + 5 + 8 * k);
                    fifo[head % FIFO_FRAMES][1] = (int32_t)get32le(rx + 9 + 8 * k);
                    head++;
                }
            }
            requested -= n < requested ? n : requested;
            memmove(rx, rx + need, rx_len - need);
            rx_len -= need;
        }
        if (rx_len >= 1 && rx[0] != 'D') {
            lost();
            return;
        }
    }
}

void twin_link_frame(void)
{
    int32_t iq[2 * AUDIO_FRAMES_PER_MS];
    if (fd >= 0) {
        int path = vboard.gpio[HAL_GPIO_LO_PATH];
        uint32_t relays = vhw_relay_outputs(&vboard);
        if (vboard.si5351_writes != last_writes || relays != last_relays || path != last_path) {
            last_writes = vboard.si5351_writes;
            last_relays = relays;
            last_path = path;
            send_state();
        }
        if (fd >= 0) receive();
        if (fd >= 0 && requested == 0 && head - tail < LOW_WATER) {
            uint8_t m[5] = {'R'};
            put32le(m + 1, REQUEST_FRAMES);
            send_all(m, sizeof m);
            requested = REQUEST_FRAMES;
        }
    }
    for (unsigned k = 0; k < AUDIO_FRAMES_PER_MS; k++) {
        if (head != tail) {
            iq[2 * k] = fifo[tail % FIFO_FRAMES][0];
            iq[2 * k + 1] = fifo[tail % FIFO_FRAMES][1];
            tail++;
        } else {
            iq[2 * k] = iq[2 * k + 1] = 0;
            short_frames++;
        }
    }
    hal_virtual_i2s_push(iq, AUDIO_FRAMES_PER_MS);
}
