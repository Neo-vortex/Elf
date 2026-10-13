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
