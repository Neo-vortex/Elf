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
