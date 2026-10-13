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
