#include "usb_app.h"

#include "audio.h"
#include "cat.h"
#include "tusb.h"
#include "usb_desc.h"

static bool streaming;

static void cat_out(const char *s, size_t n)
{
    tud_cdc_write(s, (uint32_t)n);
    tud_cdc_write_flush();
}

void usb_app_init(void)
{
    streaming = false;
    cat_init(cat_out);
}

bool usb_app_streaming(void) { return streaming; }

void usb_app_frame(void)
{
    uint8_t pkt[AUDIO_PACKET_BYTES];
    audio_packet(pkt, AUDIO_FRAMES_PER_MS);
    if (streaming) tud_audio_write(pkt, AUDIO_PACKET_BYTES);
}

/* ---------- CDC: CAT ---------- */

void tud_cdc_rx_cb(uint8_t itf)
{
    (void)itf;
    uint8_t buf[64];
    uint32_t n;
    while ((n = tud_cdc_read(buf, sizeof buf)) > 0) cat_input(buf, n);
}

/* ---------- UAC2 ---------- */

bool tud_audio_set_itf_cb(uint8_t rhport, const tusb_control_request_t *req)
{
    (void)rhport;
    if (TU_U16_LOW(req->wIndex) == ITF_AUDIO_STREAMING) streaming = TU_U16_LOW(req->wValue) == 1;
    return true;
}

bool tud_audio_set_itf_close_ep_cb(uint8_t rhport, const tusb_control_request_t *req)
{
    (void)rhport;
    if (TU_U16_LOW(req->wIndex) == ITF_AUDIO_STREAMING) streaming = false;
    return true;
}

bool tud_audio_get_req_entity_cb(uint8_t rhport, const tusb_control_request_t *req)
{
    uint8_t entity = TU_U16_HIGH(req->wIndex), sel = TU_U16_HIGH(req->wValue);
    if (entity == UAC2_ENTITY_CLOCK && sel == AUDIO20_CS_CTRL_SAM_FREQ) {
        if (req->bRequest == AUDIO20_CS_REQ_CUR) {
            audio20_control_cur_4_t f = {.bCur = (int32_t)ELF_AUDIO_RATE};
            return tud_audio_buffer_and_schedule_control_xfer(rhport, req, &f, sizeof f);
        }
        if (req->bRequest == AUDIO20_CS_REQ_RANGE) {
            audio20_control_range_4_n_t(1) r = {.wNumSubRanges = 1};
            r.subrange[0].bMin = ELF_AUDIO_RATE;
            r.subrange[0].bMax = ELF_AUDIO_RATE;
            r.subrange[0].bRes = 0;
            return tud_audio_buffer_and_schedule_control_xfer(rhport, req, &r, sizeof r);
        }
    }
    if (entity == UAC2_ENTITY_CLOCK && sel == AUDIO20_CS_CTRL_CLK_VALID && req->bRequest == AUDIO20_CS_REQ_CUR) {
        audio20_control_cur_1_t v = {.bCur = 1};
        return tud_audio_buffer_and_schedule_control_xfer(rhport, req, &v, sizeof v);
    }
    if (entity == UAC2_ENTITY_INPUT_TERMINAL && sel == AUDIO20_TE_CTRL_CONNECTOR) {
        audio20_desc_channel_cluster_t c = {.bNrChannels = ELF_AUDIO_CHANNELS,
                                            .bmChannelConfig = (audio20_channel_config_t)(AUDIO20_CHANNEL_CONFIG_FRONT_LEFT |
                                                                                          AUDIO20_CHANNEL_CONFIG_FRONT_RIGHT),
                                            .iChannelNames = 0};
        return tud_audio_buffer_and_schedule_control_xfer(rhport, req, &c, sizeof c);
    }
    return false;                                   /* stall: not supported */
}

bool tud_audio_set_req_entity_cb(uint8_t rhport, const tusb_control_request_t *req, uint8_t *buf)
{
    (void)rhport;
    /* the clock is fixed: accept a host setting it to the only rate there is */
    uint8_t entity = TU_U16_HIGH(req->wIndex), sel = TU_U16_HIGH(req->wValue);
    if (entity == UAC2_ENTITY_CLOCK && sel == AUDIO20_CS_CTRL_SAM_FREQ && req->bRequest == AUDIO20_CS_REQ_CUR &&
        req->wLength == 4)
        return tu_unaligned_read32(buf) == ELF_AUDIO_RATE;
    return false;
}
