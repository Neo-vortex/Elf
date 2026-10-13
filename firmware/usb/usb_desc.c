/* USB descriptors: composite device with an IAD per function.
 * Audio (UAC2): clock source (internal, fixed 96 kHz) → input terminal (2 channels: I = front
 * left, Q = front right) → output terminal (USB streaming); one streaming interface with a
 * zero-bandwidth alternate 0 and alternate 1 = 24-bit PCM in 3-byte subslots, asynchronous
 * isochronous IN. CDC-ACM for CAT. */
#include <string.h>

#include "tusb.h"
#include "usb_desc.h"

static const tusb_desc_device_t device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = ELF_USB_VID,
    .idProduct = ELF_USB_PID,
    .bcdDevice = ELF_USB_BCD,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};

const uint8_t *tud_descriptor_device_cb(void) { return (const uint8_t *)&device; }

#define AUDIO_AC_LEN (TUD_AUDIO20_DESC_CLK_SRC_LEN + TUD_AUDIO20_DESC_INPUT_TERM_LEN + TUD_AUDIO20_DESC_OUTPUT_TERM_LEN)
#define AUDIO_DESC_LEN                                                                                  \
    (TUD_AUDIO20_DESC_IAD_LEN + TUD_AUDIO20_DESC_STD_AC_LEN + TUD_AUDIO20_DESC_CS_AC_LEN + AUDIO_AC_LEN + \
     2 * TUD_AUDIO20_DESC_STD_AS_LEN + TUD_AUDIO20_DESC_CS_AS_INT_LEN + TUD_AUDIO20_DESC_TYPE_I_FORMAT_LEN + \
     TUD_AUDIO20_DESC_STD_AS_ISO_EP_LEN + TUD_AUDIO20_DESC_CS_AS_ISO_EP_LEN)
#define CONFIG_LEN (TUD_CONFIG_DESC_LEN + AUDIO_DESC_LEN + TUD_CDC_DESC_LEN)
#define IQ_CHANNELS (AUDIO20_CHANNEL_CONFIG_FRONT_LEFT | AUDIO20_CHANNEL_CONFIG_FRONT_RIGHT)

static const uint8_t configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_TOTAL, 0, CONFIG_LEN, TUSB_DESC_CONFIG_ATT_SELF_POWERED, 100),   /* the jack powers the board */
    TUD_AUDIO20_DESC_IAD(ITF_AUDIO_CONTROL, 2, 0),
    TUD_AUDIO20_DESC_STD_AC(ITF_AUDIO_CONTROL, 0, 4),
    TUD_AUDIO20_DESC_CS_AC(0x0200, AUDIO20_FUNC_MICROPHONE, AUDIO_AC_LEN, 0),
    TUD_AUDIO20_DESC_CLK_SRC(UAC2_ENTITY_CLOCK, AUDIO20_CLOCK_SOURCE_ATT_INT_FIX_CLK,
                             (AUDIO20_CTRL_R << AUDIO20_CLOCK_SOURCE_CTRL_CLK_FRQ_POS), UAC2_ENTITY_INPUT_TERMINAL, 0),
    TUD_AUDIO20_DESC_INPUT_TERM(UAC2_ENTITY_INPUT_TERMINAL, AUDIO_TERM_TYPE_IN_GENERIC_MIC, 0, UAC2_ENTITY_CLOCK,
                                ELF_AUDIO_CHANNELS, IQ_CHANNELS, 0, 0, 0),
    TUD_AUDIO20_DESC_OUTPUT_TERM(UAC2_ENTITY_OUTPUT_TERMINAL, AUDIO_TERM_TYPE_USB_STREAMING, 0,
                                 UAC2_ENTITY_INPUT_TERMINAL, UAC2_ENTITY_CLOCK, 0, 0),
    TUD_AUDIO20_DESC_STD_AS_INT(ITF_AUDIO_STREAMING, 0, 0, 0),
    TUD_AUDIO20_DESC_STD_AS_INT(ITF_AUDIO_STREAMING, 1, 1, 0),
    TUD_AUDIO20_DESC_CS_AS_INT(UAC2_ENTITY_OUTPUT_TERMINAL, AUDIO20_CTRL_NONE, AUDIO20_FORMAT_TYPE_I,
                               AUDIO20_DATA_FORMAT_TYPE_I_PCM, ELF_AUDIO_CHANNELS, IQ_CHANNELS, 0),
    TUD_AUDIO20_DESC_TYPE_I_FORMAT(ELF_AUDIO_BYTES, 24),
    TUD_AUDIO20_DESC_STD_AS_ISO_EP(EP_AUDIO_IN,
                                   (uint8_t)(TUSB_XFER_ISOCHRONOUS | TUSB_ISO_EP_ATT_ASYNCHRONOUS | TUSB_ISO_EP_ATT_DATA),
                                   ELF_AUDIO_EP_SIZE, 1),
    TUD_AUDIO20_DESC_CS_AS_ISO_EP(AUDIO20_CS_AS_ISO_DATA_EP_ATT_NON_MAX_PACKETS_OK, AUDIO20_CTRL_NONE,
                                  AUDIO20_CS_AS_ISO_DATA_EP_LOCK_DELAY_UNIT_UNDEFINED, 0),
    TUD_CDC_DESCRIPTOR(ITF_CDC, 5, EP_CDC_NOTIF, 8, EP_CDC_OUT, EP_CDC_IN, 64),
};

_Static_assert(sizeof configuration == CONFIG_LEN, "configuration length");

const uint8_t *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    return configuration;
}

static const char *const strings[] = {
    NULL,                       /* 0: language */
    "Elf",                      /* 1: manufacturer */
    "Elf HF Receiver",          /* 2: product (the ALSA card name) */
    "ELF0001",                  /* 3: serial */
    "Elf I/Q",                  /* 4: audio function */
    "Elf CAT",                  /* 5: CDC */
};

const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;
    static uint16_t out[32];
    size_t n;
    if (index == 0) {
        out[1] = 0x0409;
        n = 1;
    } else {
        if (index >= sizeof strings / sizeof strings[0]) return NULL;
        const char *s = strings[index];
        n = strlen(s);
        if (n > 31) n = 31;
        for (size_t k = 0; k < n; k++) out[1 + k] = (uint8_t)s[k];
    }
    out[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * n + 2));
    return out;
}
