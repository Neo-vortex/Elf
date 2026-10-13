/* TinyUSB configuration for Elf: UAC2 stereo 24-bit 96 kHz capture (I/Q) + CDC-ACM (CAT),
 * full speed (the ESP32-S3's USB). The host build sets CFG_TUSB_MCU = OPT_MCU_NONE and runs
 * the device controller over USB/IP (host/dcd_usbip.c). */
#ifndef TUSB_CONFIG_H_
#define TUSB_CONFIG_H_

#ifndef CFG_TUSB_MCU
#error CFG_TUSB_MCU must be defined by the build
#endif
#if CFG_TUSB_MCU == OPT_MCU_NONE
#define TUP_DCD_ENDPOINT_MAX 8
#endif

#define CFG_TUSB_OS OPT_OS_NONE
#define CFG_TUSB_DEBUG 0
#define CFG_TUD_ENABLED 1
#define CFG_TUD_MAX_SPEED OPT_MODE_FULL_SPEED
#define CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_ALIGN __attribute__((aligned(4)))
#define CFG_TUD_ENDPOINT0_SIZE 64

#define CFG_TUD_CDC 1
#define CFG_TUD_AUDIO 1
#define CFG_TUD_MSC 0
#define CFG_TUD_HID 0
#define CFG_TUD_MIDI 0
#define CFG_TUD_VENDOR 0

#define CFG_TUD_CDC_RX_BUFSIZE 256
#define CFG_TUD_CDC_TX_BUFSIZE 256
#define CFG_TUD_CDC_EP_BUFSIZE 64

/* audio: 2 channels (I, Q), 3-byte samples, 96 kHz, isochronous IN */
#define ELF_AUDIO_RATE 96000
#define ELF_AUDIO_CHANNELS 2
#define ELF_AUDIO_BYTES 3
#define ELF_AUDIO_EP_SIZE TUD_AUDIO_EP_SIZE(0, ELF_AUDIO_RATE, ELF_AUDIO_BYTES, ELF_AUDIO_CHANNELS)
#define CFG_TUD_AUDIO_ENABLE_EP_IN 1
#define CFG_TUD_AUDIO_ENABLE_EP_OUT 0
#define CFG_TUD_AUDIO_EP_IN_FLOW_CONTROL 1
#define CFG_TUD_AUDIO_FUNC_1_EP_IN_SZ_MAX ELF_AUDIO_EP_SIZE
#define CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ (8 * ELF_AUDIO_EP_SIZE)
#define CFG_TUD_AUDIO_CTRL_BUF_SZ 64

#endif
