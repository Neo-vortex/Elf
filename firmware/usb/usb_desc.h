#ifndef ELF_USB_DESC_H
#define ELF_USB_DESC_H

/* pid.codes test VID/PID (https://pid.codes/1209/0001/) until the project has its own */
#define ELF_USB_VID 0x1209
#define ELF_USB_PID 0x0001
#define ELF_USB_BCD 0x0100

enum { ITF_AUDIO_CONTROL = 0, ITF_AUDIO_STREAMING, ITF_CDC, ITF_CDC_DATA, ITF_TOTAL };
enum { EP_AUDIO_IN = 0x81, EP_CDC_NOTIF = 0x82, EP_CDC_OUT = 0x03, EP_CDC_IN = 0x83 };
enum { UAC2_ENTITY_INPUT_TERMINAL = 0x01, UAC2_ENTITY_OUTPUT_TERMINAL = 0x03, UAC2_ENTITY_CLOCK = 0x04 };

#endif
