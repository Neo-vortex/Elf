/* USB application glue: UAC2 class requests, I/Q streaming and CAT over CDC. Shared by the
 * target and the virtual board. */
#ifndef ELF_USB_APP_H
#define ELF_USB_APP_H

#include <stdbool.h>

void usb_app_init(void);
/* Call every 1 ms: moves one frame's I/Q (96 frames) from the audio ring to the USB FIFO while
 * the host streams; otherwise drains the ring so stale samples never reach a new stream. */
void usb_app_frame(void);
bool usb_app_streaming(void);

#endif
