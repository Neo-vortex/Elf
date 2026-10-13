/* USB/IP server side of the virtual board's device controller (dcd_usbip.c). */
#ifndef ELF_USBIP_SERVER_H
#define ELF_USBIP_SERVER_H

#include <stdbool.h>
#include <stdint.h>

#define USBIP_PORT 3240

typedef struct {
    uint32_t imports, urbs_received, urbs_completed, unlinks, write_errors;
    uint32_t iso_packets, iso_empty_packets;
} usbip_stats_t;

/* Listens on 127.0.0.1:port; returns 0 or -1. */
int usbip_server_start(uint16_t port);
/* Waits up to timeout_ms for the client and handles one message. */
void usbip_poll(int timeout_ms);
/* The 1 ms frame tick: SOF and one isochronous packet per streaming endpoint. */
void usbip_frame(void);
bool usbip_connected(void);
const usbip_stats_t *usbip_stats(void);

#endif
