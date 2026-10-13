/* The virtual board's link to the digital twin (sim/neosim/vboard.py): sends the hardware state
 * the firmware set (Si5351 registers, relay outputs, LO path select) whenever it changes, and
 * feeds the virtual CS5361 with the I/Q the twin renders for it. */
#ifndef ELF_TWIN_LINK_H
#define ELF_TWIN_LINK_H

#include <stdbool.h>
#include <stdint.h>

/* Connects to the twin's Unix socket; returns 0 or -1. */
int twin_link_open(const char *socket_path);
/* One 1 ms frame: state update if changed, read what has arrived, push 96 frames to the ADC
 * (silence when the twin has not delivered yet). */
void twin_link_frame(void);
bool twin_link_up(void);
uint32_t twin_link_short_frames(void);   /* frames filled with silence */

#endif
