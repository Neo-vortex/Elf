/* hal.h for the virtual board: the parts are vhw.c models; I2S frames come from a source the
 * main loop feeds once per 1 ms frame (a test tone, or the digital twin). */
#ifndef ELF_HAL_VIRTUAL_H
#define ELF_HAL_VIRTUAL_H

#include <stddef.h>
#include <stdint.h>

#include "vhw.h"

extern vhw_t vboard;

void hal_virtual_init(void);
/* Queues frames for hal_i2s_read (interleaved I, Q, signed 24-bit in int32). */
void hal_virtual_i2s_push(const int32_t *iq, size_t frames);
/* Milliseconds since hal_virtual_init. */
uint64_t hal_virtual_now_ms(void);

#endif
