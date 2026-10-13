/* hal.h over the virtual hardware, with manual time and a scripted I2S source. */
#ifndef ELF_TEST_HAL_H
#define ELF_TEST_HAL_H

#include "vhw.h"

extern vhw_t test_vhw;
extern uint32_t test_now_ms;

void test_hal_reset(void);
/* Queues frames the next hal_i2s_read calls return (interleaved I, Q). */
void test_i2s_queue(const int32_t *iq, size_t frames);

#endif
