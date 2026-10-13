/* Relay chain: 3 × 74HC595 → 3 × ULN2803 → 24 coils. Bit k of the word is chain bit k
 * (74HC595 #k/8, output Q(k%8)), in the order of controller.json (tables_gen.h). */
#ifndef ELF_RELAYS_H
#define ELF_RELAYS_H

#include <stdbool.h>
#include <stdint.h>

/* The word for a filter (1–11, 0 = bank muted: every filter released), attenuator and preamp. */
uint32_t relays_word(int filter, bool atten, bool preamp);

/* Shifts the word into the chain (bit 23 first, so bit 0 lands on the first register's QA),
 * latches it and enables the outputs (/OE low). */
void relays_write(uint32_t word);

/* Power-up: outputs disabled (/OE high, as the pull-up holds them) and an all-released word
 * latched before they are enabled. */
void relays_init(void);

#endif
