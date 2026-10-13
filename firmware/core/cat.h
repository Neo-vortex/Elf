/* CAT over CDC: a Kenwood TS-2000 subset (works with Hamlib's TS-2000 backend) plus Elf
 * extensions. Commands end with ';'; an unknown or invalid command answers "?;".
 *
 *   FA / FB   VFO A / B frequency in Hz, 11 digits (FA tunes the receiver; FB is stored only)
 *   IF        transceiver status (38 characters)
 *   ID        "ID019;" (TS-2000)
 *   MD        mode 1–9 (stored for the host's demodulator; the board is I/Q only)
 *   PA        preamp: PA0 / PA1 (answer "PAx0;")
 *   RA        attenuator: RA00 / RA01 (answer "RAxx00;")
 *   AI, PS, RX  accepted (AI0, PS1, receive)
 *   ZB        Elf: bias-T, ZB0 / ZB1 (refused with "?;" without ≥ 1.5 A or the jack)
 *   ZT        Elf: telemetry "ZT<vbus mV>,<ext mV>,<main mV>,<bias µA>,<temp m°C>,<flags>;"
 *   ZC        Elf: calibration tone, ZC0 / ZC1<offset Hz>, answer "ZC<0|1><offset>;" */
#ifndef ELF_CAT_H
#define ELF_CAT_H

#include <stddef.h>
#include <stdint.h>

typedef void (*cat_out_fn)(const char *s, size_t n);

void cat_init(cat_out_fn out);
void cat_input(const uint8_t *data, size_t n);

#endif
