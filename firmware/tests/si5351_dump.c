/* Prints the Si5351 register image for every LO on a grid: "<f> <path> <256 hex bytes>". */
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

#include "si5351.h"

int hal_i2c_write(uint8_t addr, uint8_t reg, const uint8_t *data, size_t len)
{
    (void)addr, (void)reg, (void)data, (void)len;
    return 0;
}

int main(int argc, char **argv)
{
    if (argc != 5) {
        fprintf(stderr, "usage: si5351_dump START STOP STEP CAL_OFFSET\n");
        return 2;
    }
    uint32_t start = (uint32_t)strtoul(argv[1], 0, 10), stop = (uint32_t)strtoul(argv[2], 0, 10);
    uint32_t step = (uint32_t)strtoul(argv[3], 0, 10);
    int32_t cal = (int32_t)strtol(argv[4], 0, 10);
    for (uint32_t f = start; f <= stop; f += step) {
        lo_plan_t p;
        uint8_t reg[SI5351_REG_COUNT];
        if (lo_plan(f, cal != 0, cal, &p) != LO_OK) return 1;
        si5351_image(&p, reg);
        printf("%" PRIu32 " %d ", f, p.path);
        for (int k = 0; k < SI5351_REG_COUNT; k++) printf("%02x", reg[k]);
        printf("\n");
    }
    return 0;
}
