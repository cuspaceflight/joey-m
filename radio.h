/**
 * JOEY-M by CU Spaceflight
 *
 * This file is part of the JOEY-M project by Cambridge University Spaceflight.
 *
 * Jon Sowman 2012
 */
#include <avr/io.h>

void radio_init(void);
void radio_enable(void);
void radio_disable(void);
void _radio_dac_write(uint8_t channel, uint16_t value);
