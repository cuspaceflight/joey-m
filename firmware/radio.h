/**
 * JOEY-M by CU Spaceflight
 *
 * This file is part of the JOEY-M project by Cambridge University Spaceflight.
 *
 * Jon Sowman 2012
 */

#ifndef __RADIO_H__
#define __RADIO_H__

#include <avr/io.h>

void radio_init(void);
void radio_enable(void);
void radio_disable(void);
void _radio_dac_write(uint8_t channel, uint16_t value);
void _radio_dac_off(void);
void radio_transmit_string(char* string);
void _radio_transmit_bit(uint8_t data, uint8_t ptr);
void radio_set_shift(uint16_t shift);
void radio_set_baud(uint8_t baud);
void _radio_transition(uint16_t target);

#endif /* __RADIO_H__ */
