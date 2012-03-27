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

#define RADIO_EN        2
#define RADIO_EN_DDR    DDRC
#define RADIO_EN_PORT   PORTC

#define RADIO_PORT      PORTB
#define RADIO_DDR       DDRB
#define RADIO_MOSI      3
#define RADIO_MISO      4
#define RADIO_SCK       5
#define RADIO_SS        2

#define RADIO_DAC_A     0
#define RADIO_DAC_B     1
#define RADIO_FINE      RADIO_DAC_B
#define RADIO_COARSE    RADIO_DAC_A

#define RADIO_BAUD_50   156
#define RADIO_BAUD_300  26
#define RADIO_CENTER_FREQ_433975    0XE800
#define RADIO_SHIFT_425 0x0600

#define DSP_SAMPLES     50
#define DSP_OFFSET      0

void radio_init(void);
void radio_enable(void);
void radio_disable(void);
void _radio_dac_write(uint8_t channel, uint16_t value);
void _radio_dac_off(void);
void radio_transmit_sentence(char* string);
void radio_transmit_string(char* string);
void _radio_transmit_bit(uint8_t data, uint8_t ptr);
uint16_t radio_calculate_checksum(char* data);
void radio_set_shift(uint16_t shift);
void radio_set_baud(uint8_t baud);
void _radio_transition(uint16_t target);
void radio_chatter(void);

#endif /* __RADIO_H__ */
