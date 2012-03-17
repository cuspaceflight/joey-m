/**
 * JOEY-M by CU Spaceflight
 *
 * This file is part of the JOEY-M project by Cambridge University Spaceflight.
 *
 * Jon Sowman 2012
 */

#ifndef __GLOBAL_H__
#define __GLOBAL_H__

#define LED_DDR         DDRB
#define LED_PORT        PORTB
#define LED_RED         1
#define LED_GREEN       0

#define RADIO_COURSE
#define RADIO_FINE
#define RADIO_EN        2
#define RADIO_EN_DDR    DDRC
#define RADIO_EN_PORT   PORTC

#define RADIO_SS_PORT   PORTB
#define RADIO_SS_DDR    DDRB
#define RADIO_SS        2

#define RADIO_DAC_A     0
#define RADIO_DAC_B     1
#define RADIO_FINE      RADIO_DAC_A
#define RADIO_COARSE    RADIO_DAC_B

#endif /* __GLOBAL_H__ */
