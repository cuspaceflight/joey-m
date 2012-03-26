/**
 * JOEY-M by CU Spaceflight
 *
 * This file is part of the JOEY-M project by Cambridge University Spaceflight.
 *
 * Jon Sowman 2012
 */

#ifndef __LED_H__
#define __LED_H__

#define LED_DDR         DDRB
#define LED_PORT        PORTB
#define LED_RED         1
#define LED_GREEN       0

void led_init();
void led_set();

#endif /* __LED_H__ */
