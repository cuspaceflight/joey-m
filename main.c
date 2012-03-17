/**
 * JOEY-M by CU Spaceflight
 *
 * This file is part of the JOEY-M project by Cambridge University Spaceflight.
 *
 * Jon Sowman 2012
 */
#include <avr/io.h>
#include <avr/delay.h>
#include <stdbool.h>

#include "led.h"
#include "radio.h"
#include "global.h"

int main()
{
    led_init();
    radio_init();
    radio_enable();

    while(true) {
        led_set(LED_RED, 1);
        _radio_dac_write(RADIO_COARSE, 0x00);
        _delay_ms(500);
        led_set(LED_RED, 0);
        _radio_dac_write(RADIO_COARSE, 0x7FFF);
        _delay_ms(500);
    }

    return 0;
}
