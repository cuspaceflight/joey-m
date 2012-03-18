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

// 30kHz range on COARSE, 3kHz on FINE

int main()
{
    led_init();
    led_set(LED_GREEN, 1);
    radio_init();
    radio_enable();
    _radio_dac_off();
    _delay_ms(1000);

    _radio_dac_write(RADIO_COARSE, 0x0000);

    while(true)
    {
//        led_set(LED_RED, 1);
        _delay_ms(1000);
//        led_set(LED_RED, 0);
        _delay_ms(1000);
    }

    return 0;
}
