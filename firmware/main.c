/**
 * JOEY-M by CU Spaceflight
 *
 * This file is part of the JOEY-M project by Cambridge University Spaceflight.
 *
 * Jon Sowman 2012
 */
#include <avr/io.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <string.h>
#include <stdbool.h>

#include "led.h"
#include "radio.h"
#include "global.h"

// 30kHz range on COARSE, 3kHz on FINE

char s[30];

int main()
{
    led_init();
    radio_init();
    radio_enable();
    _radio_dac_off();
    _delay_ms(1000);

    // Set the radio shift and baud rate
    radio_set_shift(0x0600);
    radio_set_baud(RADIO_BAUD_300);

    _radio_dac_write(RADIO_COARSE, 0xf000);
    _radio_dac_write(RADIO_FINE, 0);

    strcpy(s, "U");

    while(true)
    {
        led_set(LED_RED, 1);
        radio_transmit_string(s);
        led_set(LED_RED, 0);
    }

    return 0;
}
