/**
 * JOEY-M by CU Spaceflight
 *
 * This file is part of the JOEY-M project by Cambridge University Spaceflight.
 *
 * Jon Sowman 2012
 */

#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <string.h>
#include <stdbool.h>

#include "led.h"
#include "radio.h"
#include "global.h"
#include "gps.h"

// 30kHz range on COARSE, 3kHz on FINE

char s[100];

int main()
{
    led_init();
    radio_init();
    gps_init();
    radio_enable();
    _delay_ms(1000);

    // Set the radio shift and baud rate
    radio_set_shift(0x0600);
    radio_set_baud(RADIO_BAUD_300);

    _radio_dac_write(RADIO_COARSE, 0xf000);
    _radio_dac_write(RADIO_FINE, 0);

    int32_t lat = 0;
    int32_t lon = 0;
    int32_t alt = 0;
    uint8_t hour = 0;
    uint8_t minute = 0;
    uint8_t second = 0;

    while(true)
    {
        led_set(LED_GREEN, 1);

        // Get information from the GPS
        gps_get_position(&lat, &lon, &alt);
        gps_get_time(&hour, &minute, &second);
        uint8_t lock = gps_check_lock();
        uint8_t sats = gps_num_sats();

        led_set(LED_GREEN, 0);

        // Format the telemetry string & transmit
        sprintf(s, "$$JOEY,%02u:%02u:%02u,%ld,%ld,%ld,%u,%x\n",
            hour, minute, second, lat, lon, 
            alt, sats, lock);
        radio_transmit_string(s);

        led_set(LED_RED, 0);
        _delay_ms(1000);
    }

    return 0;
}
