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
#include <stdlib.h>

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
    _radio_dac_write(RADIO_COARSE, 0xf000);
    _radio_dac_write(RADIO_FINE, 0);
    radio_set_shift(0x0600);
    radio_set_baud(RADIO_BAUD_300);

    int32_t lat = 0, lon = 0, alt = 0;
    uint8_t hour = 0, minute = 0, second = 0;

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
        double lat_fmt = (double)lat / 10000000.0;
        double lon_fmt = (double)lon / 10000000.0;

        sprintf(s, "$$JOEY,%02u:%02u:%02u,%02.7f,%03.7f,%ld,%u,%x\n",
            hour, minute, second, lat_fmt, lon_fmt, alt, sats, lock);
        radio_transmit_string(s);

        led_set(LED_RED, 0);
        _delay_ms(1000);
    }

    return 0;
}
