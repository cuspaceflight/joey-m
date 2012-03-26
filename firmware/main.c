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
#include "temperature.h"

// 30kHz range on COARSE, 3kHz on FINE

char s[100];

int main()
{
    led_init();
    radio_init();
    gps_init();
    radio_enable();
    _delay_ms(1000);

    // Set the I2C lines to Hi-Z for TMP102 testing
    DDRC &= ~(_BV(4) | _BV(5));

    // Set the radio shift and baud rate
    _radio_dac_write(RADIO_COARSE, 0xf000);
    _radio_dac_write(RADIO_FINE, 0);
    radio_set_shift(0x0600);
    radio_set_baud(RADIO_BAUD_300);

    int32_t lat = 0, lon = 0, alt = 0;
    uint32_t ticks = 0;
    uint8_t hour = 0, minute = 0, second = 0, lock = 0, sats = 0;

    while(true)
    {
        // Get temperature
        led_set(LED_GREEN, 1);
        //int16_t temperature = temperature_read();
        int16_t temperature = 0;

        // Get information from the GPS
        gps_check_lock(&lock, &sats);
        if( lock == 0x02 || lock == 0x03 || lock == 0x04 )
        {
            gps_get_position(&lat, &lon, &alt);
            gps_get_time(&hour, &minute, &second);
        }

        led_set(LED_GREEN, 0);
        
        // Format the telemetry string & transmit
        double lat_fmt = (double)lat / 10000000.0;
        double lon_fmt = (double)lon / 10000000.0;
        alt /= 1000;

        sprintf(s, "$$JOEY,%lu,%02u:%02u:%02u,%02.7f,%03.7f,%ld,%d,%u,%x",
            ticks, hour, minute, second, lat_fmt, lon_fmt, alt, temperature,
            sats, lock);
        radio_transmit_sentence(s);

        led_set(LED_RED, 0);
        ticks++;
        _delay_ms(1000);
    }

    return 0;
}
