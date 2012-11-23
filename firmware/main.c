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
#include <avr/interrupt.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <avr/wdt.h>

#include "led.h"
#include "radio.h"
#include "gps.h"
#include "temperature.h"

// 30kHz range on COARSE, 3kHz on FINE

char s[100];
uint32_t EEMEM ticks = 0;

int main()
{
    // Disable, configure, and start the watchdog timer
    wdt_disable();
    wdt_reset();
    wdt_enable(WDTO_8S);

    // Start and configure all hardware peripherals
    sei();
    led_init();
    temperature_init();
    radio_init();
    gps_init();
    radio_enable();

    // FIXME
    // Some bits for testing the temp sensor
    led_set(LED_RED, 1);
    temperature_read();
    led_set(LED_RED, 0);
    while(1);

    // Set the radio shift and baud rate & chatter a bit so we can find Joey
    _radio_dac_write(RADIO_COARSE, RADIO_CENTER_FREQ_434630);
    _radio_dac_write(RADIO_FINE, 0);
    radio_set_shift(RADIO_SHIFT_425);
    radio_set_baud(RADIO_BAUD_50);
    for(uint8_t i = 0; i < 5; i++)
    {
        radio_chatter();
        wdt_reset();
    }

    int32_t lat = 0, lon = 0, alt = 0;
    uint8_t hour = 0, minute = 0, second = 0, lock = 0, sats = 0;

    while(true)
    {
        led_set(LED_GREEN, 1);

        // Get the current system tick and increment
        uint32_t tick = eeprom_read_dword(&ticks) + 1;

        // Get temperature from the TMP102
        //int16_t temperature = temperature_read();
        int16_t temperature = 0;

        // Check that we're in airborne <1g mode
        if( gps_check_nav() != 0x06 ) led_set(LED_RED, 1);

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
            tick, hour, minute, second, lat_fmt, lon_fmt, alt, temperature,
            sats, lock);
        radio_chatter();
        radio_transmit_sentence(s);
        radio_chatter();

        led_set(LED_RED, 0);
        eeprom_update_dword(&ticks, tick);
        wdt_reset();
        _delay_ms(500);
    }

    return 0;
}
