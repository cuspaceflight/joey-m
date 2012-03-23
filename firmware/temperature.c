/**
 * JOEY-M by CU Spaceflight
 *
 * This file is part of the JOEY-M project by Cambridge University Spaceflight.
 *
 * Jon Sowman 2012
 */

#include <avr/io.h>
#include <stdio.h>
#include "global.h"
#include "led.h"
#include "radio.h"

/**
 * Set the I2C peripheral up for a given SCL frequency
 * and enable the transmitter/receiver.
 */
void temperature_init()
{
    // Set the SCL frequency to 100kHz
    TWBR = 72;

    // Enable the I2C peripheral
    TWCR |= _BV(TWEN);
}

/**
 * Get the current temperature from the onboard TMP102
 * sensor and return it.
 */
int16_t temperature_read()
{
    // Send a start condition and wait until sent
    TWCR |= _BV(TWSTA) | _BV(TWINT) | _BV(TWEN);
    while( !(TWCR & _BV(TWINT)) );
    if( (TWSR & 0xF8) != 0x08 ) led_set(LED_RED, 1);

    char s[10];
    sprintf(s, "%X\n", TWSR & 0xF8);
    radio_transmit_string(s);

    // Send SLA_R, wait for it to be sent, check status
    TWDR = 0b10010001;
    TWCR |= _BV(TWINT) | _BV(TWEN);
    while( !(TWCR & _BV(TWINT)) );
    //if( (TWSR & 0xF8) != 0x40 ) led_set(LED_RED, 1);

    // Clear TWINT and wait for it to go high, then read in data
    TWCR |= _BV(TWINT) | _BV(TWEA) | _BV(TWEN);
    while( !(TWCR & _BV(TWINT)) );
    uint8_t msb = TWDR;
    TWCR |= _BV(TWINT) | _BV(TWEN);
    while( !(TWCR & _BV(TWINT)) );
    uint8_t lsb = TWDR;

    // Send a stop condition and wait until sent
    TWCR |= _BV(TWSTO) | _BV(TWINT) | _BV(TWEN);
    while( !(TWCR & _BV(TWINT)) );

    int16_t data = ((msb << 8) | lsb) >> 4;
    if( data & _BV(11) ) data |= 0xF800;
    return data;
}


