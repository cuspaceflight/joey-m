/**
 * JOEY-M by CU Spaceflight
 *
 * This file is part of the JOEY-M project by Cambridge University Spaceflight.
 *
 * Jon Sowman 2012
 */

#include <avr/io.h>
#include <avr/delay.h>
#include "global.h"
#include "led.h"

/**
 * Initialise the radio subsystem including the dual 16 bit 
 * DAC.
 */
void radio_init(void)
{
    // Configure the slave select pin and set it high
    RADIO_SS_DDR |= _BV(RADIO_SS);
    RADIO_SS_PORT |= _BV(RADIO_SS);

    // Set MOSI and SCK as output, MISO as input
    RADIO_SS_DDR |= _BV(3) | _BV(5);
    RADIO_SS_DDR &= ~(_BV(4));

    // Set MSB first, sample on rising edge, 
    // clock idles low
    SPCR &= ~(_BV(DORD) | _BV(CPOL) | _BV(CPHA));

    // Disable SCK frequency double
    SPSR |= _BV(SPI2X);

    // Enable SPI, set master mode and fosc/64
    SPCR |= _BV(SPR0) | _BV(MSTR) | _BV(SPE);
}

/**
 * Enable the power amplifier on the Micrel radio
 */
void radio_enable(void)
{
    RADIO_EN_DDR |= _BV(RADIO_EN);
    RADIO_EN_PORT |= _BV(RADIO_EN);
}

/**
 * Disable the power amp on the Micrel radio to shut it down
 */
void radio_disable(void)
{
    RADIO_EN_DDR |= _BV(RADIO_EN);
    RADIO_EN_PORT &= ~(_BV(RADIO_EN));
}

/**
 * Write a value to one of the DAC channels
 */
void _radio_dac_write(uint8_t channel, uint16_t value)
{
    // Take SS low
    RADIO_SS_PORT &= ~(_BV(RADIO_SS));

    // Construct the command and address byte
    uint8_t cmd = 0x30 | (channel & 0x01);

    led_set(LED_GREEN, 1);

    // Write cmd then value to the SPI data register
    SPDR = cmd;
    while(!(SPSR & _BV(SPIF)));
    led_set(LED_GREEN, 0);
    SPDR = value >> 8;
    while(!(SPSR & _BV(SPIF)));
    SPDR = value & 0xFF;
    while(!(SPSR & _BV(SPIF)));

    // Raise SS to signal end of transaction
    RADIO_SS_PORT |= _BV(RADIO_SS);
}
