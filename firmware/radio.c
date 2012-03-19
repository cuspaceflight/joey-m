/**
 * JOEY-M by CU Spaceflight
 *
 * This file is part of the JOEY-M project by Cambridge University Spaceflight.
 *
 * Jon Sowman 2012
 */

#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
#include "global.h"
#include "led.h"

uint16_t _dac_value = 0x0000;
uint16_t _radio_shift = 0x0000;
uint16_t _delay = 10000;

/**
 * Initialise the radio subsystem including the dual 16 bit 
 * DAC.
 */
void radio_init(void)
{
    // Configure the slave select pin and set it high
    RADIO_DDR |= _BV(RADIO_SS) | _BV(RADIO_MOSI) | _BV(RADIO_SCK);
    RADIO_DDR &= ~(_BV(RADIO_MISO));
    RADIO_PORT |= _BV(RADIO_SS);

    // Set MSB first, sample on rising edge, 
    // clock idles low
    SPCR &= ~(_BV(DORD) | _BV(CPOL) | _BV(CPHA));

    // Enable SPI, set master mode and fosc/16
    SPSR |= _BV(SPI2X);
    SPCR |= _BV(SPR0) | _BV(MSTR) | _BV(SPE);

    // Set up TIMER0 to tick once per symbol and interrupt
    // CTC mode
    TCCR0A |= _BV(WGM01);

    // Prescale by 1024
    //TCCR0B |= _BV(CS02) | _BV(CS00);

    // Interrupt on compare match with OCR0A
    TIMSK0 |= _BV(OCIE0A);
    OCR0A = 156;

    // Enable global interrupts
    sei();
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
    RADIO_PORT &= ~(_BV(RADIO_SS));

    // Construct the command and address byte
    uint8_t cmd = 0x30 | (channel & 0x01);

    // Write cmd then value to the SPI data register
    SPDR = cmd;
    while(!(SPSR & _BV(SPIF)));
    SPDR = value >> 8;
    while(!(SPSR & _BV(SPIF)));
    SPDR = value & 0xFF;
    while(!(SPSR & _BV(SPIF)));

    // Raise SS to signal end of transaction
    RADIO_PORT |= _BV(RADIO_SS);
}

/**
 * Power down the DAC and set the outputs to a high-Z state
 */
void _radio_dac_off(void)
{
    // Take SS low
    RADIO_PORT &= ~(_BV(RADIO_SS));

    // Write cmd then value to the SPI data register
    SPDR = 0x4F;
    while(!(SPSR & _BV(SPIF)));
    SPDR = 0;
    while(!(SPSR & _BV(SPIF)));
    SPDR = 0;
    while(!(SPSR & _BV(SPIF)));

    // Raise SS to signal end of transaction
    RADIO_PORT |= _BV(RADIO_SS);
}

void radio_transmit_string(char* string, uint8_t len)
{
    while(*string)
    {
        _radio_transmit_byte(*string);
        string++;
    }
    _radio_transmit_byte('\n');
}

void _radio_transmit_byte(char data)
{
    // Start bit
    _radio_dac_write(RADIO_FINE, 0x0000);
    _delay_us(_delay);

    // Write the data bits
    uint8_t i = 0;
    for(i = 0; i < 8; i++)
    {
        if( (data >> i) & 0x01 )
            _radio_dac_write(RADIO_FINE, _radio_shift);
        else
            _radio_dac_write(RADIO_FINE, 0x0000);
        _delay_us(_delay);
    }

    // And two stop bits
    _radio_dac_write(RADIO_FINE, _radio_shift);
    _delay_us(_delay);
    _radio_dac_write(RADIO_FINE, _radio_shift);
    _delay_us(_delay);
}

/**
 * Set the radio shift
 */
void radio_set_shift(uint16_t shift)
{
    _radio_shift = shift;
}

/**
 * Calculate the delay required for the given baud rate and store
 */
void radio_set_baud(uint16_t baud)
{
    _delay = 1000000UL/baud;
}

/**
 * Interrupt handle for the radio timer
 */
ISR(TIMER0_COMPA_vect)
{
    _radio_dac_write(RADIO_FINE, _dac_value);
    _dac_value = 0x0F00 - _dac_value;
}
