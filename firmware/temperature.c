/**
 * JOEY-M by CU Spaceflight
 *
 * This file is part of the JOEY-M project by Cambridge University Spaceflight.
 *
 * Jon Sowman 2012
 */

#include "temperature.h"

/**
 * Set up the TMP100 temperature sensor.
 */
void temperature_init()
{
    // Set up the prescaler to give a clock frequency of 400kHz
    TWSR |= _BV(TWPS1);
    TWBR = 1;

    // Enable the I2C interface and enable interrupts
    TWCR |= _BV(TWEN) | _BV(TWIE);
}

/**
 * Return the current temperature.
 */
float temperature_read()
{
    // Become a master on the bus and transmit a start bit
    TWCR |= _BV(TWINT) | _BV(TWSTA);

    // Check that the start was successfully sent
}

/**
 * Interrupt for when the I2C interface fires an interrupt for whatever reason.
 */
ISR(TWI_vect)
{
    // What we do depends on the value of the status register
    switch(TWSR)
    {
        case TW_START_SENT:
            // Transmit SLA+W (R/W bit (LSB) is 0 for W)
            TWDR = TMP100_ADDR << 1;
            TWCR |= _BV(TWINT);
            break;

        default:
            break;
    }
    return;
}

