/**
 * JOEY-M by CU Spaceflight
 *
 * This file is part of the JOEY-M project by Cambridge University Spaceflight.
 *
 * Jon Sowman 2012
 */

#include <stdbool.h>
#include "temperature.h"

volatile bool tw_in_progress = false;
uint8_t tw_byte_tx;
volatile uint8_t tbuf[5];
volatile uint8_t* ptr = tbuf;

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
 * Turn the I2C interface 'off'.
 */
void temperature_deinit()
{
    TWCR &= ~(_BV(TWEN) | _BV(TWIE));
}

/**
 * Return the current temperature.
 */
float temperature_read()
{
}

void temperature_send_byte(uint8_t b)
{
    tw_in_progress = true;
    tw_byte_tx = b;

    // Clear the interrupt flag and begin the tranmission process
    TWCR |= _BV(TWINT) | _BV(TWSTA);
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
            if(tw_byte_tx) // if we are wanting to transmit (master txer)
                TWDR = TMP100_ADDR | 0;
            else // if we want to receive data (master rxer)
                TWDR = TMP100_ADDR | 1;
            TWCR |= _BV(TWINT);
            break;

        case TW_SLAW_ACK:
            // Transmit a data packet
            TWDR = tw_byte_tx;
            break;

        case TW_SLAR_ACK:
            // Enable ACKs from the AVR for incoming bytes
            TWCR |= _BV(TWINT) | _BV(TWEA);
            break;

        case TW_WDATA_ACK:
            // Byte successfully transmitted, send a stop condition
            TWCR |= _BV(TWINT) | _BV(TWSTO);
            tw_in_progress = false;
            break;

        case TW_RDATA_ACK:
            // There is a byte to be read and we ack'ed it
            *ptr++ = TWDR;
            TWCR |= _BV(TWINT) | _BV(TWEA);
            break;

        default:
            break;
    }
    return;
}

