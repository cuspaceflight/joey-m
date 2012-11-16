/**
 * JOEY-M by CU Spaceflight
 *
 * This file is part of the JOEY-M project by Cambridge University Spaceflight.
 *
 * Jon Sowman 2012
 */

#include <avr/io.h>
#include <util/twi.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include "temperature.h"
#include "led.h"

volatile bool tw_in_progress = false;
uint8_t tw_byte_tx = 0xFF;
volatile uint8_t tbuf[3];
volatile uint8_t* ptr = tbuf;
volatile uint8_t counter = 0;

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
    // Set the pointer register to the temperature register
    tmp100_send_byte(0x00);

    // Empty the buffer and reset the pointer and counter
    ptr = tbuf;
    counter = 0;

    // Construct the value to be returned
    uint16_t tmp = (tbuf[0] << 8) | tbuf[1];
    return (float)(tmp >> 4);
}

/**
 * Send the given byte to the temperature sensor. This will only ever
 * be used to set the pointer register for now.
 */
void tmp100_send_byte(uint8_t b)
{
    tw_in_progress = true;
    tw_byte_tx = b;

    // Clear the interrupt flag and begin the tranmission process
    TWCR |= _BV(TWINT) | _BV(TWSTA);

    // Wait until the transmission is complete
    while(tw_in_progress);
}

/**
 * Read data from the TMP100, which will always be two bytes which comprise
 * the MSB and LSB of a left adjusted 12 bit conversion result.
 */
void tmp100_read()
{
    tw_in_progress = true;
    tw_byte_tx = 0xFF; // this is a a read op

    // Clear the interrupt flag and begin the tranmission process
    TWCR |= _BV(TWINT) | _BV(TWSTA);

    while(tw_in_progress);
}

/**
 * ISR for the I2C interface when it fires an interrupt for whatever reason.
 */
ISR(TWI_vect)
{
    // What we do depends on the value of the status register
    switch(TWSR)
    {
        case TW_START_SENT:
            if(tw_byte_tx != 0xFF) // if we are wanting to transmit (master txer)
                TWDR = TMP100_ADDR | 0;
            else // if we want to receive data (master rxer)
                TWDR = TMP100_ADDR | 1;
            TWCR |= _BV(TWINT);
            led_set(LED_RED, 0);
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
            counter++;
            if(counter == 1) // NACK the data
            {
                TWCR &= ~(_BV(TWEA));
                TWCR |= _BV(TWINT);
            } else { // ACK the data
                TWCR |= _BV(TWINT) | _BV(TWEA);
            }
            break;

        case TW_RDATA_NACK:
            // This is the final byte in the transmission, null terminate the
            // buffer
            *ptr++ = TWDR;
            *ptr++ = 0x00;
            counter++;
            tw_in_progress = false;
            break;

        default:
            break;
    }
    return;
}

