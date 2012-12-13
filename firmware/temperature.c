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
#include <string.h>
#include <util/delay.h>
#include "temperature.h"
#include "led.h"
#include "radio.h"

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
    TWCR |= _BV(TWPS1);
    TWBR = 1;

    // Enable the I2C interface and enable interrupts
    TWCR &= ~(_BV(TWSTO) | _BV(TWSTA));
    TWCR |= _BV(TWEN) | _BV(TWIE);
}

/**
 * Turn the I2C interface 'off'.
 */
void temperature_deinit()
{
    TWCR &= ~(_BV(TWEN) | _BV(TWIE) | _BV(TWSTA) | _BV(TWSTO));
}

/**
 * Return the current temperature.
 */
float temperature_read()
{
    // Set the pointer register to the temperature register
    temperature_init();
    tmp100_send_byte(TMP100_PTR_TMP);

    // Empty the buffer and reset the pointer and counter
    ptr = tbuf;
    counter = 0;
    tmp100_read();
    temperature_deinit();

    // Construct the value to be returned
    uint16_t tmp = (tbuf[0] << 8) | tbuf[1];
    return (float)(tmp >> 4) * 0.0625;
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
    TWCR &= ~_BV(TWSTO);
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
    TWCR &= ~_BV(TWSTO);
    TWCR |= _BV(TWINT) | _BV(TWSTA);

    while(tw_in_progress);
}

/**
 * ISR for the I2C interface when it fires an interrupt for whatever reason.
 */
ISR(TWI_vect)
{
    uint8_t twsr = TWSR;
    // What we do depends on the value of the status register
    switch(twsr)
    {
        // Start or repeated start, transmit SLAW or SLAR now
        case TW_START_SENT:
        case TW_RPT_START_SENT:
            if(tw_byte_tx != 0xFF) // if we are wanting to transmit (master txer)
                TWDR = TMP100_ADDR | 0;
            else // if we want to receive data (master rxer)
                TWDR = TMP100_ADDR | 1;

            // Clear the start/stop bit generator and continue the transfer
            TWCR &= ~(_BV(TWSTA) | _BV(TWSTO));
            TWCR |= _BV(TWINT) | _BV(TWEN);
            break;

        // SLAW has been ack'ed, transmit a DATA packet to the device
        case TW_SLAW_ACK:
            TWDR = tw_byte_tx;
            TWCR |= _BV(TWINT) | _BV(TWEN);
            break;

        case TW_SLAR_ACK:
            // Enable ACKs from the AVR for incoming bytes
            TWCR |= _BV(TWINT) | _BV(TWEA) | _BV(TWEN);
            break;

        case TW_WDATA_ACK:
            // Byte successfully transmitted, send a stop condition
            // TODO: We can only ever transmit a single byte!
            TWCR |= _BV(TWINT) | _BV(TWSTO) | _BV(TWEN);
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
            // TODO: We always receive two bytes, this should not be hardcoded
            *ptr++ = TWDR;
            *ptr++ = 0x00;
            counter++;
            tw_in_progress = false;
            break;

        // No relevant state information, TWINT=0. Why are we here?
        case TW_NO_STATE_INFO:
            break;

        // Bus error due to illegal start/stop condition. Bus is released by
        // hardware.
        case TW_BUS_ERROR:
            led_set(LED_RED, 1);

        default:
            led_set(LED_RED, 1);
            break;
    }
    return;
}

