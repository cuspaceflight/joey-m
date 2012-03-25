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
#include "i2c.h"

/**
 * Set the I2C peripheral up for a given SCL frequency
 * and enable the transmitter/receiver.
 */
void temperature_init()
{
    // Set the SCL frequency to 100kHz
    i2cSetBitrate(100);

    // Enable the I2C peripheral
    TWCR |= _BV(TWEN);
}

/**
 * Get the current temperature from the onboard TMP102
 * sensor and return it.
 *
 * \return The temperature in degrees Celcius
 */
int16_t temperature_read()
{
    i2cInit();
    // Send a start condition and wait until sent
    /*TWCR = _BV(TWSTA) | _BV(TWINT) | _BV(TWEN);
    while( !(TWCR & _BV(TWINT)) );
    if( (TWSR & 0xF8) != 0x08 ) led_set(LED_RED, 1);

    // Send SLA_R, wait for it to be sent, check status
    TWDR = 0b10010001;
    TWCR = _BV(TWINT) | _BV(TWEN);
    while( !(TWCR & _BV(TWINT)) );
    //if( (TWSR & 0xF8) != 0x40 ) led_set(LED_RED, 1);*/

start:
    i2cSendStart();
    i2cWaitForComplete();
    uint8_t twsr = TWSR & 0xF8;
   // if( twsr != 0x08 && twsr != 0x10 ) led_set(LED_RED, 1);

    i2cSendByte(0b10010000);
    i2cWaitForComplete();
    twsr = TWSR & 0xF8;
    if( twsr == 0x20 ) 
    {
        TWCR = _BV(TWSTO) | _BV(TWINT);
        goto start;
    }
        
    led_set(LED_RED, 0);

    /*char s[10];
    sprintf(s, "%X\n", TWSR & 0xF8);
    radio_transmit_string(s);*/

    // Clear TWINT and wait for it to go high, then read in data
    /*TWCR = _BV(TWINT) | _BV(TWEA) | _BV(TWEN);
    while( !(TWCR & _BV(TWINT)) );
    uint8_t msb = TWDR;
    TWCR = _BV(TWINT) | _BV(TWEN); // NACK this
    while( !(TWCR & _BV(TWINT)) );
    uint8_t lsb = TWDR;*/
    
    i2cReceiveByte(TRUE);
    i2cWaitForComplete();
    uint8_t msb = i2cGetReceivedByte(); //Read the MSB data
    i2cWaitForComplete();

    i2cReceiveByte(FALSE);
    i2cWaitForComplete();
    uint8_t lsb = i2cGetReceivedByte(); //Read the LSB data
    i2cWaitForComplete();
    
    // Send a stop condition
    //TWCR = _BV(TWSTO) | _BV(TWINT) | _BV(TWEN);
    i2cSendStop();

    int16_t data = ((msb << 8) | lsb) >> 4;
    if( data & _BV(11) ) data |= 0xF800;
    return data / 16;
}


