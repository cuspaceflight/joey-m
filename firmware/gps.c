/**
 * JOEY-M by CU Spaceflight
 *
 * This file is part of the JOEY-M project by Cambridge University Spaceflight.
 *
 * Jon Sowman 2012
 */

#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>
#include "led.h"
#include "global.h"
#include "gps.h"
#include "radio.h"

/**
 * Set up USART0 for communication with the uBlox GPS
 * at 38400 baud.
 */
void gps_init(void)
{
    // Double the transmission speed
    UCSR0A |= _BV(U2X0);

    // Set 8N1
    UCSR0C |= _BV(UCSZ01) | _BV(UCSZ00);

    // Set baud rate to 38400
    UBRR0H = 0x00;
    UBRR0L = 0x33;

    // Enable the receiver and transmitter
    UCSR0B |= _BV(TXEN0) | _BV(RXEN0);
}

/**
 * Poll the GPS for a position message then extract the useful
 * information from it.
 */
void gps_get_position(int32_t* lat, int32_t* lon, int32_t* alt)
{
    // Request a NAV-POSLLH message from the GPS
    uint8_t request[8] = {0xB5, 0x62, 0x01, 0x02, 0x00, 0x00, 0x03,
        0x0A};
    _gps_send_msg(request, 8);
    
    uint8_t buf[36];
    for(uint8_t i = 0; i < 36; i++)
        buf[i] = _gps_get_byte();

    // Verify the sync and header bits
    if( buf[0] != 0xB5 || buf[1] != 0x62 )
        led_set(LED_RED, 1);
    if( buf[2] != 0x01 || buf[3] != 0x02 )
        led_set(LED_RED, 1);

    // 4 bytes of longitude (1e-7)
    *lon = (int32_t)buf[10] | (int32_t)buf[11] << 8 | 
        (int32_t)buf[12] << 16 | (int32_t)buf[13] << 24;
    
    // 4 bytes of latitude (1e-7)
    *lat = (int32_t)buf[14] | (int32_t)buf[15] << 8 | 
        (int32_t)buf[16] << 16 | (int32_t)buf[17] << 24;
    
    // 4 bytes of altitude above MSL (mm)
    *alt = (int32_t)buf[22] | (int32_t)buf[23] << 8 | 
        (int32_t)buf[24] << 16 | (int32_t)buf[25] << 24;

    if( !_gps_verify_checksum(&buf[2], 32) ) led_set(LED_RED, 1);
}

/**
 * Get the hour, minute and second from the GPS using the NAV-TIMEUTC
 * message.
 */
void gps_get_time(uint8_t* hour, uint8_t* minute, uint8_t* second)
{
    // Send a NAV-TIMEUTC message to the receiver
    uint8_t request[8] = {0xB5, 0x62, 0x01, 0x21, 0x00, 0x00,
        0x22, 0x67};
    _gps_send_msg(request, 8);

    // Get the message back from the GPS
    uint8_t buf[28];
    for(uint8_t i = 0; i < 28; i++)
        buf[i] = _gps_get_byte();

    // Verify the sync and header bits
    if( buf[0] != 0xB5 || buf[1] != 0x62 )
        led_set(LED_RED, 1);
    if( buf[2] != 0x01 || buf[3] != 0x21 )
        led_set(LED_RED, 1);

    *hour = buf[22];
    *minute = buf[23];
    *second = buf[24];

    if( !_gps_verify_checksum(&buf[2], 24) ) led_set(LED_RED, 1);
}

/**
 * Check the navigation status to determine the quality of the
 * fix currently held by the receiver with a NAV-STATUS message.
 */
uint8_t gps_check_lock(void)
{
    // Construct the request to the GPS
    uint8_t request[8] = {0xB5, 0x62, 0x01, 0x03, 0x00, 0x00,
        0x04, 0x0D};
    _gps_send_msg(request, 8);

    // Get the message back from the GPS
    uint8_t buf[24];
    for(uint8_t i = 0; i < 24; i++)
        buf[i] = _gps_get_byte();

    // Verify the sync and header bits
    if( buf[0] != 0xB5 || buf[1] != 0x62 )
        led_set(LED_RED, 1);
    if( buf[2] != 0x01 || buf[3] != 0x03 )
        led_set(LED_RED, 1);

    if( !_gps_verify_checksum(&buf[2], 20) ) led_set(LED_RED, 1);

    return buf[10];
}

/**
 * Return the number of satellites the receiver is currently
 * tracking using the RXM-SVSI message.
 */
uint8_t gps_num_sats(void)
{
    uint8_t request[8] = {0xB5, 0x62, 0x02, 0x20, 0x00, 0x00, 
        0x22, 0x68};
    _gps_send_msg(request, 8);

    // Get the message back from the GPS
    // The length of the message varies depending on the number of
    // sats we are tracking. We should really clock in all of the
    // message and verify the checksum. This solution is lazy.
    uint8_t buf[15];
    for(uint8_t i = 0; i < 15; i++)
        buf[i] = _gps_get_byte();

    // Force flush since there's a tonne of data remaining
    _gps_flush_buffer();
    
    // Verify the sync and header bits
    if( buf[0] != 0xB5 || buf[1] != 0x62 )
        led_set(LED_RED, 1);
    if( buf[2] != 0x02 || buf[3] != 0x20 )
        led_set(LED_RED, 1);

    /*char s[20];
    sprintf(s, "%x %x %x %x\n", buf[0], buf[1], buf[2], buf[3]);
    radio_transmit_string(s);*/

    // FIXME: We should check the UBX checksum here

    return buf[13];
}

/**
 * Verify the checksum for the given data and length.
 */
bool _gps_verify_checksum(uint8_t* data, uint8_t len)
{
    uint8_t a, b;
    gps_ubx_checksum(data, len, &a, &b);
    if( a != *(data + len) || b != *(data + len + 1))
        return false;
    else
        return true;
}

/**
 * Calculate a UBX checksum using 8-bit Fletcher (RFC1145)
 */
void gps_ubx_checksum(uint8_t* data, uint8_t len, uint8_t* cka,
        uint8_t* ckb)
{
    *cka = 0;
    *ckb = 0;
    for( uint8_t i = 0; i < len; i++ )
    {
        *cka += *data;
        *ckb += *cka;
        data++;
    }
}

/**
 * Send a binary message to the GPS of length len.
 */
void _gps_send_msg(uint8_t* data, uint8_t len)
{
    _gps_flush_buffer();
    for(uint8_t i = 0; i < len; i++)
    {
        while( !( UCSR0A & (1<<UDRE0)) );
        UDR0 = *data;
        data++;
    }
    while( !(UCSR0A & (1<<UDRE0)) );
}

/**
 * Receive a single byte from the GPS and return it.
 */
uint8_t _gps_get_byte(void)
{
    // Wait until we have received a byte
    while( !(UCSR0A & _BV(RXC0)) );
    return UDR0;
}

/**
 * Flush the USART recieve buffer.
 */
void _gps_flush_buffer(void)
{
    uint8_t dummy;
    while ( UCSR0A & _BV(RXC0) ) dummy = UDR0;
}
