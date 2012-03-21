/**
 * JOEY-M by CU Spaceflight
 *
 * This file is part of the JOEY-M project by Cambridge University Spaceflight.
 *
 * Jon Sowman 2012
 */

#include <avr/io.h>
#include "led.h"
#include "global.h"
#include "gps.h"

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
    // Flush the USART receive buffer
    _gps_flush_buffer();

    // Request a NAV-POSLLH message from the GPS
    uint8_t request[8] = {0xB5, 0x62, 0x01, 0x02, 0x00, 0x00, 0x03,
        0x0A};
    for( uint8_t i = 0; i < 8; i++ )
    {
        while( !( UCSR0A & (1<<UDRE0)) );
        UDR0 = request[i];
    }
    while( !(UCSR0A & (1<<UDRE0)) );
    
    // Now get the data back from the GPS
    uint8_t a, b;

    // Verify the sync bits
    a = _gps_get_byte();
    b = _gps_get_byte();
    if( a != 0xB5 || b != 0x62 ) led_set(LED_RED, 1);

    // Verify the packet header bytes
    a = _gps_get_byte();
    b = _gps_get_byte();
    if( a != 0x01 || b != 0x02 ) led_set(LED_RED, 1);

    // Get length field
    uint8_t len = _gps_get_byte();

    // Skip 4 bytes
    for( int8_t i = 3; i >= 0; i-- ) a = _gps_get_byte();

    // 4 bytes of latitude (1e-7)
    for( int8_t i = 3; i >= 0; i-- )
        *lat |= _gps_get_byte() << (8 * i);

    // 4 bytes of longitude (1e-7)
    for( int8_t i = 3; i >= 0; i-- )
        *lon |= _gps_get_byte() << (8 * i);
    
    // Skip 4 bytes
    for( int8_t i = 3; i >= 0; i-- ) a = _gps_get_byte();

    // 4 bytes of altitude above MSL (mm)
    for( int8_t i = 3; i >= 0; i-- )
        *alt |= _gps_get_byte() << (8 * i);

    // Flush the rest of the packet
    _gps_flush_buffer();
}

/**
 * Check the navigation status to determine the quality of the
 * fix currently held by the receiver.
 */
uint8_t gps_check_lock(void)
{
    // Flush the buffer
    _gps_flush_buffer();

    // Construct the request to the GPS
    uint8_t request[8] = {0xB5, 0x62, 0x01, 0x03, 0x00, 0x00,
        0x04, 0x0D};
    for( uint8_t i = 0; i < 8; i++ )
    {
        while( !( UCSR0A & (1<<UDRE0)) );
        UDR0 = request[i];
    }
    while( !(UCSR0A & (1<<UDRE0)) );
    
    // Now get the data back from the GPS
    uint8_t a, b;

    // Verify the sync bits
    a = _gps_get_byte();
    b = _gps_get_byte();
    if( a != 0xB5 || b != 0x62 ) led_set(LED_RED, 1);

    // Verify the packet header bytes
    a = _gps_get_byte();
    b = _gps_get_byte();
    if( a != 0x01 || b != 0x03 ) led_set(LED_RED, 1);

    // Drop length + 4 (i.e. 5) bytes
    for( int8_t i = 5; i >= 0; i-- ) a = _gps_get_byte();

    uint8_t fix = _gps_get_byte();
    return fix;
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
        *cka += *(data + i);
        *ckb += *cka;
    }
}

/**
 * Receive a single byte from the GPS and return it
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
