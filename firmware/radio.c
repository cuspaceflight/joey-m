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
#include <avr/eeprom.h>
#include "stdbool.h"
#include "global.h"
#include "led.h"

uint16_t _radio_shift = 0x0000;
uint16_t _delay = 10000;

volatile uint16_t _dac_value = 0;
volatile uint8_t sample = 0;
volatile int32_t _transition_delta = 0;
volatile uint16_t _transition_start = 0;
volatile bool transition_complete = false;

const uint8_t step[50] = {0,  5,  10,  15,  20,  25,  30,  35,  40,  45,  50,  55,  60,  65,  70,  75,  80,  85,  90,  95,  100,  105,  110,  115,  120,  125,  130,  135,  140,  145,  150,  155,  160,  165,  170,  175,  180,  185,  190,  195,  200,  205,  210,  215,  220,  225,  230,  235,  240,  245};

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

    // Enable SPI, set master mode and fosc/2, giving an SPI
    // interface SCK speed of 8MHz
    SPSR |= _BV(SPI2X);
    SPCR &= ~(_BV(SPR0) | _BV(SPR1));
    SPCR |= _BV(MSTR) | _BV(SPE);

    // Set up TIMER0 to tick once per symbol and interrupt
    // CTC mode
    TCCR0A |= _BV(WGM01);

    // Prescale by 1024
    //TCCR0B |= _BV(CS02) | _BV(CS00);

    // Interrupt on compare match with OCR0A
    TIMSK0 |= _BV(OCIE0A);
    OCR0A = 156;

    // Set up TIMER2 for the DSP (!) stuff
    // No clock prescale to get 62.5kHz sample rate with an 8 bit timer
    TCCR2B |= _BV(CS20);

    // Do not interrupt on overflow for now
    TIMSK2 &= ~(_BV(TOIE2));

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

    // Write cmd then value to the SPI data register
    SPDR = 0x30 | (channel & 0x01);
    while(!(SPSR & _BV(SPIF)));
    SPDR = value >> 8;
    while(!(SPSR & _BV(SPIF)));
    SPDR = value & 0xFF;
    while(!(SPSR & _BV(SPIF)));

    // Raise SS to signal end of transaction
    RADIO_PORT |= _BV(RADIO_SS);

    // Update the DAC value
    if(channel == RADIO_FINE) _dac_value = value;
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
 * Transition using the FIR filter
 */
void _radio_transition(uint16_t target)
{
    if( _dac_value == target ) return;

    // Update the global target and initial
    _transition_start = _dac_value;
    _transition_delta = (int32_t)target - (int32_t)_transition_start;
    transition_complete = false;

    // Start the DSP timer interrupting
    TIMSK2 |= _BV(TOIE2);

    // Wait until transition finished
    while( !transition_complete );
}

/**
 * Interrupt handle for the radio timer
 */
ISR(TIMER0_COMPA_vect)
{
    _radio_dac_write(RADIO_FINE, _dac_value);
    _dac_value = 0x0F00 - _dac_value;
}

/**
 * Interrupt handler for the DSP timer. Read out the next step
 * response value and write it to the DAC.
 */
ISR(TIMER2_OVF_vect)
{
    led_set(LED_GREEN, 1);
    if( sample < DSP_SAMPLES )
    {
        int32_t d = _transition_delta * (int32_t)(step[sample]);
        d /= 256;
        d += (int32_t)_transition_start;
        _radio_dac_write(RADIO_FINE, (uint16_t)d);
        sample++;
    } else {
        TIMSK2 &= ~(_BV(TOIE2));
        sample = 0;
        transition_complete = true;
    }
}
