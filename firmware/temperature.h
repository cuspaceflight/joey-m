/**
 * JOEY-M by CU Spaceflight
 *
 * This file is part of the JOEY-M project by Cambridge University Spaceflight.
 *
 * Jon Sowman 2012
 */

#ifndef __TEMPERATURE_H__
#define __TEMPERATURE_H__

#define TMP100_ADDR         1001011

// Status codes for the onewire interface
#define     TW_START_SENT           0x08
#define     TW_RPT_START_SENT       0x10
#define     TW_SLAW_ACK             0x18
#define     TW_SLAW_NACK            0x20
#define     TW_DATA_ACK             0x28
#define     TW_DATA_NACK            0x30
#define     TW_SLAW_DATA_ARBLOST    0x38

void temperature_init(void);
float temperature_read(void);

#endif /* __TEMPERATURE_H__ */
