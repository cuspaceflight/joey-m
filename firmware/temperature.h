/**
 * JOEY-M by CU Spaceflight
 *
 * This file is part of the JOEY-M project by Cambridge University Spaceflight.
 *
 * Jon Sowman 2012
 */

#ifndef __TEMPERATURE_H__
#define __TEMPERATURE_H__

#define     TMP100_ADDR             0b10010110

// Pointer registers in the TMP100
#define     TMP100_PTR_TMP          0x00
#define     TMP100_PTR_CFG          0x01

// Status codes for the TWI
#define     TW_START_SENT           0x08
#define     TW_RPT_START_SENT       0x10
#define     TW_SLAW_ACK             0x18
#define     TW_SLAW_NACK            0x20
#define     TW_WDATA_ACK            0x28
#define     TW_WDATA_NACK           0x30
#define     TW_SLAx_DATA_ARBLOST    0x38
#define     TW_SLAR_ACK             0x40
#define     TW_SLAR_NACK            0x48
#define     TW_RDATA_ACK            0x50
#define     TW_RDATA_NACK           0x58
#define     TW_NO_STATE_INFO        0xF8
#define     TW_BUS_ERROR            0x00

void temperature_init(void);
float temperature_read(void);
void tmp100_send_byte(uint8_t b);
void tmp100_read(void);

#endif /* __TEMPERATURE_H__ */
