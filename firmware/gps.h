/**
 * JOEY-M by CU Spaceflight
 *
 * This file is part of the JOEY-M project by Cambridge University Spaceflight.
 *
 * Jon Sowman 2012
 */

#ifndef __GPS_H__
#define __GPS_H__

void gps_init(void);
void gps_get_position(int32_t* lat, int32_t* lon, uint16_t* alt);
void gps_ubx_checksum(uint8_t* data, uint8_t len, uint8_t* cka, uint8_t* ckb);
uint8_t _gps_get_byte(void);
void _gps_flush_buffer(void);

#endif /*__GPS_H__ */
