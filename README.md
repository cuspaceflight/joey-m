JOEY-M - CU Spaceflight
====================

Joey-M is an experimental flight computer produced as part of the Wombat flight
computer project at Cambridge University Spaceflight.  

It uses a twin varactor crystal pulling arrangement to change the transmit
frequency of a Micrel MICRF112YMM 434MHz ISM band transmitter, along with a dual
DAC from Linear Technology to control the varactor bias voltages.  

The DAC can be any of the LTC range in MSOP-8 package which share a footprint, 
including the LTC1661/1662 (10 bit), LTC2622 (12 bit), LTC2612 (14 bit)
and LTC2602 (16 bit).  

Other equipment on board is a uBlox NEO-6Q GPS, TMP100 12 bit temperature
sensor, and an Atmel ATMEGA328P MCU.  

Designed and released into the public domain by Jon Sowman - March 2012.  

[Contact CUSF](mailto:contact@cusf.co.uk)
