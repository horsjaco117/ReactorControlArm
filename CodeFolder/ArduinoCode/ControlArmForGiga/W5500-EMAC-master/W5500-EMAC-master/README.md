# Mbed OS Ethernet MAC driver for the W5500 

This Arduino library uses [W5500 driver](https://github.com/njh/W5500MacRaw) by Nicholas Humfrey with some adaptation.

Arduino Giga should work with Arduino Ethernet 2 shield. Arduino Portenta H7 boards should work with MKR ETH shield. 

If you use a nodule with W5500, wire it to standard SPI pins of the board. On Nano pins 10 to 13. On Portenta pins 8, 9, 10 and SC pin 5 (as the MKR ETH Shield). On Giga R1 use the SPI header and pin 10 as CS. 

For Arduino Giga R1, the Ethernet library has to be copied from Mbed Portenta boards package. For Mbed Core Nano boards and RP2040 boards, Ethernet and SocketWrapper library have to be copied.

To use the driver include it in the sketch with the Portenta Ethernet library:

```
#include <W5500EMAC.h>
#include <PortentaEthernet.h>
``````
