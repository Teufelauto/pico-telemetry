# pico-telemetry
Vehicle telemetry logging for the Raspberry Pi Pico written in microPython for now. Future plans for C++
## Required hardware:
 * Raspberry Pi Pico
 * microSD Card with its full size adapter to have pins soldered to, or an SD daughter board that actually functions at 3.3v
 * BerryGPS-IMU-4 - This board has an m8c u-blox gps receiver and an LSM6DSL inertial module
 * Future plans for MCP3208 12-bit Analog to Digital Converter to gather sensor data
## Library Dependancies
These libraries are required in the pico's lib folder:
 * UBX_PICO.py file created in this repository for certain communications with the M8C gps module
 * [microPython driver sdcard.py](https://github.com/micropython/micropython/tree/master/drivers/sdcard)
 * [ozzmaker's IMU_I2C.py](https://github.com/ozzmaker/BerryIMU/tree/master/PicoMicroPython) and [LSM6DSL.py](https://github.com/ozzmaker/BerryIMU/tree/master/PicoMicroPython) required to communicate with the IMU.
One of the following:
 * [Peter Hinch's as_GPS.py](https://github.com/peterhinch/micropython-async/tree/master/v3/as_drivers/as_GPS) for the async version.
 * [Calvin McCoy's micropyGPS.py](https://github.com/inmcm/micropyGPS) if choosing the non-async version. Version to be depreciated due to performance issues without async.
## UBX_PICO.py
This file has code to speak the language of the u-blox GPS receiver.

Includes initial setup of the u-blox m8c GPS for:
 * Cold Reboot (for troubleshooting)
 * Setting update rate of GPS to 1, 2, 5, or 10hz
 * Setting the baud rate of the gps
 * Disabling extranious NMEA messages that we don't need, and take precious time to transmit over UART
 * Disabling the RMC message (the most useful NMEA message) if the ubx protocol is used
 * Disabling extranious ubx messages that are not needed to reduce UART clutter
 * Enabling the ubx-pvt message, which is similar to NMEA's RMC, but in the u-blox's native format

Includes a function to calculate the checksum of ubx messages, and if run by itself, can be used to check and output checksums by placing the byte-code in question towards the top of the file after "message = ".

Eventually, this file will handle the ubx-pvt messages, much as micropyGPS or as_GPS do for NMEA, and those files will be unneeded.
