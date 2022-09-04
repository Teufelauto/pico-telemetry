# pico-telemetry
Vehicle telemetry logging for the Raspberry Pi Pico written in microPython 1.19.1 for now. Future plans are for migrating to C++ if Python proves to be a bottleneck. (I had a semester of 'Intro to C++' at university, so it should be easy! ;-) -Last words of a Mechanical Engineering Technologist.

This software is in very early development, and I'm very much an amateur. Do not expect a functioning datalogger for your race car. You will never be able to use this with MoTeC i2. Their file type is proprietary so you will buy their superior products. I do intend to eventually create a way to combine, at least some of, the data with a 60 fps onboard video. The [Road-Keeper](https://www.race-keeper.com/) would be a much better option than this project, but it's just a hobby. Use of these files is at your own risk.

 * telemetry_micropyGPS.py should be considered historical, and will not be receiving much, if any, updates.
## Required hardware:
 * Raspberry Pi Pico
 * microSD Card with its full size adapter to have pins soldered to, or an SD daughter board that actually functions at 3.3v
 * [BerryGPS-IMU-4](https://ozzmaker.com/berrygps-berrygps-imu-quick-start-guide/#) or other comprised of:
   - u-blox M8 gps receiver
   - LSM6DSL inertial module
 * Future plans for MCP3208 12-bit Analog to Digital Converter to gather sensor data
## Library Dependancies
Using Thonny, create a folder on the pico named "lib". These files are placed in the pico's lib folder:
 * UBX_PICO.py file in this repository for certain communications with the M8C gps module
 * [microPython driver sdcard.py](https://github.com/micropython/micropython/tree/master/drivers/sdcard)
 * [ozzmaker's IMU_I2C.py](https://github.com/ozzmaker/BerryIMU/tree/master/PicoMicroPython) and [LSM6DSL.py](https://github.com/ozzmaker/BerryIMU/tree/master/PicoMicroPython) required to communicate with the IMU. IMU_I2C.py must be modified to specify the correct pins on your Pico, as the functionality to change it programatically was not included in the driver.

   And one of the following:
 * [Peter Hinch's as_GPS.py](https://github.com/peterhinch/micropython-async/tree/master/v3/as_drivers/as_GPS) for the async version.
 * [Calvin McCoy's micropyGPS.py](https://github.com/inmcm/micropyGPS) if choosing the non-async version. Version deprecated due to change in focus, but has basic functionality.
## telemetry_async.py
Rename as main.py and place in Pico's root

Creates a file named {whateverYouWish}{yymmddhhmmss}.csv to the SD card after the GPS has initialized. GPS data is collected at 10hz thanks to stripping unnecessary messages from the UART stream. For 60hz telemetry recording, the IMU and various sensors must collect data 6 times per gps cycle. 

The second core of the RP2040 is used for writing to the SD card, since writing just 12 lines of data every 200ms into the .csv takes upwards of 50ms. 100 lines takes barely any more time to write, but the string concatenation is too time consuming with only 16ms available between data collection points. I'm currently using f-strings to concatenate. 

The convenience (and 'open source-iness' of the output file) of ASCII formatting may need to be cut to save processing time. The ubx-pvt message from the gps isn't ASCII, so once that's implemented, it's a natural progression to ditch ASCII. It will require an additional program (Python 3) to convert to a .csv so anyone can do anything with the data. 

The Pins used on the Pico may be easily changed, along with many other constant parameters toward the top of the code.
## UBX_PICO.py
Place in Pico's lib folder

This file has code to speak the language of the u-blox GPS receiver.

Includes initial setup of the u-blox m8c GPS for:
 * Cold Reboot (for troubleshooting)
 * Setting update rate of GPS to 1, 2, 5, 10 or 20hz. (20hz functionality unknown, but the byte-code is there)
 * Setting the baud rate of the gps
 * Disabling extranious NMEA messages that we don't need, and take precious time to transmit over UART
 * Disabling the RMC message (the most useful NMEA message) if the ubx protocol is used
 * Disabling extranious ubx messages that are not needed to reduce UART clutter
 * Enabling the ubx-pvt message, which is similar to NMEA's RMC, but in the u-blox's native format

Includes a function to calculate the checksum of ubx messages, and if this code is executed as __main__, can be used to check and output checksums by placing the 'ubx byte-code in question' towards the top of the code after "message = ".

Eventually, this file will handle the ubx-pvt messages, much as micropyGPS or as_GPS do for NMEA, and those files will be unneeded.

# SD Card

A microSD adapter may have pins soldered to it for breadboard use since the SD card and the Pico both talk at 3v3. DO NOT TRY THIS WITH a 5v ARDUINO!

![image](https://github.com/Teufelauto/pico-telemetry/blob/main/images/SD_Card_Adapter_Pins.jpg)

Parden the awful soldering. I know it's bad. I was in a hurry, and had no flux to give it.

