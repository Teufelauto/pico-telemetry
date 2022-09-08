# UBX protocol checksum checker and creator, and library for RP2 Pico
# by Jamie Halford

# Copyright (c) 2022 Jamie Halford
# Released under the MIT License (MIT) - see LICENSE file

# Tested with ublox MAX-8C reciever with Protocol ver 18.00

# Use this file as library for T/F, or below enter message value to find checksums
# Be aware that Python prints hex as ascii if it's in the range. \xb5\x62 prints \xb5b
#  sync chars 0xB5 0x62  "mu b" begin every UBX message.

import time
from machine import UART, Pin


# Enter a ubx message to be manually checked here when using this code stand-alone:

message = b'\xB5\x62\x06\x00\x14\x00\x01\x00\x00\x00\xD0\x08\x00\x00\x80\x25\x00\x00\x07\x00\x03\x00\x00\x00\x00\x00\xA2\xB5'


# bits and pieces for testing

# UBX-ACK-ACK Message Acknowledged
# b'\xb5\x62\x05\x01\x02\x00

# UBX-ACK-NAK Message Not Acknowledged
# b'\xb5\x62\x05\x00\x02\x00



######################
# Settings Functions #
######################

def time_pulse_setup(uartGPS, gps_pulses_per_sec):
    # Set the frequency of the PPS pulse
    if gps_pulses_per_sec == 10:
        uartGPS.write(b'\xB5\x62\x06\x31\x20\x00\x00\x01\x00\x00\x32\x00\x00\x00\x0A\x00\x00\x00\x0A\x00\x00\x00\x00\x00\x00\x00\x9A\x99\x99\x19\x00\x00\x00\x00\x6F\x00\x00\x00\xF2\x7C')
    elif gps_pulses_per_sec == 5:
        uartGPS.write(b'\xB5\x62\x06\x31\x20\x00\x00\x01\x00\x00\x32\x00\x00\x00\x05\x00\x00\x00\x05\x00\x00\x00\x00\x00\x00\x00\x9A\x99\x99\x19\x00\x00\x00\x00\x6F\x00\x00\x00\xE8\xA0')
    elif gps_pulses_per_sec == 2:
        uartGPS.write(b'\xB5\x62\x06\x31\x20\x00\x00\x01\x00\x00\x32\x00\x00\x00\x02\x00\x00\x00\x02\x00\x00\x00\x00\x00\x00\x00\x9A\x99\x99\x19\x00\x00\x00\x00\x6F\x00\x00\x00\xE2\x1C')
    elif gps_pulses_per_sec == 1:
        uartGPS.write(b'\xB5\x62\x06\x31\x20\x00\x00\x01\x00\x00\x32\x00\x00\x00\x40\x42\x0F\x00\x40\x42\x0F\x00\x00\x00\x00\x00\xA0\x86\x01\x00\x00\x00\x00\x00\x77\x00\x00\x00\x4A\xB6')
    else:
        raise Exception('Pulse Rate Unsupported')
    print(f'Time Pulse set to {gps_pulses_per_sec}Hz')
    time.sleep_ms(500)

def cold_reboot(uartGPS):
    # cold reboot command
    uartGPS.write(b'\xb5\x62\x06\x04\x04\x00\xff\xff\x02\x00\x0e\x61')
    print('COLD REBOOT! Prepare to wait for satalites')
    time.sleep_ms(2000)

def fast_baud(uartGPS, gps_fast_baud):
    # Set desired baud rate on GPS

    if gps_fast_baud == 19200:
        uartGPS.write(b'\xB5\x62\x06\x00\x14\x00\x01\x00\x00\x00\xD0\x08\x00\x00\x00\x4B\x00\x00\x07\x00\x03\x00\x00\x00\x00\x00\x48\x57')
    elif gps_fast_baud == 38400:
        uartGPS.write(b'\xB5\x62\x06\x00\x14\x00\x01\x00\x00\x00\xD0\x08\x00\x00\x00\x96\x00\x00\x07\x00\x03\x00\x00\x00\x00\x00\x93\x90')
    elif gps_fast_baud == 57600:
        uartGPS.write(b'\xB5\x62\x06\x00\x14\x00\x01\x00\x00\x00\xD0\x08\x00\x00\x00\xE1\x00\x00\x07\x00\x03\x00\x00\x00\x00\x00\xDE\xC9')
    elif gps_fast_baud == 115200:
        uartGPS.write(b'\xB5\x62\x06\x00\x14\x00\x01\x00\x00\x00\xD0\x08\x00\x00\x00\xC2\x01\x00\x07\x00\x03\x00\x00\x00\x00\x00\xC0\x7E')
    elif gps_fast_baud == 9600:
        # GPS already boots at 9600 baud, but here's the command
        uartGPS.write(b'\xB5\x62\x06\x00\x14\x00\x01\x00\x00\x00\xD0\x08\x00\x00\x80\x25\x00\x00\x07\x00\x03\x00\x00\x00\x00\x00\xA2\xB5') #9600 baud
    else:
        raise Exception('Baud Rate Unsupported')
    print(f'gps fast baudrate set to {gps_fast_baud}')
    time.sleep_ms(1000)

def gps_freq(uartGPS, gps_update_freq):
    # Set GPS update frequency
    if gps_update_freq == 20:
        uartGPS.write(b'\xB5\x62\x06\x08\x06\x00\x32\x00\x01\x00\x01\x00\x48\xE6')  # sets 20 Hz freq
    elif gps_update_freq == 10:
        uartGPS.write(b'\xB5\x62\x06\x08\x06\x00\x64\x00\x01\x00\x01\x00\x7A\x12')  # sets 10 Hz freq
    elif gps_update_freq == 5:
        uartGPS.write(b'\xB5\x62\x06\x08\x06\x00\xC8\x00\x01\x00\x01\x00\xDE\x6A')  # sets 5 Hz freq
    elif gps_update_freq == 2:
        uartGPS.write(b'\xB5\x62\x06\x08\x06\x00\xF4\x01\x01\x00\x01\x00\x0B\x77')  # sets 2 Hz freq
    elif gps_update_freq == 1:
        pass # GPS already boots at 1 Hz
    else:
        raise Exception('Update Frequency Unsupported')
    print(f'GPS update frequency set to {gps_update_freq} Hz')
    time.sleep_ms(1000)

def disable_nmea(uartGPS): #Does not disable RMC
    # Disable unneeded bandwidth hogs.
    uartGPS.write(b'\xB5\x62\x06\x01\x08\x00\xF0\x00\x00\x00\x00\x00\x00\x01\x00\x24') # GGA
    uartGPS.write(b'\xB5\x62\x06\x01\x08\x00\xF0\x01\x00\x00\x00\x00\x00\x01\x01\x2B') # GLL
    uartGPS.write(b'\xB5\x62\x06\x01\x08\x00\xF0\x02\x00\x00\x00\x00\x00\x01\x02\x32') # GSA
    uartGPS.write(b'\xB5\x62\x06\x01\x08\x00\xF0\x03\x00\x00\x00\x00\x00\x01\x03\x39') # GSV
    uartGPS.write(b'\xB5\x62\x06\x01\x08\x00\xF0\x05\x00\x00\x00\x00\x00\x01\x05\x47') # VTG
    print('NMEA decluttered')
    time.sleep_ms(1000)

def disable_rmc(uartGPS):
    # Most useful NMEA ascii message. If only this is being output, my_gps may be quick enough.
    uartGPS.write(b'\xB5\x62\x06\x01\x08\x00\xF0\x04\x00\x00\x00\x00\x00\x01\x04\x40') # RMC
    print('RMC message disabled')
    time.sleep_ms(1000)

def disable_ubx(uartGPS):
    # Disable unwanted ubx messages
    uartGPS.write(b'\xB5\x62\x06\x01\x08\x00\x01\x02\x00\x00\x00\x00\x00\x00\x12\xB9') # NAV-POSLLH
    uartGPS.write(b'\xB5\x62\x06\x01\x08\x00\x01\x03\x00\x00\x00\x00\x00\x00\x13\xC0') # NAV-STATUS
    print('ubx messages decluttered')
    time.sleep_ms(1000)

def enable_pvt(uartGPS):
    # The UBX-NAV-PVT message has the info we want. While RMC is ascii, this may be better for speed?
    uartGPS.write(b'\xB5\x62\x06\x01\x08\x00\x01\x07\x00\x01\x00\x00\x00\x00\x18\xE1') # NAV-PVT
    print('UBX-NAV-PVT message enabled')
    time.sleep_ms(1000)


#####################
# Checksum Function #
#####################

def ubx_checksum(message): # returns True if good complete message, and 2 integer checksums
    # 8-Bit Fletcher Algorithm for UBX Checksum
    # Takes about .5ms
    N = (message[5] * 256) + message[4] + 6  # number of bytes in message before checksum.
                            # Positions 4 & 5 specify payload length. Check-range excludes
                            # first 2 bytes and the last 2 bytes (checksum) if present.
    CK_A = 0
    CK_B = 0

    for i in range(2,N): # skip the first 2 bytes in message which don't get checked
        CK_A += message[i]
        CK_A &= 255  # & 255 prevents INTs bigger than 255
        CK_B += CK_A
        CK_B &= 255
        
    if len(message) == N+2:   # if message contains checksum already, check it
        if CK_A == message[N] and CK_B == message[N+1]:
            return True, CK_A, CK_B # return True if good complete message

    return False, CK_A, CK_B


######################
# GPS Start-Up Setup #
######################

def setup(gps_uart_number, gps_uart_tx_pin, gps_uart_rx_pin,
          gps_update_freq=1, gps_fast_baud=9600, gps_pulses_per_sec=1, gps_cold_reboot=False,
          disable_nmea_clutter=False, disable_RMC=False,
          disable_UBX_clutter=False, enable_UBX_PVT=False):

    # gps_uart_tx_pin is the TX GPIO pin on the pico. same goes for RX
    # pico Must xmit at 9600 baud on power up of u-blox CAM-M8
    uartGPS = UART(gps_uart_number,baudrate=9600, tx=Pin(gps_uart_tx_pin),
                   rx=Pin(gps_uart_rx_pin), bits=8, parity=None, stop=1)
    print('initial baudrate set to 9600')
    time.sleep_ms(100)

    if gps_cold_reboot == True:
        cold_reboot(uartGPS) # cold reboot command
    if disable_nmea_clutter == True:
        disable_nmea(uartGPS) # Disable extranious NMEA messages
    if disable_RMC == True:
        disable_rmc(uartGPS)  # Disable if NO NMEA messages are desired
    if disable_UBX_clutter == True:
        disable_ubx(uartGPS)  # Disable unwanted ubx messages
    if enable_UBX_PVT == True:
        enable_pvt(uartGPS)   # Enables the ubx version of RMC

    gps_freq(uartGPS, gps_update_freq) # Set GPS update frequency
    
    time_pulse_setup(uartGPS, gps_pulses_per_sec) # Setup speed of Time Pulses
    # Set fast baud ONLY at end of setup
    fast_baud(uartGPS, gps_fast_baud) # Set desired baud rate on GPS 




#####################################################################################
# Usage for creating or verifying message checksums. Change message variable to use #
#####################################################################################

if __name__ == "__main__":
    
    print(message)
    checksum_results = ubx_checksum(message)

    CK_A = checksum_results[1]
    CK_B = checksum_results[2]
    CK_A_hex = hex(CK_A) # convert INT to hex. Note that values falling in ascii range will print as ascii text
    CK_B_hex = hex(CK_B)

    print(checksum_results[0]) # prints True if message is valid, containing correct checksums.
    # prints False if incorrect checksums or no checksums

    print(f'Checksum is {CK_A}  {CK_B}') # These are the integer checksum that should match positions message[N] and [N+1]
    print(f'Checksum is {CK_A_hex} {CK_B_hex}') # These are the hex checksum that should match positions message[N] and [N+1]
