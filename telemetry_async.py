# Data Logger Designed for BerryGPS-IMUv4 and microSD Card on Raspberry Pi Pico
# by Jamie Halford

# Copyright (c) 2022 Jamie Halford
# Released under the MIT License (MIT) - see LICENSE file

# If 9600 fast baud not set, must cycle power to BerryGPS before each execution,
#     though threading pretty much requires a power-cycle, regardless.
# Messages sent to GPS must be in u-blox UBX protocol. ('mu b' 0xB5 0x62 starts a message)
# For breadboarding, SD micro to full size adapter has pins soldered to contacts.
#     http://justanotherlanguage.org/content/jallib/tutorials/tutorial_sd_card
# I am currently using 32 GB card from MicroCenter. Data can be opened with
#     Thonny if program was previously stopped with ctrl+c
# SPI baud can be set very high in sdcard library, but has minor effect on
#     short write intervals.

#---------------------+             Connects to:
#                   []  \   <-- No connection      
#                     [] |  CS       SPIx CSn
#                     [] |  MOSI     SPIx TX      
#       Micro to      [] |  GND      Ground     
#         SD          [] |  VCC      3v3(OUT)  
#       Adapter       [] |  SCK      SPIx SCK   
#                     [] |  GND      Ground    
#                     [] |  MISO     SPIx RX   
#                     [] |  <-- No connection 
#------------------------+


# SD Card SPI setup ---------------------------------------------------------
SD_SPI_NUMBER      = const(0)        # 0, 1
SD_MISO_RX_PIN     = const(4)        # Pins for SPI0 [0, 4, 16] for SPI1 [8, 12]
SD_CS_PIN          = const(5)        # Pins for SPI0 [1, 5, 17] for SPI1 [9, 13]
SD_SCK_PIN         = const(2)        # Pins for SPI0 [2, 6, 18] for SPI1 [10, 14]
SD_MOSI_TX_PIN     = const(3)        # Pins for SPI0 [3, 7, 19] for SPI1 [11, 15]

# IMU I2C setup --------------------------------------------------------------
IMU_I2C_NUMBER     = const(1)        # 0,1
IMU_SDA_PIN        = const(6)        # Pins for I2C0 [0, 4, 8, 12, 16, 20] for I2C1 [2, 6, 10, 14, 18, 26]
IMU_SCL_PIN        = const(7)        # Pins for I2C0 [1, 5, 9, 13, 17, 21] for I2C1 [3, 7, 11, 15, 19, 27]

# GPS UART setup -------------------------------------------------------------
GPS_UART_NUMBER    = const(0)        # 0, 1
GPS_UART_TX_PIN    = const(0)        # Pins for UART0 [0, 12, 16] for UART1 [4, 8]
GPS_UART_RX_PIN    = const(1)        # Pins for UART0 [1, 13, 17] for UART1 [5, 9]
GPS_FAST_BAUD      = const(9600)     # 9600, 19200, 38400, 57600, 115200 (9600 factory default)

# GPS Sync Clock ---------------------------------------------------------------------
GPS_PPS_PIN        = const(20)       # Pin 20 or 22 (ideally)

# GPS Message output setup ----------------------------------------------------------------
GPS_UPDATE_FREQ      = const(10)     # (Hz) 1, 2, 5, 10 (1 factory default)
DISABLE_NMEA_CLUTTER = const(False)   # Removes all NMEA messages except GNRMC from output
DISABLE_GxRMC        = const(False)  # Removes GNRMC from output
DISABLE_UBX_CLUTTER  = const(True)   # Removes several ubx messages from output
ENABLE_UBX_PVT       = const(False)  # Adds ubx message with relevent data to output
GPS_COLD_REBOOT      = const(False)  # set to True for cold reboot (start without cached gps data- for trouble shooting)
LOCAL_OFFSET         = const(-4)     # Time Zone hours offset from UTC (integer)
SPEED_UNITS          = const(1)      # 0=knots 1=mph 2=kph

# File Name Formatting ------------------------------------------------------------
filename_path      = '/sd/'          # "/sd/" unless custom folders on SD Card
filename_prefix    = 'myMPHdata'        # Can be anything you like to ID your files.
# YYMMDDHHMMSS will be appended to file make each unique
filename_extension = '.csv'          # Recommend ".csv" 

# File's column data header (Date only written once per file for code speed and file write speed.
header = 'Date,Time,Latitude,Longitude,Speed,Course,xG,yG,zG'


# Standard libraries ##########################################################
import time
import uos
from machine import UART, Pin, SPI
from math import floor
import _thread

# Install in "lib" folder on pico ###############################################
# temp micropygps until it can be purged from file creation and data collection.
from micropyGPS import MicropyGPS # download from https://github.com/inmcm/micropyGPS and placed in the /lib folder on main storage.
import sdcard  # download from https://github.com/micropython/micropython/tree/master/drivers/sdcard and placed in the /lib folder on main storage.
import IMU_I2C as IMU # MUST be modified to match i2c pinout #download from https://github.com/ozzmaker/BerryIMU/tree/master/PicoMicroPython
#      LSM6DSL  # install alongside IMU_I2C. https://github.com/ozzmaker/BerryIMU/tree/master/PicoMicroPython
import UBX_PICO as ubx # by Jamie Halford

import uasyncio as asyncio
import as_GPS
#############################################################################



gps_data = ''
date_string = ''
data_string = ''
write_string = ''
zero_sec = 0
new_data_flag = False
baton = _thread.allocate_lock()
led = Pin(25, Pin.OUT, value=0)


# SD Card Mounting --------------------------------------------------------------
SD_spi = SPI(SD_SPI_NUMBER, baudrate=1_000_000, polarity=0, phase=0, bits=8,
             firstbit=machine.SPI.MSB, sck=Pin(SD_SCK_PIN),
             mosi=Pin(SD_MOSI_TX_PIN), miso=Pin(SD_MISO_RX_PIN))
sd = sdcard.SDCard(SD_spi,Pin(SD_CS_PIN, Pin.OUT))
vfs = uos.VfsFat(sd)
uos.mount(vfs, "/sd")

# IMU initialisation ------------------------------------------------------
IMU.initIMU()       #Initialize the accelerometer, gyroscope and compass

# Real Time Clock -------------------------------------------------------------
rtc = machine.RTC()
pps = Pin(GPS_PPS_PIN, Pin.IN, Pin.PULL_DOWN) # rising edge is time pulse from gps.

# GPS Communication Initialization ---------------------------------------------
# my_gps = MicropyGPS(LOCAL_OFFSET,"dd")


# GPS setup
ubx.setup(GPS_UART_NUMBER, GPS_UART_TX_PIN, GPS_UART_RX_PIN,
          GPS_UPDATE_FREQ, GPS_FAST_BAUD, GPS_COLD_REBOOT,DISABLE_NMEA_CLUTTER,
          DISABLE_GxRMC, DISABLE_UBX_CLUTTER, ENABLE_UBX_PVT)

# Set pico UART baud rate to match the GPS output 
uartGPS = UART(GPS_UART_NUMBER,baudrate= GPS_FAST_BAUD, tx=Pin(GPS_UART_TX_PIN),
               rx=Pin(GPS_UART_RX_PIN), bits=8, parity=None, stop=1)
print(f'pico fast baudrate set to {GPS_FAST_BAUD}' )
time.sleep_ms(1000)

                                                                           
  

# Functions -------------------------------------------------------------------

def make_current_time(gps_time,zero_sec):
    diff_sec = time.ticks_diff(time.ticks_ms(), zero_sec)# get timer diff
    if diff_sec < 10:
        diff_sec = f'0{diff_sec}'
    elif diff_sec > 99:
        print(f'{gps_time}{diff_sec}  time error')
    current_time = f'{gps_time}{diff_sec}'
    return current_time # adds hundredths and thousandths or, centiseconds ;-P
                        # and milliseconds to the time


def date_form(datestamp, timestamp, LOCAL_OFFSET):
    # Desired formatting yymmdd not in micropyGPS
    # micropyGPS does not account for local offset in the date.
    # This if-statement chain fixes it.
    zulu = (timestamp[0] - LOCAL_OFFSET)
    datestamp_list = list(datestamp) # can't modify a tuple, so make date a list

    # west of zulu
    if LOCAL_OFFSET < 0: 
        if zulu >= 24 : # we are still a day behind utc
            
            if datestamp_list[0] == 1: # detect month rollover if day is 1
                datestamp_list[1] = datestamp_list[1] - 1 # correct month
                
                if datestamp_list[1] == 0: # correct month and year if New Year's (~Cheers~)
                    datestamp_list[1] = 12 # make december
                    datestamp_list[2] = datestamp_list[2] - 1 # make year 1 less
                    
            # month and year fixed, now fix the day
            datestamp_list[0] = datestamp_list[0] - 1 # correct the Day
            
            if datestamp_list[0] == 0: # if day is 0 from month rollover
                # Set to last day in current local month
                if datestamp_list[1] in (1, 3, 5, 7, 8, 10, 12): # Used knuckle counting tecnique  :-D
                    datestamp_list[0] = 31
                    
                elif datestamp_list[1] in (4, 6, 9, 11): # Months between knuckles - Feb
                    datestamp_list[0] = 30
                    
                else: # deal with last day of Feb
                    if datestamp_list[2] in (24,28,32,36,40,44,48,52,56,60,64,68): # leap years
                        datestamp_list[0] = 29
                        
                    else:
                        datestamp_list[0] = 28
    # east of zulu
    if LOCAL_OFFSET > 0: 
        if zulu < 0: # We are in the next day
            
            datestamp_list[0] = datestamp_list[0] + 1 # add a new Day
            
            # check if we are in a new month
            if datestamp_list[0] == 32: # knuckle months
                datestamp_list[0] = 1 # day fixed
                datestamp_list[1] = datestamp_list[1] + 1 # add a month
                
                # correct month and year if New Year's (~Cheers~)
                if datestamp_list[1] == 13: 
                    datestamp_list[1] = 1 # make January
                    datestamp_list[2] = datestamp_list[2] + 1 # make year 1 more
                    
            elif datestamp_list[0] == 31: # Months between knuckles - Feb
                if datestamp_list[1] in (4, 6, 9, 11):
                    datestamp_list[0] = 1 # day fixed
                    datestamp_list[1] = datestamp_list[1] + 1 # add a month
                    
            else: # deal with last day of Feb
                if datestamp_list[1] == 2: # let pass random days from other months
                    if datestamp_list[2] in (24,28,32,36,40,44,48,52,56,60,64,68): # leap years
                        if datestamp_list[0] == 30:
                            datestamp_list[0] = 1 # day fixed
                            datestamp_list[1] = datestamp_list[1] + 1 # add a month
                    else:
                        if datestamp_list[0] == 29:
                            datestamp_list[0] = 1 # day fixed
                            datestamp_list[1] = datestamp_list[1] + 1 # add a month
                                 
    datestamp = tuple(datestamp_list) # recreate the tuple for use in file naming function

    # Create the Date String as yymmdd
    # Add leading zeros to day string if necessary
    if datestamp[0] < 10:
        day = f'0{datestamp[0]}'
    else:
        day = f'{datestamp[0]}'  
    # Add leading zeros to month string if necessary
    if datestamp[1] < 10:
        month = f'0{datestamp[1]}'
    else:
        month = f'{datestamp[1]}'
    # Add leading zeros to year string if necessary
    if datestamp[2] < 10:
        year = f'0{datestamp[2]}'
    else:
        year = f'{datestamp[2]}'
    # Build final string based on desired formatting yymmdd
    date_string = f'{year}{month}{day}'
    
    return date_string, datestamp


def time_form(timestamp):
    # Desired formatting not in micropyGPS hhmmss.s
    if timestamp[0] < 10: #Hours      
        time_string = f'0{timestamp[0]}'
    else:
        time_string = f'{timestamp[0]}'
    if timestamp[1] < 10: #Minutes
        time_string = f'{time_string}0{timestamp[1]}'
    else:
        time_string = f'{time_string}{timestamp[1]}'
    if timestamp[2] < 10: #Seconds
        time_string = f'{time_string}0{timestamp[2]}'
    else:
        time_string = f'{time_string}{timestamp[2]}'
    return time_string


def create_filename():
    #filename_path, filename_prefix, filename_extension
    timestamp = [0, 0, 0.0]
    datestamp = [0, 0, 0]
    print('Querying gps for valid date and timestamp.')
    stat_count = 0
    while True:
        if uartGPS.any(): # filename generated with date and time
            led.value(1)
            oneByte = (uartGPS.read(1))    # works to collect one character in byte form
            stat = my_gps.update(chr(oneByte[0]))  # converts byte to text character for use in function
            
            if stat:
                stat_count = stat_count + 1
                print(f'Attempt {stat_count}')
                led.value(0)
                datestamp = my_gps.date
                timestamp = my_gps.timestamp
                stat = None           

               # runs until gps gets a valid fix on 0 tenths for rtc, then allows file creation.
            if timestamp[2] == int(timestamp[2]) and timestamp != [0,0,0.0] and datestamp != [0,0,0]:
                break;   
    timestamp[2] = int(timestamp[2]) # Remove decimal from seconds
    date_form_result = date_form(datestamp, timestamp, LOCAL_OFFSET)#returns string and resets datestamp for local time
    date_string = date_form_result[0]
    datestamp = date_form_result[1] # datestamp now corrected for timezone
    time_string = time_form(timestamp)
    
    # set real time clock
    rtc.datetime((datestamp[2],datestamp[1],datestamp[0], 1, timestamp[0],
                  timestamp[1], timestamp[2], 0))
    
    filename = f'{filename_path}{filename_prefix}{date_string}{time_string}{filename_extension}'
    file_header = f'{header}\r\n{filename}\r\n{date_string}\r\n'
    print(f'Writing to {filename}')
    
    with open(filename, "w") as file:  # "w" for new file
        file.write(file_header)
    return filename


def get_gps_data():
    # cycle until GPS update recieved
    global zero_sec # global timer
    while True:
        if uartGPS.any():
            oneByte = (uartGPS.read(1))    # works to collect one character in byte form
            stat = my_gps.update(chr(oneByte[0]))  # converts byte to text character for use in function, returns NMEA identifier after message parsed
            if stat: # since all but rmc are eliminated, we begin
                zero_sec = time.ticks_ms() # start global timer immediately. this provides approx milliseconds instead of tenths timing
                time_string = time_form(my_gps.timestamp) 
                gps_data = f'{my_gps.latitude[0]:3.6f},-{my_gps.longitude[0]:3.6f},{my_gps.speed[SPEED_UNITS]:3.1f},{my_gps.course},'
                stat = None
                return gps_data, time_string, zero_sec


def get_imu_data():
    xG = (IMU.readACCx() * 0.244)/1000 # .244 may become variable, and zero compensation may need implemented
    yG = (IMU.readACCy() * 0.244)/1000
    zG = (IMU.readACCz() * 0.244)/1000
    imu_data = f'{xG:1.2f},{yG:1.2f},{zG:1.2f},CFangleX,CFangleY,'
    return imu_data


    # placeholder of time consuming data for testing. 3 analog signals, unless multiplex MCP3008
def get_acq_data():
    throttle_position = (my_gps.time_since_fix() * 244)/100
    brake_pressure = (my_gps.time_since_fix() * 0.2)/100
    steering_angle = my_gps.time_since_fix()
    rpm = 5280 + my_gps.time_since_fix()
    blinker_fluid = led.value()
    nitrous_active = IMU.readACCz()
    water_injection = led.value()
    acq_data = f'{throttle_position:4.0f},{brake_pressure:4.0f},{steering_angle:4.0f},{rpm:4.0f},{blinker_fluid},{nitrous_active},{water_injection}' # put no comma at end
    return acq_data


def add_gps_data(data_string, current_time, gps_data=''):
    if gps_data == '': # data between gps fix
        data_string = f'{data_string},{current_time},,,,,'
        return data_string
    else: # just got a gps fix
        data_string = f'{data_string},{current_time},{gps_data}'
        return data_string


def add_imu_data(data_string, imu_data):
    data_string = f'{data_string}{imu_data}'
    return data_string


def add_acq_data(data_string, acq_data):
    data_string = f'{data_string}{acq_data}\r\n' # End of data line gets carraige return
    return data_string


# Core 1 Loop
# Run on second thread, 'cause it takes so long to write to sd card
def write_data():
    global write_string # a class would be more elegant
    global new_data_flag
    
    while True: # keep core 1 active (experimental)
        
        if new_data_flag == True:
        
            baton.acquire() # we dont want main loop to overwrite write_string or call function while still writing
            
            start_timer = time.ticks_ms() #start timer
            led.on()
            
            
            with open(filename, "a") as file:  # "a" for appending
                file.write(write_string)
            
            new_data_flag = False
            led.off()
            diff_timer= time.ticks_diff(time.ticks_ms(), start_timer)# get timer diff
            print(f'{diff_timer} ms write time')# print timer
            
            baton.release()
            
        time.sleep_ms(10)





sreader = asyncio.StreamReader(uartGPS)  # Create a StreamReader
gps = as_GPS.AS_GPS(sreader)  # Instantiate GPS

async def test():
    print('waiting for GPS data')
    await gps.data_received(position=True)
    for _ in range(10):
        print(gps.latitude(), gps.longitude(), gps.altitude)
        await asyncio.sleep(.1)

asyncio.run(test())










# 
# # File creation ------------------------------------------
# 
# filename = create_filename() # uses core 0 for initial startup without threading
# time.sleep(1)
# 
# # Start second thread to save data to sd card! -------------
# _thread.start_new_thread(write_data, ()) 
# 
# 
#  ############################
#  # Main Loop
#  ############################
# 
# # Core 0 Loop
# while True:
#     while True: # delete this line after pps triggering is implemented
# #     if pps.value() == 1: # when timepulse hits, get message.
#         for ii in range(2):
#             # 2 gps fixes, 12 lines of data collected before
#             # incremental save. More lines take long time to append,
#             # less lines toke too long to save.
#             
#             gps_data_and_time = get_gps_data() # regulates timing somewhat.
#             # Need to deal with no message received
#             
#             gps_data = gps_data_and_time[0]
#             gps_time = gps_data_and_time[1]
#             zero_sec = gps_data_and_time[2]
#             
#     #         led.toggle()
#     #         current_time = make_current_time(gps_time,zero_sec)
#     #         print(f'{current_time} after gps fix')
#     
#             #   Need better timing between gps time pulses
#             for i in range(6): # 6 loops per single gps fix
#                 current_time = make_current_time(gps_time,zero_sec)
#                 imu_data = get_imu_data()
#                 acq_data = get_acq_data()
#                 if i == 0:
#                     data_string = add_gps_data(data_string, current_time, gps_data)
#                 else : # second through sixth loops dont pass gps data
#                     data_string = add_gps_data(data_string, current_time)
#                 data_string = add_imu_data(data_string, imu_data)
#                 data_string = add_acq_data(data_string, acq_data)
#                 
#                 current_time = make_current_time(gps_time,zero_sec)
#                 print(f'{ii} {i} {current_time}')
#                 
#         baton.acquire() # pause here until core 1 releases lock if necessary
#         new_data_flag = True
#         write_string = data_string
#         baton.release() # the data for the write is ready, so let core 1 write
#         data_string = '' # reset the working variable

    
    
    
    
    
