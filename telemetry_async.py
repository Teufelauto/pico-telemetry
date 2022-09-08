# Telemetry Logger Designed for BerryGPS-IMUv4 and microSD Card on Raspberry Pi Pico
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
# SPI baud can be set very high in sdcard library, but has minor affect on
#     small bursts at short write intervals.

#---------------------+             Connects to:
#                   []  \   <-- No connection      
#                     [] |  CS       SPIx SCK
#                     [] |  MOSI     SPIx TX      
#       Micro to      [] |  GND      Ground     
#         SD          [] |  VCC      3v3(OUT)  
#       Adapter       [] |  SCK      SPIx SCK   
#                     [] |  GND      Ground    
#                     [] |  MISO     SPIx RX   
#                     [] |  <-- No connection 
#------------------------+


# Lines 4-6 in IMU_I2C.py MUST be changed to match your chosen wiring
# Line 7 in LSM6DSL.py MUST have value changed to 0x17 due to typo
# Line 61 in as_GPS.py ~may~ be changed to False to improve gps message parsing speed
# Line 493 in as_GPS.py must be changed from 1 to 0 or an arbitrarily small amount to function at high hz. await asyncio.sleep(0)

# Standard Pico Libraries ##########################################################
import time
import uos
from machine import UART, Pin, SPI
from math import floor
import _thread
import uasyncio as asyncio

# Libraries to install in the "lib" folder you create on Pico ###############################################

import sdcard  # https://github.com/micropython/micropython/tree/master/drivers/sdcard
import IMU_I2C as IMU # MUST be modified to match i2c pinout # https://github.com/ozzmaker/BerryIMU/tree/master/PicoMicroPython
#      LSM6DSL.py - install alongside IMU_I2C. https://github.com/ozzmaker/BerryIMU/tree/master/PicoMicroPython
from as_GPS import * # https://github.com/peterhinch/micropython-async/tree/master/v3/as_drivers/as_GPS
# import aadc  # future
import UBX_PICO as ubx # by Jamie Halford for talking to the u-blox CAM M8 GPS receiver on the Berry


#################################################
# Constants to be modified for your application #
#################################################

# File Name Formatting -----------------------------------------------------
filename_path      = '/sd/'          # "/sd/" unless custom folders on SD Card
filename_prefix    = 'RACEdata'      # Can be anything you like to ID your files.
filename_extension = '.csv'          # Recommend ".csv" as data is comma deliniated
# YYMMDDHHMMSS will be appended, making each file unique, and in chronological order
# A typical filename would be: RACEdata220903065128.csv
# This will help correlate telemetry files with your onboard videos 
# File's column header (Date only written once per file for code speed and
#  file write speed. The first column will be mostly blank except first few rows.

header = 'Date,Time,Latitude,Longitude,Speed,Course,xG,yG,zG,xGyro,yGyro,zGyro,mag_heading,throttle_position,brake_pressure,steering_angle,rpm,LF_damper_position,RF_damper_position,LR_damper_position,RR_damper_position'


# Set the constants below to match your wiring and configuration
# SD Card SPI setup ---------------------------------------------------------
SD_SPI_NUMBER      = const(0)        # 0, 1
SD_MISO_RX_PIN     = const(4)        # Pins for SPI0 [0, 4, 16] for SPI1 [8, 12]
SD_CS_PIN          = const(5)        # Pins for SPI0 [1, 5, 17] for SPI1 [9, 13]
SD_SCK_PIN         = const(2)        # Pins for SPI0 [2, 6, 18] for SPI1 [10, 14]
SD_MOSI_TX_PIN     = const(3)        # Pins for SPI0 [3, 7, 19] for SPI1 [11, 15]

# IMU I2C setup --------------------------------------------------------------
IMU_I2C_NUMBER     = const(1)        # 0, 1
IMU_SDA_PIN        = const(6)        # Pins for I2C0 [0, 4, 8, 12, 16, 20] for I2C1 [2, 6, 10, 14, 18, 26]
IMU_SCL_PIN        = const(7)        # Pins for I2C0 [1, 5, 9, 13, 17, 21] for I2C1 [3, 7, 11, 15, 19, 27]

# GPS UART setup -------------------------------------------------------------
GPS_UART_NUMBER    = const(0)        # 0, 1
GPS_UART_TX_PIN    = const(0)        # Pins for UART0 [0, 12, 16] for UART1 [4, 8]
GPS_UART_RX_PIN    = const(1)        # Pins for UART0 [1, 13, 17] for UART1 [5, 9]
GPS_FAST_BAUD      = const(9600)     # 9600, 19200, 38400, 57600, 115200 (9600 factory default)

# GPS Sync Clock -------------------------------------------------------------
GPS_PPS_PIN        = const(20)       # Pin 20 or 22 (ideally for setting RTC?) but any will work
PULSES_PER_SEC     = const(10)       # (Hz) 1, 2, 5, 10 Set to match GPS_UPDATE_FREQ (seperate for testing purposes)

# GPS Message output setup ----------------------------------------------------
GPS_UPDATE_FREQ      = const(10)     # (Hz) 1, 2, 5, 10, 20? (1 factory default)
DISABLE_NMEA_CLUTTER = const(True)   # Removes all NMEA messages except GNRMC from output
DISABLE_RMC          = const(False)  # Removes GNRMC from output
DISABLE_UBX_CLUTTER  = const(True)   # Removes several ubx messages from output
ENABLE_UBX_PVT       = const(False)  # Adds ubx message with relevent data to output
GPS_COLD_REBOOT      = const(False)  # set to True for cold reboot (start without cached gps data- for trouble shooting)
LOCAL_OFFSET         = const(-4)     # Time Zone hours offset from UTC (integer only. sorry)
SPEED_UNITS          = MPH           # KNOT, MPH, KPH (A constant in the as_GPS lib) 

# Analog to Digital Converter SPI setup -----------------------------------------------------------
ADC_SPI_NUMBER      = const(1)        # 0, 1
ADC_MISO_RX_PIN     = const(8)        # Pins for SPI0 [0, 4, 16] for SPI1 [8, 12]
ADC_CS_PIN          = const(9)        # Pins for SPI0 [1, 5, 17] for SPI1 [9, 13]
ADC_SCK_PIN         = const(10)        # Pins for SPI0 [2, 6, 18] for SPI1 [10, 14]
ADC_MOSI_TX_PIN     = const(11)        # Pins for SPI0 [3, 7, 19] for SPI1 [11, 15]

###########
# Classes #
###########

class TimingPulse: # to eventually be put in UBX_PICO 
    # usage ex: PPS.new_pulse = True
    def __init__(self, pin):
        self._pin = pin
        self.sense = pin.value()
        self.new_pulse = False
        
#     async def run(self):
#         # check for timing pulse
#         while True:
#             self.sense = pin.value()
#             await asyncio.sleep(0)


class WriteFlags:
    # instead of using Globals for talking between cores
    def __init__(self, write_string='', new_data_flag=False):
        self.write_string = write_string
        self.new_data_flag = new_data_flag


class DataLines:
    # Experiment for faster line creation
    # Use of a buffer might be helpful, but data is not of fixed size...
    # Using async will help
    def __init__(self):
        pass
    

class TimeSuck:
    # A class for collecting time deltas
    def __init__(self):
        self.lower_bound = 0  # to keep plotter scaled
        self.upper_bound = 10 # to keep plotter scaled
        self.get_current_time_collect = 0 #retrieve time  ~
        self.timcon = 0 # concatenate time string   ~.6ms, sometimes 1.1ms
        self.get_current_time_timer = 0  #          ~1.5ms
        self.waiting_for_gps_pulse = 0         
        self.gpsget = 0 # time to get gps data      ~3ms  Huge problem.
        # loop time too slow to catch each 100 ms time pulse. need async data gather?
        
        
        self.gpscon = 0 # time to concatenate gps   ~1-2ms
        self.imuget = 0 #                           ~7-8ms
        self.imucon = 0 #                           ~2-3ms
        self.acqget = 0 #                           ~5ms but is placeholder function
        self.acqcon = 0 #                           ~3ms
        self.addline = 0 # concat gps, imu, acq     ~2.5ms but sometimes 11ms
        self.write_timer = 0 # time to append .csv file in milliseconds
        self.current_time_float = 0 #
        self.cycle_timer = 0 #                      ~23ms but up to 50 on gps cycle
    
    def selected_data(self, ts):
        # Choose any data to print to Shell for Plotter
        # Perhaps combining all concatenation into one .join is way to go.
        # OR address writes.
        
        print(f'L bound: {ts.lower_bound} Message delay: {ts.waiting_for_gps_pulse}')
#         print(f'L bound: {ts.lower_bound} Get gps: {ts.gpsget} Data Line: {ts.cycle_timer}') 
#         print(f'L bound: {ts.lower_bound} Time retrieval: {ts.get_current_time_collect}')
#         print(f'L bound: {ts.lower_bound} Con Time: {ts.timcon}')
#         print(f'L bound: {ts.lower_bound} Current time func: {ts.get_current_time_timer}')
#         print(f'L bound: {ts.lower_bound} Get gps: {ts.gpsget}') 
#         print(f'L bound: {ts.lower_bound} Con gps: {ts.gpscon}')
#         print(f'L bound: {ts.lower_bound} Get imu: {ts.imuget}')
#         print(f'L bound: {ts.lower_bound} Con imu: {ts.imucon}')
#         print(f'L bound: {ts.lower_bound} Get acq: {ts.acqget}')
#         print(f'L bound: {ts.lower_bound} Con acq: {ts.acqcon}')
#         print(f'L bound: {ts.lower_bound} Add Line: {ts.addline}')
#         print(f'L bound: {ts.lower_bound} Write time: {ts.write_timer}')
#         print(f'L bound: {ts.lower_bound} Current time: {ts.current_time_float}')
#         print(f'L bound: {ts.lower_bound} Time since fix: {my_gps.time_since_fix()}')
#         print(f'L bound: {ts.lower_bound} Data Line: {ts.cycle_timer}')
# # All concatenation events: 
#         print(f'L bound: {ts.lower_bound} Con Time: {ts.timcon} Con gps: {ts.gpscon} Con imu: {ts.imucon} Con acq: {ts.acqcon} Add Line: {ts.addline}') 
# # All Data gatering events:
#         print(f'L bound: {ts.lower_bound} Get gps: {ts.gpsget} Get imu: {ts.imuget} Get acq: {ts.acqget}
       
        
#############
# Functions #
#############


async def create_filename():
    #filename_path, filename_prefix, filename_extension
    
    print('Querying gps for valid date and timestamp.')
    
    await my_gps.data_received(date=True) # wait for a valid date
    
    timestamp = my_gps.local_time  # tuple (h, m, s) 
    datestamp = my_gps.date # tuple (d, m, yy)
    
    time_string = f'{timestamp[0]:02d}{timestamp[1]:02d}{timestamp[2]:02d}'    
    date_string = f'{datestamp[2]:02d}{datestamp[1]:02d}{datestamp[0]:02d}'
    
    # set real time clock -Probably worthless
#     rtc.datetime((datestamp[2],datestamp[1],datestamp[0], 1, timestamp[0], timestamp[1], timestamp[2], 0))
    
    filename = f'{filename_path}{filename_prefix}{date_string}{time_string}{filename_extension}'
    file_header = f'{header}\r\n{filename}\r\n{date_string}\r\n'
    print(f'Writing to {filename}')
    
    with open(filename, "w") as file:  # "w" for new file
        file.write(file_header)
    return filename


def get_current_time():
    # create current time as string 'hhmmss.sss' and as a float
    a = '.'
    m = 'Millisecond overflow.'
    
    get_current_time_start = time.ticks_us() #start timer

    timestamp = my_gps.local_time # tuple of integers
    ts_ms = my_gps.msecs # get the milliseconds portion of the timestamp
    ms_since_fix = my_gps.time_since_fix() # milliseconds since the timestamp
    
    milli = ts_ms + ms_since_fix
    
#     if milli > 999:
#         print(f'{m}')
    TS.get_current_time_collect = time.ticks_diff(time.ticks_us(), get_current_time_start)# get timer diff
    
    start_timer = time.ticks_us() #start timer
    
    current_time_string = f'{timestamp[0]:02d}{timestamp[1]:02d}{timestamp[2]:02d}{a}{milli:003d}'
    
    TS.timcon= time.ticks_diff(time.ticks_us(), start_timer)# get timer diff    
    
    current_time_float = float(current_time_string) # not yet used, but might be useful for timing. msecs may be enough for that purpose, though.

    TS.get_current_time_timer = time.ticks_diff(time.ticks_us(), get_current_time_start)# get timer diff

    return current_time_string, current_time_float # adds hundredths and thousandths or, centiseconds ;-P and milliseconds to the time


async def get_gps_data():
    b = ','
    
    start_timer = time.ticks_us() #start timer

    await my_gps.data_received()
        
    lat = my_gps.latitude()
    if lat[1] == 'N':
        lat_sign = ''
    else: lat_sign = '-'
        
    lon = my_gps.longitude()
    if lon[1] == 'E':
        lon_sign = ''
    else: lon_sign = '-'
    
    speed = my_gps.speed(SPEED_UNITS)
    course = my_gps.course
    
    TS.gpsget= time.ticks_diff(time.ticks_us(), start_timer)# get timer diff
    
    start_timer = time.ticks_us() #start timer

    gps_data = f'{lat_sign}{lat[0]:3.5f}{b}{lon_sign}{lon[0]:3.5f}{b}{speed:3.1f}{b}{course:3.1f}{b}'
    
    TS.gpscon= time.ticks_diff(time.ticks_us(), start_timer)# get timer diff
        
    return gps_data


def get_imu_data():
    b = ','
    
    start_timer = time.ticks_us() #start timer
    
    xG = IMU.readACCx() #* 0.244)/1000 # .244 may become variable, and zero compensation may need implemented
    yG = IMU.readACCy() #* 0.244)/1000
    zG = IMU.readACCz() #* 0.244)/1000
    xGyro = IMU.readGYRx()
    yGyro = IMU.readGYRy()
    zGyro = IMU.readGYRz()
    mag_heading = IMU.readGYRz() # placeholder until implemented.
    
    TS.imuget= time.ticks_diff(time.ticks_us(), start_timer)# get timer diff
    start_timer = time.ticks_us() #start timer
    
    imu_data = f'{xG:1.2f}{b}{yG:1.2f}{b}{zG:1.2f}{b}{xGyro}{b}{yGyro}{b}{zGyro}{b}{mag_heading}{b}'
    
    TS.imucon= time.ticks_diff(time.ticks_us(), start_timer)# get timer diff
    
    return imu_data


    # placeholder of time consuming data for testing. 3 analog signals, unless MCP3208 over spi
def get_acq_data():
    b = ','
    
    start_timer = time.ticks_us() #start timer

    throttle_position  = 1#IMU.readGYRx()
    brake_pressure     = 1#IMU.readGYRx()
    steering_angle     = 1#IMU.readGYRx()
    rpm                = 1#IMU.readGYRx()
    LF_damper_position = 1#IMU.readGYRx()
    RF_damper_position = 1#led.value()
    LR_damper_position = 1#led.value()
    RR_damper_position = 1#led.value()

    TS.acqget = time.ticks_diff(time.ticks_us(), start_timer)# get timer diff
    start_timer = time.ticks_us() #start timer

    acq_data = f'{throttle_position}{b}{brake_pressure}{b}{steering_angle}{b}{rpm}{b}{LF_damper_position}{b}{RF_damper_position}{b}{LR_damper_position}{b}{RR_damper_position}' # put no comma at end

    TS.acqcon= time.ticks_diff(time.ticks_us(), start_timer)# get timer diff
    
    return acq_data


def add_data_line(data_string, current_time_float, imu_data, acq_data, gps_data=None):
    # Adds a line of data to the string. takes 2-3 ms
    b = ','
    c = '\r\n'
    start_timer = time.ticks_us() #start timer
    
    if gps_data == None: # data between gps fix
#         start_timer = time.ticks_us() #start timer
        
        data_string = f'{data_string}{b}{current_time_float}{b}{b}{b}{b}{b}{imu_data}{acq_data}{c}' # gps_data has 4 items

#         diff_timer= time.ticks_diff(time.ticks_us(), start_timer)# get timer diff
#         w = ' ms to write gps data'
#         print(f'{diff_timer}{w}')# print timer
        
    else: # just got a gps fix       
        data_string = f'{data_string}{b}{current_time_float}{b}{gps_data}{imu_data}{acq_data}{c}'

    TS.addline= time.ticks_diff(time.ticks_us(), start_timer)# get timer diff

    return data_string


####################
# Core 1 Save Loop #
####################
 
def write_data():
    # Run on second thread, 'cause it takes so long to write to sd card. about 150 ms
    
    WF.new_data_flag = False
    
    while True: # keep core 1 active
        
        if WF.new_data_flag == True:
        
            baton.acquire() # we dont want main loop to overwrite write_string or call function while still writing
            led.on()
            start_timer = time.ticks_ms() #start timer
            
            with open(filename, "a") as file:  # "a" for appending
                file.write(WF.write_string)
            
            WF.new_data_flag = False
            
            baton.release()
            
            led.off()
            TS.write_timer = time.ticks_diff(time.ticks_ms(), start_timer)# get timer diff
            
        time.sleep_ms(5) # Keep the if-True statement from hammering the core

  
 #######################
 # Main Loop on Core 0 #
 #######################

async def main():
    data_string = ''

    while True: 
     
        for _ in range(2):
            # 2 gps fixes, 12 lines of data collected before
            # incremental save. More lines take long time to concatenate,
            # less lines take too long to save.
            wait_pulse_start = time.ticks_ms() #start timer
            while pps_pin.value() == 0: # if = 1, continue on to collect gps message
                await asyncio.sleep(0)
            TS.waiting_for_gps_pulse = time.ticks_diff(time.ticks_ms(), wait_pulse_start)# get timer diff

            cycle_timer_start = time.ticks_ms() #start timer before getting gps
            gps_data = await get_gps_data()
            
            # Need to deal with no message received

            #   Need better timing between gps time pulses
            for i in range(6): # 6 loops per single 10hz gps fix is 60hz
                if i !=0:
                    cycle_timer_start = time.ticks_ms() #restart timer if not fresh gps
                current_time_string, current_time_float = get_current_time()
                imu_data = get_imu_data() # Gather IMU data
                acq_data = get_acq_data() # Gather adc etc data
                 
                if i == 0: # first time through adds new gps data to string                    
                    data_string = add_data_line(data_string, current_time_string,
                                               imu_data, acq_data, gps_data)
                    
                else : # second through sixth loops dont send gps data to function
                    data_string = add_data_line(data_string, current_time_string,
                                               imu_data, acq_data)
                    
#                 TS.current_time_float = current_time_float # for experimenting
                TS.cycle_timer = time.ticks_diff(time.ticks_ms(), cycle_timer_start)# get timer diff
                TS.selected_data(TS) # Print timing data at 60hz
                
        # For Loops data collection done. Time for save
        baton.acquire() # pause here until core 1 releases lock if necessary
        WF.write_string = data_string 
        WF.new_data_flag = True
        baton.release() # the data for the write is ready, so let core 1 write
        data_string = '' # reset the working variable
        
#         TS.selected_data(TS) # Print timing data at 5hz

#####################
# Get Program Going #
#####################

# Program prep ###############################################################

WF = WriteFlags() # initialize the class flags for file saving
TS = TimeSuck() # initialize the troubleshooting timer class

baton = _thread.allocate_lock() # create the baton to pass when saving data

led = Pin(25, Pin.OUT) # on-board LED used to show saving data
led.off()

# SD Card Mounting -----------------------------------------------------------
SD_spi = SPI(SD_SPI_NUMBER, baudrate=1_000_000, polarity=0, phase=0, bits=8,
             firstbit=machine.SPI.MSB, sck=Pin(SD_SCK_PIN),
             mosi=Pin(SD_MOSI_TX_PIN), miso=Pin(SD_MISO_RX_PIN))
sd = sdcard.SDCard(SD_spi,Pin(SD_CS_PIN, Pin.OUT))
vfs = uos.VfsFat(sd)
uos.mount(vfs, "/sd")

# IMU initialisation ------------------------------------------------------
IMU.initIMU()       #Initialize the accelerometer, gyroscope and compass

# ADC initialization ---------------------------------------------------------
""" # For the future!
adc_spi = SPI(ADC_SPI_NUMBER, baudrate=1_000_000, polarity=0, phase=0, bits=8,
             firstbit=machine.SPI.MSB, sck=Pin(ADC_SCK_PIN),
             mosi=Pin(SD_MOSI_TX_PIN), miso=Pin(ADC_MISO_RX_PIN))
adc = adc_pico.ADConvert(adc_spi,Pin(ADC_CS_PIN, Pin.OUT))
"""

# GPS Communication Initialization --------------------------------------------
# GPS receiver setup
ubx.setup(GPS_UART_NUMBER, GPS_UART_TX_PIN, GPS_UART_RX_PIN,
          GPS_UPDATE_FREQ, GPS_FAST_BAUD, PULSES_PER_SEC, GPS_COLD_REBOOT,DISABLE_NMEA_CLUTTER,
          DISABLE_RMC, DISABLE_UBX_CLUTTER, ENABLE_UBX_PVT)

# Set pico UART baud rate to match the new GPS output 
uartGPS = UART(GPS_UART_NUMBER,baudrate= GPS_FAST_BAUD, tx=Pin(GPS_UART_TX_PIN),
               rx=Pin(GPS_UART_RX_PIN), bits=8, parity=None, stop=1)
print(f'pico fast baudrate set to {GPS_FAST_BAUD}' )
time.sleep_ms(1000)

# Async_GPS setup ------------------------------------------------------
sreader = asyncio.StreamReader(uartGPS)  # Create a StreamReader
my_gps = AS_GPS(sreader,LOCAL_OFFSET)  # Instantiate GPS                                                                          

# Clock / Timing ----------------------------------------------------------
pps_pin = Pin(GPS_PPS_PIN, Pin.IN, Pin.PULL_DOWN) # rising edge is time pulse from gps.
TP = TimingPulse(pps_pin) # instantiate the timing pulse from gps
# rtc = machine.RTC() # Limited value?

# File creation --------------------------------------------------------------
filename = asyncio.run(create_filename()) # uses core 0 for initial startup without threading
time.sleep(1)

# Start second thread on Core 1 to save data to sd card! ---------------------
_thread.start_new_thread(write_data, ()) 

# Start Main Loop on Core 0---------------------------------------------------
asyncio.run(main())

