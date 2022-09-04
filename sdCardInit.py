# SD Card Initialization for accessing files in Thonny
# by Jamie Halford

# Copyright (c) 2022 Jamie Halford
# Released under the MIT License (MIT) - see LICENSE file


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


# Standard Pico Libraries ##########################################################

import uos
from machine import UART, Pin, SPI
from math import floor

# Libraries to install in the "lib" folder you create on Pico ###############################################

import sdcard  # https://github.com/micropython/micropython/tree/master/drivers/sdcard

#################################################
# Constants to be modified for your application #
#################################################


# Set the constants below to match your wiring
# SD Card SPI setup ---------------------------------------------------------
SD_SPI_NUMBER      = const(0)        # 0, 1
SD_MISO_RX_PIN     = const(4)        # Pins for SPI0 [0, 4, 16] for SPI1 [8, 12]
SD_CS_PIN          = const(5)        # Pins for SPI0 [1, 5, 17] for SPI1 [9, 13]
SD_SCK_PIN         = const(2)        # Pins for SPI0 [2, 6, 18] for SPI1 [10, 14]
SD_MOSI_TX_PIN     = const(3)        # Pins for SPI0 [3, 7, 19] for SPI1 [11, 15]

#####################
# Get Program Going #
#####################

# SD Card Mounting -----------------------------------------------------------
SD_spi = SPI(SD_SPI_NUMBER, baudrate=1_000_000, polarity=0, phase=0, bits=8,
             firstbit=machine.SPI.MSB, sck=Pin(SD_SCK_PIN),
             mosi=Pin(SD_MOSI_TX_PIN), miso=Pin(SD_MISO_RX_PIN))
sd = sdcard.SDCard(SD_spi,Pin(SD_CS_PIN, Pin.OUT))
vfs = uos.VfsFat(sd)
uos.mount(vfs, "/sd")

