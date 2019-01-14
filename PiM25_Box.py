#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################             
#            Raspberry Pi 3 GPIO Pinout;           Corner -->  #
#                     (pin 1)  | (pin 2)                       #                  
#br   OLED/GPS Vcc     +3.3V   |  +5.0V    PM25 G3 pin 1 Vcc br#
#y    OLED SDA        GPIO  2  |  +5.0V    MOS Gas Sensor +5V p#     
#o    OLED SCL        GPIO  3  |  GND      MCP3008 GND/GND   br#
#                     GPIO  4  | UART TX                       #
#r    OLED GND         GND     | UART RX                       #
#o    MCP3008 CSbar   GPIO 17  | GPIO 18                       #
#y    MCP3008 MOSI    GPIO 27  |  GND                          #
#g    MCP3008 MISO    GPIO 22  | GPIO 23                       #
#r    MCP3008 Vcc/Vref +3.3V   | GPIO 24   PM25 G3 pin 5 TX   g#
#bl   MCP3008 CLK     GPIO 10  |  GND      PM25 G3 pin 2 GND  o#
#                     GPIO  9  | GPIO 25                       # 
#                     GPIO 11  | GPIO  8                       #
#                      GND     | GPIO  7                       #
#                     Reserved | Reserved                      #
#                     GPIO  5  |  GND                          #
#                     GPIO  6  | GPIO 12   GPS TX              #
#                     GPIO 13  |  GND      GPS GND             #
#b   DHT22 POWER      GPIO 19  | GPIO 16                       #
#w   DHT22 DATA       GPIO 26  | GPIO 20                       #
#gy  DHT22 GND         GND     | GPIO 21                       #
#                    (pin 39)  |(pin 40)                       #                  
################################################################            

gpsstatic = {'latitude':  25.033661,
             'longitude': 121.564841,
             'altitude':  550,
             'n_sats':    0}   # TPE101

from PiM25 import BOX
import time

import numpy as np
import matplotlib.pyplot as plt

# make a box

box = BOX('my box', use_WiFi=True, use_LASS=True, 
          use_SMBus=True, use_pigpio=True, use_MQTT=True)

# add devices

dht   = box.new_DHT22bb('my dht', DATA=26, POWER=19)
g3    = box.new_G3bb('my g3', DATA=24, collect_time = 3.0)
gps   = box.new_GPSbb('my gps', DATA=12, collect_time = 3.0) #for now use static dummy
gpsd  = box.new_Dummy('my dummy gps', gpsstatic)

adc   = box.new_MCP3008bb('my adc', MISO=22, MOSI=27,
                          CSbar=17, SCLK=10, Vref=3.3)

CO2_cal = [[100,   200], [1000, 300], [10000, 400], [100000, 500]]
CO_cal  = [[100,   200], [1000, 300], [10000, 400], [100000, 500]]
# if you don't have MOS gas sensors, just use a resistor divider to make a voltage for the ADC

CO2   = box.new_MOS_Gas_Sensor('my CO2', ADC=adc, channel=5,
                               int_or_float=float, calibdatapairs= CO2_cal,
                               divider_posn='top', R_series=5000, V_supply=5,
                               units_name='ppm', gasname='CO2', xlog=True, ylog=True, xlimitsok=True)

CO    = box.new_MOS_Gas_Sensor('my CO', ADC=adc, channel=6,
                               int_or_float=float, calibdatapairs= CO_cal,
                               divider_posn='top', R_series=5000, V_supply=5,
                               units_name='ppm', gasname='CO', xlog=True, ylog=True, xlimitsok=True)

# start the OLED display
oled      = box.new_OLEDi2c('my oled', rotate180=True)
oled.YAMLsetup('PiM25_Box.yaml')    
oled.initiate() 
oled.Turn_On_Display()
for thing in ('show_white', 'show_black', 'show_gray'):
    getattr(oled, thing)()
oled.show_image('TPE101small.png', resize_method='fit',
                conversion_method='threshold', threshold=60)
time.sleep(2)

print "Thread me!"
oled.start_thread()

# configure the LASS reporter
lass  = box.new_LASS_reporter('mylass')
lass.set_static_location(latlon=(gpsstatic['latitude'], gpsstatic['longitude']),
                         alt   =gpsstatic['altitude']) 

lass.set_sources(humsrc=dht, tempsrc=dht,
                 pm25src=g3, pm1src=g3, pm10src=g3, 
                 timedatesrc='system', GPSsrc='static',   # try not using quotes
                 gassensors=[CO2, CO])  # or use string names

# configure the DATALOGGER 
mydatalogconfig = {dht:['temperature', 'humidity'], 
                   g3:['PM25', 'PM1', 'PM10'],
                   CO2:['value'], CO:['value']}

mydatalog = box.new_DATALOG(filename='mydatalogD.txt', name='mydatalog')
mydatalog.configure(mydatalogconfig)

# check alll readable devices
box.read_all_readables()
oled.update_all_screens()

gps.preview_SNR_plot()

entry = lass.build_entry()

# box.subscribe('test')  # already during test, no unsubscribe yet
print 'box.use_MQTT :', box.use_MQTT
print 'box.use_LASS :', box.use_LASS
print 'box.internet_pingable :', box.internet_pingable


if box.use_MQTT and box.use_LASS and box.internet_pingable:
    box.publish(payload=entry, topic='test', qos=0)
    time.sleep(2)
    box.publish(payload=entry, topic='test', qos=1)
    time.sleep(2)
    box.publish(payload=entry, topic='test', qos=2)

time.sleep(5)

box.read_all_readables()
oled.update_all_screens()

if True:
    n_loops = 22
    loop    = 0
    print "  ###   ###  starting to loop {} times".format(n_loops)
    
    while loop <= n_loops:

        loop += 1
        print "loop {} of {}".format(loop, n_loops)

        box.read_all_readables()
        oled.update_all_screens()

        mydatalog.build_and_save_entry(sysinfo_interval=10)

        entry = lass.build_entry()
        if box.use_MQTT and box.use_LASS and box.internet_pingable:
            box.publish(payload=entry)


