#!/usr/bin/env python
# -*- coding: utf-8 -*-

# SUGGESTED CONNECTIONS, but you can of course do it differenlty!
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


import pigpio, smbus
import atexit 
import re, commands
import psutil    # http://psutil.readthedocs.io/en/latest/

import yaml

import time, datetime
import numpy as np

from binascii import hexlify
from PIL import Image, ImageDraw, ImageFont
import matplotlib.pyplot as plt

import logging
import socket
import paho.mqtt.client as mqtt 

import Queue
from threading import Thread, Lock, Event


# Currently available to future classes, consider moving inside OLED 
OLED_screens_lock = Lock()
OLED_smbus_lock   = Lock()


class BOX(object):

    devkind = "BOX"
    hints   = """  You have to START HERE, all other classes instantiated from box.new_xxx()
box = BOX("name", use_WiFi=False, use_SMBus=True,
use_pigpio=True, use_LASS=False, use_MQTT=False)
use_WiFi=True turns on WiFi and ping the internet
use_MQTT=True will set up an MQTT client, connect, start looping...
subscribe, and test-publish."""

    def __init__(self, name=None, use_WiFi=False, use_SMBus=True,
                 use_pigpio=True, use_LASS=False, use_MQTT=False):

        self.hints = """box.ping_the_internet(pingable_url=None)
box.WiFi_onoff(set_WiFi=None)

start_MQTT()
disconnect_MQTT()
publish(payload, topic='test', qos=1)
subscribe(topic='test', qos=1)
unubscribe(topic='test')

mpu  = box.new_MPU6050i2c(name=None, I2C_clockdiv=9, DLPF=2, divideby=100, 
                          recordgyros=True, recordaccs=True, recordtemp=False,
                          n_gyro_scale=0, n_acc_scale=0, fifo_block_read=True,
                          chunkbytes=24, AD0=1)
step = box.new_STEPPER_MOTOR('my step", max_steprate=20, ramp_rate=10,
                             GPIO_A=GPIO, GPIO_B=GPIO, GPIO_C=GPIO, GPIO_D=GPIO)
G3   = box.new_G3bb("my g3", DATA=GPIO, collect_time=3)
gps  = box.new_GPSbb("my gps", DATA=GPIO, collect_time=3)
dht  = box.new_DHT22bb("my dht", DATA=None, POWER=None)
adc  = box.new_MCP3008bb("my adc", CSbar=GPIO, MISO=GPIO,
                          MOSI=GPIO, SCLK=GPIO,
                          SPI_baud=10000, Vref=3.3)
CO2 = box.new_MOS_Gas_Sensor("my CO2", ADC="my adc", channel=1, 
                        int_or_float=float, calibdatapairs=datapointlist,
                        divider_posn='bottom', R_series=5000, V_supply=5,
                        units_name='ppm', xlog = False, ylog = False,
                        xlimitsok=True)
oled = box.new_OLEDi2c("my oled", rotate180=False)
rtc  = box.new_RTC_smbus("my rtc", baud=None)
dummy= box.new_Dummy("my dummy", dummydatadict=None)
lass = box.new_LASS_reporter("my lass"=None)
dlog = box.new_DATALOG(filename='deleteme.txt', name="my datalog",
                      configure_dict=None)

box.add_device(device)
box.get_device(device)
box.read_all_readables()
box.print_system_info()

box.stopall()"""
        
        self.name = name

        # atexit.register(self.stopall)      # shutdown when exiting python

        # internet and mqtt constants
        self.ping_the_internet_default_url = "www.google.com"

        self.mqtt_host_url                 = "gpssensor.ddns.net"  # for LASS
        self.mqtt_host_url_1               = "test.mosquitto.org"  # other
        self.mqtt_host_url_2               = "iot.eclipse.org"     # other  

        self.mqtt_LASS_PM25_topic          = "LASS/Test/PM25"
        self.mqtt_test_topic               = "LASS/Test/test"

        self.mqtt_keepalive                = 60
        self.mqtt_port                     = 1883

        mqtt.Client.connected_flag         = False
        self.internet_pingable             = None

        # Create and configure logger
        self.logging_level      = logging.DEBUG
        self.logging_filename   = "PiM25box_logging.log"
        # self.logging_format     = "%(levelname)s %(asctime)s - %(message)s"
        self.logging_format     = '%(levelname)s %(asctime)s %(funcName)s %(message)s '
        self.logging_filemode   = 'w'

        logging.basicConfig(filename = self.logging_filename, 
                            level    = self.logging_level, 
                            format   = self.logging_format, 
                            filemode = self.logging_filemode)

        self.logger   = logging.getLogger()
        self.cname    = self.__class__.__name__

        self.logger   = logging.getLogger()

        console       = logging.StreamHandler()
        formatter     = logging.Formatter(self.logging_format, "%Y-%m-%d %H:%M:%S")
        console.setFormatter(formatter)

        console.setLevel(logging.DEBUG)

        logging.getLogger('').addHandler(console)

        msg = (('{x.cname}("{x.name}") \n Instantiated new box ' +
                'named "{x.name}"').format(x=self))
        self.logger.info(msg) 


        msg = (('{x.cname}("{x.name}") \n Started Python logging, ' +
               'level={x.logging_level}, filename="{x.logging_filename}"').format(x=self))
        self.logger.info(msg) 

        # get locals for YAML saving in the future
        self.instance_things = locals()
        donts = ('self', 'name')
        for dont in donts:
            try:
                self.instance_things.pop(dont)
            except:
                pass

        # connections to the world
        self.use_WiFi     = use_WiFi
        self.use_SMBus    = use_SMBus
        self.use_pigpio   = use_pigpio
        self.use_MQTT     = use_MQTT
        self.use_LASS     = use_LASS    # this is for extra safety

        msg = (('{x.cname}("{x.name}") \n use_WiFi={x.use_WiFi}, use_SMBus={x.use_SMBus}, ' + 
                'use_pigpio={x.use_pigpio}').format(x=self))
        self.logger.info(msg) 

        # lists of devices and objects
        self.devices        = []
        self.readables      = []
        self.unreadables    = []
        self.dataloggers    = [] 
        self.LASS_reporters = [] 

        # This Raspberry Pi's MAC address
        self.mac_address    = None
        self.get_mac_address()


        msg = ('{x.cname}("{x.name}") \n Found MAC address {x.mac_address}' +
               'named "{x.name}"'.format(x=self))
        self.logger.info(msg) 

        # This Raspberry Pi's WiFi
        self._get_nWiFi()

        if self.use_WiFi:
            self.WiFi_onoff('on')
            msg = (('{x.cname}("{x.name}") \n WiFi requested').format(x=self))
        else:
            msg = (('{x.cname}("{x.name}") \n WiFi NOT requested').format(x=self))
        self.logger.info(msg)

        WiFi_ison = self.WiFi_onoff()  # double check

        msg = (('{x.cname}("{x.name}") \n currently WiFi_ison={}').format(WiFi_ison, x=self))
        self.logger.info(msg)

        # This Raspberry Pi's internet connection
        self.internet_pingable = self.ping_the_internet()
        if self.internet_pingable:
            msg = (('{x.cname}("{x.name}") \n Internet is pingable, yay!').format(x=self))
        else:
            msg = (('{x.cname}("{x.name}") \n ping_the_internet failed.').format(x=self))
        self.logger.info(msg)

        if True and self.use_WiFi:   # Give some time for the WiFi connection to work
            if not self.internet_pingable:
                print "  no ping, try again in 5 seconds"
                time.sleep(5)
                self.internet_pingable = self.ping_the_internet()
            
            if not self.internet_pingable:
                print "  no ping, try again in 5 seconds"
                time.sleep(5)
                self.internet_pingable = self.ping_the_internet()
            
            if not self.internet_pingable:
                print "  no ping, try again in 10 seconds"
                time.sleep(10)
                self.internet_pingable = self.ping_the_internet()
            
            if not self.internet_pingable:
                print "  no ping, try again in 10 seconds"
                time.sleep(10)
                self.internet_pingable = self.ping_the_internet()

        if not self.internet_pingable:
            print "still no internet connection, BUT NOT setting use_MQTT to False."
            self.use_MQTT = False

        # This Raspberry Pi's MQTT
        if self.use_MQTT:
            status = self.start_MQTT()
            if status==0:
                msg = (('{x.cname}("{x.name}") \n start_MQTT okay ### status={}').format(status, x=self))
                self.logger.info(msg)
            else:
                msg = (('{x.cname}("{x.name}") \n start_MQTT problem ### status={}').format(status, x=self))
                self.logger.warning(msg)

        # This Raspberry Pi's pigpio
        self.pi              = None
        self.pigpiod_process = None
        if self.use_pigpio:
            msg = (('{x.cname}("{x.name}") \n make_a_pi() called').format(x=self))
            self.make_a_pi()
        else:
            msg = (('{x.cname}("{x.name}") \n use pigpio not requested').format(x=self))
            self.logger.info(msg) 

        # This Raspberry Pi's SMBus
        self.bus             = None
        if self.use_SMBus:
            self.bus             = smbus.SMBus(1)
            msg = (('{x.cname}("{x.name}") \n SMBus instantiated').format(x=self))
        else:
            msg = (('{x.cname}("{x.name}") \n SMBus NOT instantiated').format(x=self))
        self.logger.info(msg) 

    def __repr__(self):
        return ('{self.__class__.__name__}("{self.name}")'
                .format(self=self))

    def start_MQTT(self):
        
        self.use_MQTT        = True

        print "trying to connect with MQTT: "
        self.mqtt_client     = mqtt.Client(client_id="Luke, I am your client",
                                           clean_session=True, userdata=None, 
                                           transport="tcp")  # protocol=MQTTv311 or MQTTv31
        
        self.mqtt_client.reconnect_delay_set(min_delay=5, max_delay=120)
        
        self.mqtt_client.on_connect     = self._on_connect
        self.mqtt_client.on_disconnect  = self._on_disconnect
        self.mqtt_client.on_log         = self._on_log # or _on_log_verbose
        self.mqtt_client.on_message     = self._on_message

        self.internet_pingable = self.ping_the_internet()
        
        print "pingable is: ", self.internet_pingable
        
        status_con = self.mqtt_client.connect(host=self.mqtt_host_url,
                                              keepalive=self.mqtt_keepalive,
                                              port=self.mqtt_port)  # 5 https://stackoverflow.com/a/35581280/3904031
        print "connect status: ", status_con

        time.sleep(1)

        status_loop = self.mqtt_client.loop_start()
        print "loop_start status: ", status_loop

        time.sleep(1)

        # test message
        status_pub1 = self.mqtt_client.publish(topic="test", payload="hello!", qos=1, retain=True)   # 7 publish  
        self.test_qos1_mqtt_status    = status_pub1
        print "self.test_qos1_mqtt_status: ", self.test_qos1_mqtt_status

        time.sleep(1)

        status_pub2 = self.mqtt_client.publish(topic="test", payload="hello!", qos=2, retain=True)   # 7 publish  
        self.test_qos2_mqtt_status    = status_pub2
        print "self.test_qos2_mqtt_status: ", self.test_qos2_mqtt_status

        time.sleep(1)

        return status_con

    def _on_log(self, client, userdata, level, buf):
        print("On Log: ", buf)

    def _on_log_verbose(self, client, userdata, level, buf):
        print ("On Log")
        print "    buf:", buf
        print "    level:", level
        print "    userdata:", userdata
        print "    client:", client

    def _on_connect(self, client, userdata, flags, rc):
        print ("Connect code: ", rc)
        if rc == 0:
            client.connected_flag = True
            print ("Connect OK")
        else:
            print ("Connect failed, looping stopped")
            client.loop.stop()

    def _on_disconnect(self, client, userdata, flags, rc=0):
        print ("Disconnect code: ", rc)

    def _on_message(self, client, userdata, msg):
        print "Message received: "
        print "    Topic:   ", msg.topic
        print "    Message: ", str(msg.payload.decode("utf-8", "ignore"))

    def publish(self, payload, topic='test', qos=1):

        status = -1

        if not ((type(payload) is str) and (len(payload)>0)):
            print "bad payload"
            print "type(payload): ", type(payload)
            print "len(payload): ", len(payload)
            return status
        if not ((type(qos) is int) and (qos in (0, 1, 2))):
            print "bad qos"
            return status
        if not ((type(topic) is str) and len(topic)>0):
            print "bad topic"
            return status

        if topic.lower() == 'test':
            topic = self.mqtt_test_topic

        status = self.mqtt_client.publish(topic=topic,
                                          payload=payload,
                                          qos=qos, retain=True)  
        print "published!, status: ", status

        return status

    def subscribe(self, topic='test', qos=1):
        
        status = -1
        
        if not ((type(qos) is int) and (qos in (0, 1, 2))):
            print "bad qos"
            return status
        if not ((type(topic) is str) and len(topic)>0):
            print "bad topic"
            return status

        if topic.lower() == 'test':
            topic = self.mqtt_test_topic[:]

        status   = self.mqtt_client.subscribe(topic=topic, qos=qos)

        print "subscribed to topic={}, status={} ".format(status, topic)

        return status

    def unubscribe(self, topic='test'):
        
        status = -1
        
        if not ((type(topic) is str) and len(topic)>0):
            print "bad topic"
            return status

        if topic.lower() == 'test':
            topic = self.mqtt_test_topic

        status   = self.mqtt_client.unsubscribe(topic=topic)

        print "unubscribed to topic={}, status={} ".format(status, topic)

        return status
    
    def disconnect_MQTT(self, ):

        status = self.mqtt_client.loop_stop()  

        print "loop_stop status: ", status 

        time.sleep(1)

        status = self.mqtt_client.disconnect()  

        print "disconnect status: ", status

        return status

    def ping_the_internet(self, pingable_url=None):

        pingable = False
        try:

            if pingable_url == None:
                pingable_url = self.ping_the_internet_default_url

            s        = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            status   = s.connect_ex((pingable_url, 443))
            pingable = status == 0

        except:
            pass

        return pingable

    def self_check(self):

        msg = (('{x.cname}("{x.name}") \n box "{x.name}" self_check started').format(x=self))
        self.logger.info(msg)

        # check WiFi situation
        wifi_is_on = self.WiFi_onoff()   # check anyway

        msg = (('{x.cname}("{x.name}") \n box "{x.name}" use_WiFi={x.use_WiFi}, '
                + 'WiFi is on={}').format(wifi_is_on, x=self))
        self.logger.info(msg)

        if self.use_WiFi and not wifi_is_on:
            msg = (('{x.cname}("{x.name}") \n ...and that is a problem.').format(x=self))
            self.logger.info(msg)

        # check internet connectivity 
        self.internet_pingable = self.ping_the_internet()
        
        msg = (('{x.cname}("{x.name}") \n box "{x.name}" use_WiFi={x.use_WiFi}, ' +
                'internet pingable={}').format(self.internet_pingable, x=self))
        self.logger.info(msg)

        if self.use_WiFi and not self.internet_pingable:
            msg = (('{x.cname}("{x.name}") \n ...and that is a problem.').format(x=self))
            print (msg)
            self.logger.info(msg)

        if use_pigpio:
            stat, procid = commands.getstatusoutput('sudo pidof pigpiod')
            if not stat:
                msg = (('{x.cname}("{x.name}") \n box "{x.name}" use_pigpio={x.use_pigpio}, ' +
                        'pigpio process ID={}').format(procid, x=self))
            else:
                msg = (('{x.cname}("{x.name}") \n box "{x.name}" use_pigpio={x.use_pigpio}, ' +
                        'but pigpio not running and that is a problem').format(x=self))
            self.logger.info(msg)

    def get_system_timedate_dict(self):

        sysnow              = datetime.datetime.now()
        sysnow_str          = sysnow.strftime("%Y-%m-%d %H:%M:%S")
        sysdate_str         = sysnow_str[:10]
        systime_str         = sysnow_str[11:]
        sysmicroseconds_str = str(sysnow.microsecond)
        
        systimedatedict     = dict()

        systimedatedict['sysnow_str'] = sysnow_str
        systimedatedict['timestr']    = systime_str
        systimedatedict['datestr']    = sysdate_str
        systimedatedict['tickstr']    = sysmicroseconds_str

        return systimedatedict

    def make_a_pi(self):

        status, process = commands.getstatusoutput('sudo pidof pigpiod')

        if status:  #  it wasn't running, so try to start it
            
            msg = (('{x.cname}("{x.name}") \n pigpiod was not running, try to start').format(x=self))
            self.logger.info(msg)
            
            commands.getstatusoutput('sudo pigpiod')  # start it
            time.sleep(0.5)
            status, process = commands.getstatusoutput('sudo pidof pigpiod')   # check it again        

        if not status:  # if it worked or if it's running...

            self.pigpiod_process = process

            msg = (('{x.cname}("{x.name}") \n pigpiod is running; ' +
                    'process ID={x.pigpiod_process}').format(x=self))
            self.logger.info(msg)

            try:
                self.pi = pigpio.pi()  # local GPIO only
                msg = (('{x.cname}("{x.name}") \n Instantiated pi from pigpio').format(x=self))
                self.logger.info(msg)
            except Exception, e:
                str_e = str(e)
                msg = (('{x.cname}("{x.name}") \n problem instantiating pi: {}').format(str_e, x=self))
                self.logger.warning(msg)
                self.start_pigpiod_exception = str_e

    def stop_pigpiod(self):

        command = 'sudo kill {}'.format(self.pigpiod_process)

        status_kill = commands.getstatusoutput(command)

        time.sleep(1)

        status_check, process = commands.getstatusoutput('sudo pidof pigpiod')   # check it again        

        if status_check:
            msg = (('{x.cname}("{x.name}") \n killed pigpiod.').format(x=self))
        else:
            msg = (('{x.cname}("{x.name}") \n tried to kill pigpiod but it is still running ' +
                    'with process ID: {}').format(process, x=self))
            self.logger.info(msg)

    def WiFi_onoff(self, set_WiFi=None):

        if set_WiFi == None:

            WiFi_is_on = None
            try:
                stat, isblocked = commands.getstatusoutput("sudo rfkill list " +
                                                           str(self.nWiFi) +
                                                           " | grep Soft | awk '{print $3}'")
                if isblocked == 'yes':
                    WiFi_is_on = False
                    msg = (('{x.cname}("{x.name}") \n WiFi is off').format(x=self))
                    self.logger.info(msg)
                elif isblocked == 'no':
                    WiFi_is_on = True
                    msg = (('{x.cname}("{x.name}") \n WiFi is on').format(x=self))
                    self.logger.info(msg)
                else:
                    msg = (('{x.cname}("{x.name}") \n I cant tell if WiFi is on or off: ' +
                    '{}, {}').format(stat, out, x=self))
                    self.logger.warning(msg)
            except:
                msg = (('{x.cname}("{x.name}") \n problem checking WiFi status: ' +
                    '{}, {}').format(stat, out, x=self))
                self.logger.warning(msg)
                

        elif ((type(set_WiFi)==str and set_WiFi.lower() == 'on') or
              (type(set_WiFi)==bool and set_WiFi)):
            try:
                stat, out = commands.getstatusoutput("sudo rfkill unblock " +
                                                     str(self.nWiFi))
                if stat:
                    msg = (('{x.cname}("{x.name}") \n problem turning WiFi on: ' +
                    '{}, {}').format(stat, out, x=self))
                    self.logger.warning(msg)
                else:
                    WiFi_is_on = True
            except:
                msg = (('{x.cname}("{x.name}") \n problem turning on WiFi: ' +
                    '{}, {}').format(stat, out, x=self))
                self.logger.warning(msg)

        elif ((type(set_WiFi)==str and set_WiFi.lower() == 'off') or
              (type(set_WiFi)==bool and not set_WiFi)):
            try:
                stat, out = commands.getstatusoutput("sudo rfkill block " +
                                                     str(self.nWiFi))
                if stat:
                    msg = (('{x.cname}("{x.name}") \n problem turning WiFi off: ' +
                    '{}, {}').format(stat, out, x=self))
                    self.logger.warning(msg)
                else:
                    WiFi_is_on = False
            except:
                msg = (('{x.cname}("{x.name}") \n problem turning off WiFi: ' +
                        '{}, {}').format(stat, out, x=self))
                self.logger.warning(msg) 
                
        else:
            msg = (('{x.cname}("{x.name}") \n set WiFi status command not recognized: '+
                    '{}, {}').format(stat, out, x=self))
            self.logger.warning(msg)
            
        return WiFi_is_on 

    def _get_nWiFi(self):

        stat, out = commands.getstatusoutput("sudo rfkill list | grep phy0 | awk '{print $1}'")
        try:
            self.nWiFi = int(out.replace(':', '')) # confirm by checking that it can be an integer
            msg = (('{x.cname}("{x.name}") \n nWiFi={x.nWiFi}').format(x=self))
            self.logger.info(msg)
        except:
            msg = (('{x.cname}("{x.name}") \n there was a problem checking nWiFi!').format(x=self))
            self.logger.warning(msg)
            self.nWiFi = None

    # METHODS that involve MAC address

    def get_mac_address(self):

        ifconfig = commands.getoutput("ifconfig eth0 " +
                                      " | grep HWaddr | " + 
                                      "awk '{print $5}'")
        msg = ("this Raspberry Pi's ifconfig: {}".format(ifconfig))
        self.logger.info(msg)
  
        if type(ifconfig) is str:

            possible_mac = ifconfig.replace(':','')   # alternate

            msg = (('{x.cname}("{x.name}") \n this Raspberry Pi possible MAC address={}')
                   .format(possible_mac, x=self))
            self.logger.info(msg)
  
            if len(possible_mac) == 12:
                self.mac_address = possible_mac
                msg = (('{x.cname}("{x.name}") \n this Raspberry Pi ' +
                        'MAC address={x.mac_address}').format(x=self))
                self.logger.info(msg)

    # METHODS that involve system status

    def _get_system_info(self):
        info_lines = ['', '--------', '--------']
        things     = ('uname -a', 'lsb_release -a', 'df -h', 'free',
                     'vcgencmd measure_temp')
        for thing in things:
            err, msg = commands.getstatusoutput(thing)
            if not err:
                info_lines += ['COMMAND: ' + thing +
                               ' returns: ' + msg, '--------']
            else:
                info_lines += ['COMMAND: ' + thing +
                               ' returns: error', '--------']
        info_lines += ['--------']
        
        return info_lines

    def print_system_info(self):
        info_lines = self._get_system_info()
        for line in info_lines:
            print line

    def get_system_datetime(self):

        sysnow              = datetime.datetime.now()
        sysnow_str          = sysnow.strftime("%Y-%m-%d %H:%M:%S")
        sysdate_str         = sysnow_str[:10]
        systime_str         = sysnow_str[11:]
        sysmicroseconds_str = str(sysnow.microsecond)

        return sysnow_str

    def show_CPU_temp(self):
        temp = None
        err, msg = commands.getstatusoutput('vcgencmd measure_temp')
        #if not err:
            #m = re.search(r'-?\d+\.?\d*', msg)
            #try:
                #temp = float(m.group())
            #except:
                #pass
        # return temp
        return msg
    
    def add_device(self, device):

        if type(device.name) is not str:
            msg = (('{x.cname}("{x.name}") \n device not added, device name is not str: {}')
                   .format(device.name, x=self))
            self.logger.error(msg)
        elif len(device.name) == 0:
            msg = (('{x.cname}("{x.name}") \n device not added, zero-length string ' +
                    'name not allowed').format(x=self))
            self.logger.error(msg)
        elif self.get_device(device.name) is not None:
            msg = (('{x.cname}("{x.name}") \n device with name "{}" not added, ' +
                    'that name is already present').format(device.name, x=self))
            self.logger.error(msg)
        else:
            self.devices.append(device)
            msg = (('{x.cname}("{x.name}") \n device with unique name "{}" successfully added')
                   .format(device.name, x=self))
            self.logger.info(msg)


    def new_LASS_reporter(self, name=None):

        lass = LASS_reporter(box=self, name=name)

        self.LASS_reporters.append(lass)

        msg = (('{x.cname}("{x.name}") \n Adding new instance, name="{y.name}"')
                .format(x=self, y=lass))
        self.logger.info(msg)

        return lass

    def new_DATALOG(self, filename='deleteme.txt', name=None,
                    configure_dict=None):

        dlog = DATALOG(self, filename, name, configure_dict)
        
        self.dataloggers.append(dlog)
        
        msg = (('{x.cname}("{x.name}") \n Adding new instance, name="{y.name}" ' +
                'filename={y.filename} configure_dict={y.configure_dict}').format(x=self, y=dlog))
        self.logger.info(msg)
             
        return dlog

    def new_OLEDi2c(self, name, rotate180=False):

        oled = OLEDi2c(box=self, name=name, rotate180=rotate180)

        msg = (('{x.cname}("{x.name}") \n Adding new instance, name="{y.name}" ' +
                'rotate180={y.rotate180}').format(x=self, y=oled))
        self.logger.info(msg)

        return oled

    def new_G3bb(self, name, DATA=None, collect_time=None):
        
        g3 = G3bb(box=self, name=name, DATA=DATA, collect_time=collect_time)

        msg = (('{x.cname}("{x.name}") \n Adding new instance, name="{y.name}" DATA={y.DATA} '+
                'collect_time={y.collect_time} ').format(x=self, y=g3))
        self.logger.info(msg)

        return g3

    def new_GPSbb(self, name, DATA=None, collect_time=None):

        gps = GPSbb(box=self, name=name, DATA=DATA, collect_time=collect_time)

        msg = (('{x.cname}("{x.name}") \n Adding new instance, name="{y.name}" DATA={y.DATA} '+
                'collect_time={y.collect_time} ').format(x=self, y=gps))
        self.logger.info(msg)

        return gps

    def new_DHT22bb(self, name, DATA=None, POWER=None):

        dht = DHT22bb(box=self, name=name, DATA=DATA, POWER=POWER)

        msg = (('{x.cname}("{x.name}") \n Adding new instance, name="{y.name}" ' +
                'DATA={y.DATA} POWER={y.POWER} ').format(x=self, y=dht))
        self.logger.info(msg)

        return dht
        
    def new_MCP3008bb(self, name, CSbar=None, MISO=None,
                      MOSI=None, SCLK=None,
                      SPI_baud=None, Vref=None):

        mcp3008 = MCP3008bb(box=self, name=name, CSbar=CSbar,
                            MISO=MISO, MOSI=MOSI, SCLK=SCLK,
                            SPI_baud=SPI_baud, Vref=Vref)

        msg = (('{x.cname}("{x.name}") \n Adding new instance, name="{y.name}" CSbar={y.CSbar} '+
                'MISO={y.MISO} MOSI={y.MOSI} SCLK={y.SCLK} SPI_baud={y.SPI_baud} Vref={y.Vref}')
               .format(x=self, y=mcp3008))
        self.logger.info(msg)

        return mcp3008

    def new_Analog_Device(self, name, ADC=None, channel=None,
                          int_or_float=float, calibdatapairs=None,
                          units_name=None, xlog = False, ylog = False,
                          xlimitsok=True):
        
        device = Analog_Device(box=self, name=name, 
                               ADC=ADC, channel=channel,
                               int_or_float=int_or_float,
                               calibdatapairs=calibdatapairs, units_name=units_name,
                               xlog = xlog, ylog = ylog, xlimitsok=xlimitsok)

        msg = (('{x.cname}("{x.name}") \n Adding new instance, name="{y.name}" ADC={y.ADC} '+
                'channel={y.channel} int_or_float={y.int_or_float} units_name={y.units_name} ' +
                'xlog={y.xlog} ylog={y.ylog} xlimitsok = {y.xlimitsok}' +
                'calibdatapairs={y.rawcalibdatapairs} ').format(x=self, y=device))
        self.logger.info(msg)

        return device


    def new_MOS_Gas_Sensor(self, name, ADC=None, channel=None,
                           int_or_float=float, calibdatapairs=None,
                           divider_posn=None, R_series=None, V_supply=None,
                           units_name=None, gasname=None, xlog = False, ylog = False,
                           xlimitsok=True):
        
        device = MOS_Gas_Sensor(box=self, name=name, 
                               ADC=ADC, channel=channel, int_or_float=int_or_float,
                               calibdatapairs=calibdatapairs, divider_posn=divider_posn,
                               R_series=R_series, V_supply=V_supply, units_name=units_name, gasname=gasname, 
                               xlog = xlog, ylog = ylog, xlimitsok=xlimitsok)

        msg = (('{x.cname}("{x.name}") \n Adding new instance, name="{y.name}" ADC={y.ADC} '+
                'channel={y.channel} int_or_float={y.int_or_float} divider_posn={y.divider_posn} '+
                 'R_series={y.R_series} V_supply={y.V_supply} units_name={y.units_name} '+
                'gasname={y.gasname} xlog={y.xlog} ylog={y.ylog} xlimitsok = {y.xlimitsok}' +
                'sortedcalibdatapairs={y.sortedcalibdatapairs} ').format(x=self, y=device))

        self.logger.info(msg)

        return device

    def new_RTC_smbus(self, name=None, baud=None):

        rtc = RTC_smbus(box=self, name=name, baud=baud)

        msg = (('{x.cname}("{x.name}") \n Adding new instance, ' +
                'name="{y.name}" ').format(x=self, y=rtc))
        self.logger.info(msg)

        return rtc

    def new_Dummy(self, name, dummydatadict=None):

        dummy = Dummy(box=self, name=name, dummydatadict=dummydatadict)

        msg = (('{x.cname}("{x.name}") \n Adding new instance, name="{x.name}" ' +
                'dummydatadict={y.dummydatadict}').format(x=self, y=dummy))
        self.logger.info(msg)

        return dummy

    def new_STEPPER_MOTOR(self, name, max_steprate=None, ramp_rate=None,
                          GPIO_A=None, GPIO_B=None, GPIO_C=None, GPIO_D=None):

        stepper = STEPPER_MOTOR(box=self, name=name, max_steprate=max_steprate,
                                ramp_rate=ramp_rate, GPIO_A=GPIO_A,
                                GPIO_B=GPIO_B, GPIO_C=GPIO_C, GPIO_D=GPIO_D)

        msg = (('{x.cname}("{x.name}") \n Adding new instance, name="{y.name}"'+
                'max_steprate={y.max_steprate}, ramp_rate={y.ramp_rate}, ' +
                'four_GPIOS = {y.four_GPIOS}').format(x=self, y=stepper))
        self.logger.info(msg)

        return stepper

    def new_MPU6050i2c(self, name=None, I2C_clockdiv=None, DLPF=None,
                       divideby=None, recordgyros=None, recordaccs=None,
                       recordtemp=None, n_gyro_scale=None, n_acc_scale=None,
                       fifo_block_read=None, chunkbytes=None, AD0=None):

        mpu = MPU6050i2c(box=self, name=name, I2C_clockdiv=I2C_clockdiv,
                         DLPF=DLPF, divideby=divideby, recordgyros=recordgyros,
                         recordaccs=recordaccs, recordtemp=recordtemp,
                         n_gyro_scale=n_gyro_scale, n_acc_scale=n_acc_scale,
                         fifo_block_read=fifo_block_read, chunkbytes=chunkbytes,
                         AD0=AD0)

        msg = (('{x.cname}("{x.name}") \n Adding new instance, name="{y.name}"').format(x=self, y=mpu))
        self.logger.info(msg)

        return mpu

    def new_HMC5883i2c(self, name):

        mag = HMC5883i2c(box=self, name=name)

        msg = (('{x.cname}("{x.name}") \n Adding new instance, name="{y.name}"').format(x=self, y=mag))
        self.logger.info(msg)

        return mag

    def clear_all_readables(self):

        for device in self.readables:
            device.datadict = dict()    # easiest way to clear!

    def read_all_readables(self):

        results = []

        for device in self.readables:
            device.read()
            results.append((str(device), device.ierr))

        return results            

    def get_device(self, dev):

        device = None

        if dev in self.devices:
            device = dev
        else:
            try:
                device = [d for d in self.devices if d.name == dev][0]
            except:
                pass
        return device

    def stopall(self):

        oleds = [dev for dev in self.devices if 'oled' in dev.devkind.lower()]

        print "   OLEDS: ", len(oleds)

        for o in oleds:
            print o
            try:
                o.stop_thread()
                print '  successfully stoppped'
            except:
                print '  UNsuccessfully stoppped'
                pass

            # o.Turn_Off_Display()    # hey try NOT DOING this 

        try:
            self.disconnect_MQTT()
            print '  MQTT successfully disconnected'
        except:
            print '  MQTT UNsuccessfully disconnected'
            pass
        print "     all stop!"

class LASS_reporter(object):

    devkind = "LASS"
    
    def __init__(self, box, name=None):
    
        self.hints = """set_static_location(latlon=tuple, alt=None)
set_sources(humsrc=None, tempsrc=None, pm25src=None,
            pm1src=None, pm10src=None, timedatesrc=None,
            GPSsrc=None, gassensors=None)
build_entry()
_generate_LASS_string()"""

        self.box              = box
        self.name             = name

        self.cname = self.__class__.__name__

        self.mac_address      = box.mac_address
        
        self.last_system_info = None    # double check this should be here
        
        self.devices          = []

        # six static box parameters for LASS
        self.app              = 'PiM25'
        self.ver_app          = '0.1.0'
        self.device           = 'PiM25Box ' + name
        self.device_id        = self.box.mac_address
        self.ver_format       = 3 # now version 3 see https://lass.hackpad.tw/LASS-data-format-9OcSHfWwyUx 
        self.fmt_opt          = 1 # (0) default (real GPS) (1) gps information invalid   always 0 or 1

        self.battery_level_static    = 100.0
        self.battery_mode_static     = 1.0
        self.motion_speed_static     = 0.0
        self.CPU_utilization_static  = 0.0  # Hey link this up

        self.sequence_number         = 1 

        self.static_lat       = None
        self.static_lon       = None
        self.static_alt       = None
        self.static_fix       = 0
        self.static_num       = 0

        msg = (('{x.cname}("{x.name}") \n New instance named "{x.name}"')
               .format(x=self))
        self.box.logger.info(msg)

    def __repr__(self):
        return ('{self.__class__.__name__}("{self.name}")'
                .format(self=self))

    def set_static_location(self, latlon=tuple, alt=None):

        self.static_latlon = latlon
        self.static_alt    = alt
        if type(latlon) == tuple and len(latlon) >= 2:
            if all([type(x) in (float, int) for x in latlon[:2]]):
                self.static_lat = latlon[0]
                self.static_lon = latlon[1]
                msg = (('{x.cname}("{x.name}") \n set static_lat={x.static_lat} ' +
                        'static_lon={x.static_lon}').format(x=self))
                self.box.logger.info(msg)
            elif all([type(x) is str for x in latlon[:2]]):
                try:
                    lat, lon = [float(x) for x in latlon[:2]]
                    self.static_lat = lat
                    self.static_lon = lon
                    msg = (('{x.cname}("{x.name}") \n set static_lat={x.static_lat} ' +
                            'static_lon={x.static_lon}').format(x=self))
                    self.box.logger.info(msg)
                except:
                    msg = (('{x.cname}("{x.name}") \n static latitude and longitude set ' +
                            'has failed!').format(x=self))
                    self.box.logger.error(msg)
                    pass

        if type(alt) is float:
            self.static_alt = alt
            msg = "sstatic altitude set: {}".format(self.static_alt) 
            self.box.logger.info(msg)
        
    def set_sources(self, humsrc=None, tempsrc=None, pm25src=None,
                    pm1src=None, pm10src=None, timedatesrc=None,
                    GPSsrc=None, gassensors=None):
        
        self.source_dict = dict()
        
        self._lookup = {'humidity':{'DHT22':'s_h0', 'HTS221':'s_h1', 'SHT31':'s_h2',
                              'HTU21D':'s_h3', 'BME280':'s_h4', 'SHT25':'s_h5',
                              'other':'s_h9'}, 
                  'temperature':{'DHT22':'s_t0', 'HTS221':'s_t1', 'SHT31':'s_t2',
                              'HTU21D':'s_t3', 'BME280':'s_t4', 'SHT25':'s_t5',
                              'other':'s_t9'},
                  'PM25':{'G3':'s_d0', 'Panasonic':'s_d3', 'other':'s_d7'},
                  'PM1' :{'G3':'s_d1', 'other':'s_d8'},
                  'PM10':{'G3':'s_d2', 'other':'s_d9'}}

        self._gaslookup = {'NH3':'s_g0', 'CO':'s_g1', 'NO2':'s_g2', 'C3H8':'s_g3',
                     'C4H10':'s_g4', 'CH4':'s_g5', 'H2':'s_g6',
                     'C2H5OH':'s_g7', 'CO2':'s_g8', 'TVOC':'s_gg'}

        if humsrc:
            param, source = 'humidity',    humsrc
            self.source_dict[self._lookup[param][source.devkind]] = (source, param)
            msg = (('{x.cname}("{x.name}") \n humsrc={} param={} ').format(source, param, x=self))
            self.box.logger.info(msg)

        if tempsrc:
            param, source = 'temperature', tempsrc
            self.source_dict[self._lookup[param][source.devkind]] = (source, param)
            msg = (('{x.cname}("{x.name}") \n tempsrc={} param={} ').format(source, param, x=self))
            self.box.logger.info(msg)
       
        if pm25src:
            param, source = 'PM25',        pm25src
            self.source_dict[self._lookup[param][source.devkind]] = (source, param)
            msg = (('{x.cname}("{x.name}") \n pm25src={} param={} ').format(source, param, x=self))
            self.box.logger.info(msg)
             
        if pm1src:
            param, source = 'PM1',        pm1src
            self.source_dict[self._lookup[param][source.devkind]] = (source, param)
            msg = (('{x.cname}("{x.name}") \n pm1src={} param={} ').format(source, param, x=self))
            self.box.logger.info(msg)
       
        if pm10src:
            param, source = 'PM10',        pm10src
            self.source_dict[self._lookup[param][source.devkind]] = (source, param)
            msg = (('{x.cname}("{x.name}") \n pm10src={} param={} ').format(source, param, x=self))
            self.box.logger.info(msg)

        if not gassensors:
            gassensors = []

        for sensor in gassensors:
            param, source, gasname = 'ppm', sensor, sensor.devkind
            self.source_dict[self._gaslookup[gasname]] = (source, param)
            msg = (('{x.cname}("{x.name}") \n gas sensor={} param={} gasname={} ')
                    .format(source, param, gasname, x=self))
            self.box.logger.info(msg)

        if type(GPSsrc) is str and GPSsrc.lower() == 'static':
            self.fmt_opt                = 1  # (1) gps information invalid   always 0 or 1
            self.source_dict['gps_lat'] = ('static', 'static_lat')
            self.source_dict['gps_lon'] = ('static', 'static_lon')
            self.source_dict['gps_alt'] = ('static', 'static_alt')
            self.source_dict['gps_fix'] = ('static', 'static_fix')
            self.source_dict['gps_num'] = ('static', 'static_num')
            msg = (('{x.cname}("{x.name}") \n GPSsrc={}').format('"static"', x=self))
            self.box.logger.info(msg)
            
        else:
            self.fmt_opt                = 0 # (0) default (real GPS)            self.source_dict['gps_lat'] = (GPSsrc,   'latitude' )
            self.source_dict['gps_lat'] = (GPSsrc,   'latitude')
            self.source_dict['gps_lon'] = (GPSsrc,   'longitude')
            self.source_dict['gps_alt'] = (GPSsrc,   'altitude' )
            self.source_dict['gps_fix'] = (GPSsrc,   'fix'      )
            self.source_dict['gps_num'] = (GPSsrc,   'n_satsused'   )
            msg = (('{x.cname}("{x.name}") \n GPSsrc={}').format(GPSsrc, x=self))
            self.box.logger.info(msg)

        if type(timedatesrc) is str and timedatesrc.lower() == 'system':
            self.source_dict['time']  = ('system',  'timestr')
            self.source_dict['date']  = ('system',  'datestr')
            self.source_dict['ticks'] = ('system',  'tickstr')
            msg = (('{x.cname}("{x.name}") \n timedatesrc={}').format('"system"', x=self))
            self.box.logger.info(msg)

        else:
            self.source_dict['time']  = (timedatesrc, 'timestr')
            self.source_dict['date']  = (timedatesrc, 'datestr')
            self.source_dict['ticks'] = ('system',    'tickstr')
            msg = (('{x.cname}("{x.name}") \n timedatesrc={}').format(timedatesrc, x=self))
            self.box.logger.info(msg)

        # 's_gx' g0,  g1, g2,  g3,   g4,    g5,  g6  g7,     g8,              gg
        # 's_gx' NH3, CO, NO2, C3H8, C4H10, CH4, H2, C2H5OH, SenseAir S8 CO2, TVOC
        # 's_hx' h0,    h1,     h2,    h3,     h4,     h5,
        # 's_hx' DHT22, HTS221, SHT31, HTU21D, BME280, SHT25
        # 's_tx' s_t0,   s_t1,  s_t2,   s_t3,   s_t4,   s_t5
        # 's_tx' DHT22, HTS221, SHT31, HTU21D, BME280, SHT25
        # 's_dx' d0,    d1,   d2   d3
        # 's_dx' PM2.5, PM10, PM1, Panasonic

    def build_entry(self):

        self.LASS_data = []

        # static box information
        self.LASS_data.append('ver_format=' + str(self.ver_format))
        self.LASS_data.append('fmt_opt='    + str(self.fmt_opt))
        self.LASS_data.append('app='        + str(self.app))
        self.LASS_data.append('ver_app='    + str(self.ver_app))
        self.LASS_data.append('device_id='  + str(self.device_id))
        self.LASS_data.append('device='     + str(self.device))

        systdd = self.box.get_system_timedate_dict()

        for key, (source, param) in self.source_dict.items():
            if (key in ('time', 'date', 'ticks') and source == 'system'):
                thing = systdd[param]
                if type(thing) is str and len(thing) >=8:
                    self.LASS_data.append(key  + '=' + systdd[param])
            elif ('gps' in key and source == 'static'):
                thing = getattr(self, param)
                if thing != None:
                    self.LASS_data.append(key  + '=' + str(thing) )
            else:
                try:
                    thing = source.datadict[param]
                    if thing != None:
                        self.LASS_data.append(key  + '=' + str(thing))
                except:
                    pass
                           
        # https://www.saltycrane.com/blog/2008/11/python-datetime-time-conversions/
        
        # sequence number
        self.LASS_data.append('s_0=' + str(self.sequence_number))
        self.sequence_number += 1

        # battery level 
        self.LASS_data.append('s_1=' + str(self.battery_level_static))

        # battery mode
        self.LASS_data.append('s_2=' + str(self.battery_mode_static))

        # motion speed 
        self.LASS_data.append('s_3=' + str(self.motion_speed_static))

        # CPU utilization
        # http://psutil.readthedocs.io/en/latest/
        self.CPU_utilization = psutil.cpu_percent()
        self.LASS_data.append('s_4=' + str(self.CPU_utilization))

        string = self._generate_LASS_string()

        return string

    def _generate_LASS_string(self):
        
        self.LASS_string =  '|'.join([''] + self.LASS_data + [''])

        msg = (('{x.cname}("{x.name}") \n LASS_string generated={x.LASS_string}')
                .format(x=self))
        self.box.logger.info(msg)

        return self.LASS_string


        # ['ver_format', 'fmt_opt', 'app', 'ver_app', 'device_id', 'tick',
        # 'date', 'time', 'device', 's_0', 's_1', 's_2', 's_3', 's_d0',
        # 's_t0', 's_h0', 'gps_lat', 'gps_lon', 'gps_fix', 'gps_num',
        # 'gps_alt']

        # time hh:mm:ss
        # date yyyy-mm-dd

        # 's_0' # sequence number
        # 's_1' # battery level
        # 's_2' # battery mode vs charging
        # 's_3' # motion speed
        # 's_4' # CPU utilization
        
        # 's_bx' # barometer b0, b1, b2 = Grove, BMP180, BME280
        # 's_dx' # dust  d0, d1, d2 is PM2.5, PM10, PM1, d3 Panasonic
        # 's_gx' # gas g0, g1, g2, g3, g4, g5, g6 g7, g8, gg
        # 's_gx' # gas NH3, CO, NO2, C3H8, C4H10, CH4, H2, C2H5OH, SenseAir S8 CO2, TVOC
        # 's_hx' # h0, h1, h2, h3, h4, h5, DHT22, HTS221, SHT31, HTU21D, BME280, SHT25

        # 's_Ix' # light
        # 's_nx' # radiation
        # 's_ox' # other, misc
        # 's_px' # rain
        # 's_sx' # sound
        # 's_tx' # temperature, t0, t1, t2, t3, t4, t5 is DHT22, HTS221, SHT31, HTU21D, BME280, SHT25
        # 's_wx' # winds w0, w1 speed, direction
        # 's_rx' # rainfall r10, r60 is 10 and 60 minutes


class DATALOG(object):
    
    devkind = "DATALOG"

    def __init__(self, box, name=None, filename=None, configure_dict=None):

        self.hints = """configure(configure_dict)
build_entry(sysinfo_interval=None)
save_entry()
build_and_save_entry(sysinfo_interval=None)"""    

        self.box                = box
        self.name               = name
        self.filename           = filename 
        self.configure_dict      = configure_dict 
        self.devices            = []

        self.cname = self.__class__.__name__

        self.was_opened_okay    = False

        if configure_dict:
            self.configure(configure_dict=configure_dict)

        msg = (('{x.cname}("{x.name}") \n Instantiating logfilename="{x.filename}"')
               .format(x=self))
        self.box.logger.info(msg)

        self.t_previous_sysinfo = None

        # this random number will be an identifier of this log file
        # both for the user, and to double check the file's identity
        self.randint = np.random.randint(1000000000) # 9 digit random integer
 
        headerlines = [] 
        headerlines.append(str(self.randint))
        
        headerlines.append('New DATALOG file, filename = ' + str(self.filename))
        headerlines.append('New DATALOG file, log name = ' + str(self.name))

        datetimedict = self.box.get_system_timedate_dict()

        headerlines.append('Time = ' + datetimedict['timestr'])
        headerlines.append('Date = ' + datetimedict['datestr'])
        headerlines.append('box name = ' + str(self.box.name))
        try:
            headerlines.append('box MAC address = ' +
                               str(self.box.macaddress))
        except:
            pass

        try:
            with open(filename, 'w') as outfile:
                outfile.writelines([line + '\n' for line in headerlines])
            self.was_opened_okay    = True
        except:
            msg = (('{x.cname}("{x.name}") \n Problem opening logfilename="{x.filename}"').format(x=self))
            self.box.logger.warning(msg)

        msg = (('{x.cname}("{x.name}") \n New instance named "{x.name}", filename="{x.filename}" ' +
                'configure_dict={x.configure_dict}').format(x=self))
        self.box.logger.info(msg)

    def __repr__(self):
        return ('{self.__class__.__name__}("{self.name}")'
                .format(self=self))

    def configure(self, configure_dict):
        
        for device, datakeys in configure_dict.items():

            device = self.box.get_device(device)

            if device:

                self.devices.append((device, datakeys))

                msg = (('{x.cname}("{x.name}") \n device "{x.name}" added').format(x=self))
                self.box.logger.info(msg)
                # does not test if datakeys are there because device may be dynamic.
            else:
                msg = (('{x.cname}("{x.name}") \n device "{x.name}" NOT added').format(x=self))
                self.box.logger.warning(msg)

    def build_entry(self, sysinfo_interval=None):       

        self.datadict = dict()
        
        self.datadict['buildtime'] = self.box.get_system_datetime()

        try:
            time_since = time.time() - self.t_previous_sysinfo
        except:
            time_since = None

        if ((time_since == None) or
            (time_since > sysinfo_interval) or
            (sysinfo_interval<=0)):

            sysinfolines = self.box._get_system_info()

            self.datadict['sysinfolines'] = sysinfolines

            self.t_previous_sysinfo = time.time()


        for device, datakeys in self.devices:
            
            devdict = dict()

            self.datadict[device.name] = devdict

            for dk in datakeys:

                try:
                    devdict[dk] = device.datadict[dk]
                except:
                    pass

    def save_entry(self):

        lines = []
        try:
            lines += self.datadict.pop('sysinfolines')  # ensures old stuff not reused
        except:
            pass
        for key, info in self.datadict.items():
            lines += [key]
            if type(info) is list:
                lines += info
            elif type(info) is str:
                lines += [info]
            elif type(info) is dict:
                for datakey, data in info.items():
                    lines += ['  datakey: ' + datakey + ' = ' + str(data)]

        with open(self.filename, 'a') as outfile:   # note, append!!!
            outfile.writelines([line + '\n' for line in lines])
                
        self.log_entry_lines = lines   # save for debugging

    def build_and_save_entry(self, sysinfo_interval=None):
        
        self.build_entry(sysinfo_interval=sysinfo_interval)
        self.save_entry()

class GPIO_DEVICE(object):

    def __init__(self, box, name=None):

        self.box              = box
        self.pi               = self.box.pi
        self.bus              = self.box.bus
        self.name             = name
        self.datadict         = dict()
        self.statistics       = {'nreads':0, 'ngoodreads':0, 
                                 'nbadreads':0} # minimal each may have more

        self.cname = self.__class__.__name__

        self.last_twenty_ierr    = []
        self.lastdata            = ' '
        
        self.box.add_device(self)
        
        self.readable = hasattr(self, 'read')
        if self.readable:
            self.box.readables.append(self)
        else:
            self.box.unreadables.append(self)

        donts = ('self', 'name', 'box')
        for dont in donts:
            try:
                self.instance_things.pop(dont)
            except:
                pass

    def __repr__(self):
        return (('{self.__class__.__name__}("{self.name}") ' +
                self.lastdata) .format(self=self))

    def get_my_current_instance_info(self):

        current_info = dict()
        for key in self.instance_things:
            current_info[key] = getattr(self, key)
        return current_info

    def get_my_original_instance_info(self):

        original_info = self.instance_things.copy()

        return original_info

    def _last_twenty_ierr_increment(self): 

        self.last_twenty_ierr = ([self.ierr] +
                                  self.last_twenty_ierr[:19])

class SCREEN(object):
    
    devkind = "SCREEN"
    
    def __init__(self, nxy, name=None):

        self.hints = """new_textfield(name, xy0, wh=None,
                  fmt=None, fontdef=None, fontsize=None,
                  threshold=None, info=None)
new_imagefield(name, xy0, wh=None, info=None)
add_imagefield(f, xy0) 
get_imagefield(fieldref) 
add_textfield(f, xy0) 
get_textfield(fieldref) 
update()
preview_me()
update_and_preview_me()  
show_image(filename, resize_method, conversion_method, threshold=None)"""

        self.name             = name
        self.textfields       = dict()
        self.imagefields      = dict()
        self.nxy              = nxy
        self.nx, self.ny      = self.nxy

        self.array = np.zeros(self.nxy[::-1], dtype=int)

    def __repr__(self):
        return ('{self.__class__.__name__}("{self.name}")'
                .format(self=self))

    def new_textfield(self, name, xy0, wh=None,
                  fmt=None, fontdef=None, fontsize=None,
                  threshold=None, info=None):      
        
        f = TEXTFIELD(name, wh, fmt, fontdef, fontsize, threshold, info)
        self.add_textfield(f, xy0)

        f.box = self.box

        return f
    
    def add_textfield(self, f, xy0): 

        self.textfields[f] = xy0
        f.screens.append(self)

        return f

    def new_imagefield(self, name, xy0, wh=None, info=None):      
        
        f = IMAGEFIELD(name, wh, info)
        self.add_imagefield(f, xy0)
        return f
    
    def add_imagefield(self, f, xy0): 

        self.imagefields[f] = xy0
        f.screens.append(self)
        f.box = self.box
        return f

    def get_imagefield(self, fieldref): 

        field = None

        if fieldref in self.imagefields:
            field = fieldref
        else:
            try:
                field = [f for f in self.imagefields if f.name == fieldref][0]
            except:
                pass
        return field

    def get_textfield(self, fieldref): 
        
        field = None

        if fieldref in self.textfields:
            field = fieldref
        else:
            try:
                field = [f for f in self.textfields if f.name == fieldref][0]
            except:
                pass
        return field

    def _embed(self, small_array, big_array, big_index):

        slices = [np.s_[i:i+j] for i,j in zip(big_index, small_array.shape)]
        big_array[slices] = small_array
        try:
            big_array[slices] = small_array
        except:
            msg = (('{x.cname}("{x.name}") \n field embed failed').format(x=self))
            self.box.logger.warning(msg)

    def _update_array(self):

        with OLED_screens_lock:

            self.array = np.zeros(self.nx * self.ny, dtype=int).reshape(self.ny, self.nx)

            for field, xy0 in self.textfields.items():

                self._embed(field.array, self.array, xy0[::-1])  

            for field, xy0 in self.imagefields.items(): 

                self._embed(field.array, self.array, xy0[::-1])

    def update(self):
            
        for field in self.textfields:

            field.update()

        for field in self.imagefields:

            field.update()

        self._update_array()

    def preview_me(self):

        plt.figure()
        plt.imshow(self.array, cmap='gray', interpolation='nearest')
        plt.show()

    def update_and_preview_me(self):
        
        self.update()
        self.preview_me()

    def show_image(self, filename, resize_method, conversion_method, threshold=None):

        img0   = Image.open(filename)
        w0, h0 = img0.width, img0.height

        msg = (('{x.cname}("{x.name}") \n showimage opened size w0, h0={}').format((w0, h0), x=self))
        self.box.logger.info(msg)  

        if resize_method == 'stretch':

            new_size = (self.w, self.h)

            imgr = img0.resize(new_size)

            msg = (('{x.cname}("{x.name}") \n showimage new_size stretch new_size={}').format(new_size, x=self))
            self.box.logger.info(msg)

        elif resize_method == 'fit':

            wscale = float(self.nx)/float(w0)
            hscale = float(self.ny)/float(h0)

            if hscale <= wscale:
                new_size = (int(w0*hscale), self.ny)
            else:
                new_size = (self.nx, int(h0*hscale))

            imgr = img0.resize(new_size)

            msg = (('{x.cname}("{x.name}") \n showimage new_size fit new_size={}').format(new_size, x=self))
            self.box.logger.info(msg)

        else:

            imgr = None

            msg = (('{x.cname}("{x.name}") \n showimage new_size failed').format(x=self))
            self.box.logger.error(msg)

        if imgr:

            if conversion_method in ('default', 'dither'):

                imgc  = imgr.convert("1")

                msg = (('{x.cname}("{x.name}") \n showimage convert "default"').format(x=self))
                self.box.logger.info(msg)

            elif conversion_method in ('threshold', ):

                imgr8 = imgr.convert("L")
                imgc  = imgr8.point(lambda x: 0 if x < threshold else 255, mode='1')

                msg = (('{x.cname}("{x.name}") \n showimage convert "threshold"').format(x=self))
                self.box.logger.info(msg)

        else:

            imgc = None

            msg = (('{x.cname}("{x.name}") \n showimage conversion failed').format(x=self))
            self.box.logger.error(msg)

        if imgc:
            
            wni, hni = imgc.width, imgc.height
            array    = np.array(list(imgc.getdata())).reshape(hni, wni)

            bigarray = np.zeros(self.h*self.w, dtype=int).reshape(self.h, self.w)

            sa0, sa1 = array.shape
            sb0, sb1 = bigarray.shape

            hoff = max(0, (sb0-sa0)/2-1)
            woff = max(0, (sb1-sa1)/2-1)
            
            _embed(array, bigarray, (hoff, woff))

            arrayf = bigarray.astype(float)
            arrayf = arrayf / arrayf.max()
            plt.figure()
            plt.imshow(arrayf)
            plt.show()

class TEXTFIELD(object):   ### THIS is the one now!

    devkind = "TEXTFIELD"
    
    def __init__(self, name, wh=None,
                 fmt=None, fontdef=None, fontsize=None,
                 threshold=None, info=None):

        self.hints = """update()
preview_me()
update_and_preview_me()"""

        self.name            = name
        self.wh              = wh

        self.screens         = []

        try:
            self.w, self.h   = self.wh[:2]
        except:
            pass

        self.fmt             = fmt

        self.fontdef         = fontdef

        if type(fontsize) is int:
            self.fontsize    = fontsize
        else:
            self.fontsize    = 14

        if self.fontdef == 'default':
            self.font  = ImageFont.load_default()
        elif type(self.fontdef) is str:
            if self.fontdef[-4:].lower() == '.ttf':
                self.font = ImageFont.truetype(self.fontdef,
                                               self.fontsize)
            else:
                msg = "fontdef problem!"
                self.box.logger.warning(msg)
            
        self.threshold       = threshold
        self.info            = info
        self.text            = ''

    def __repr__(self):
        return ('{self.__class__.__name__}("{self.name}")'
                .format(self=self))

    def update(self):

        self._update_string()
        self._generate_array()

    def _get_pairs(self, fmt):
        pairs =[(m.start(), m.end()) for m in re.finditer('{([^{)]*)}', fmt)]
        return pairs

    def _stringit(self, fmt, data):
        xchar = '##'
        xchar_alt = u'\u25A1\u25A0'
        pairs = self._get_pairs(fmt)
        if len(pairs) == 0:
            s = fmt.format()
        else:
            keepers = [fmt[:pairs[0][0]]]
            for i in range(len(pairs)-1):
                keepers.append(fmt[pairs[i][1]:pairs[i+1][0]])
            keepers.append(fmt[pairs[-1][1]:])
            zingers = [fmt[p[0]:p[1]] for p in pairs]
            newdata = []
            newfmt  = keepers[0]
            for d, z, k in zip(data, zingers, keepers[1:]):
                if d == None:
                    newfmt += xchar + k
                else:
                    newdata.append(d)
                    newfmt += z + k
            try:
                s = newfmt.format(*newdata)
            except:
                s = ''
                msg = "string formatting problem!"
                self.box.logger.warning(msg)
        return s

    def _update_string(self):
        self.string = ''
        self.values = []
        for device, key in self.info:
            try:
                dev = self.box.get_device(device)
                value = dev.datadict[key]
                self.values.append(value)
            except:
                self.values.append(None)

        self.string = self._stringit(self.fmt, self.values)

    def _generate_array(self):

        if type(self.threshold) in (int, float):
            self.imageRGB = Image.new('RGB', (self.w, self.h))
            self.draw  = ImageDraw.Draw(self.imageRGB)
        
            self.draw.text((0,0), self.string, font=self.font,
                           fill=(255, 255, 255, 255))  # R, G, B, alpha
        
            self.image8bit = self.imageRGB.convert("L")
            self.image1bit = self.image8bit.point(lambda x: 0 if x < self.threshold else 255, mode='1')
            self.image     = self.image1bit
            self.arrayRGB  = np.array(list(self.imageRGB.getdata())).reshape(self.h, self.w, 3)
            self.array8bit  = np.array(list(self.image8bit.getdata())).reshape(self.h, self.w)
            self.array1bit  = np.array(list(self.image1bit.getdata())).reshape(self.h, self.w)
        else:
            self.image = Image.new('1', (self.w, self.h)) 
            self.draw  = ImageDraw.Draw(self.image)
       
            self.draw.text((0,0), self.string, font=self.font, fill=255)

        self.ww, self.hh = self.image.size

        self.array = np.array(list(self.image.getdata())).reshape(self.hh, -1)

    def preview_me(self):

        plt.figure()
        plt.imshow(self.array, cmap='gray', interpolation='nearest')
        plt.show()

    def update_and_preview_me(self):

        self.update()
        self.preview_me()

class IMAGEFIELD(object): 

    devkind = "IMAGEFIELD"
    
    def __init__(self, name, wh=None, info=None):

        self.hints = """update()
preview_me()
update_and_preview_me()"""

        self.name            = name
        self.wh              = wh
        self.info            = info

        self.screens         = []

        try:
            self.w, self.h   = self.wh[:2]
            self.array = (np.zeros(self.wh, dtype=int))
        except:
            print "fieldshape problem"
            pass

        self.info            = info

    def __repr__(self):
        return ('{self.__class__.__name__}("{self.name}")'
                .format(self=self))

    def _embed(self, small_array, big_array, big_index):

        slices = [np.s_[i:i+j] for i,j in zip(big_index, small_array.shape)]
        big_array[slices] = small_array
        try:
            big_array[slices] = small_array
        except:
            msg = "field embed failed"
            self.box.logger.warning(msg)  

    def update(self):
        dev, key = self.info
        try:
            array = dev.datadict[key]
            if type(array) == np.ndarray and len(array.shape) == 2:
                self._embed(array, self.array, (0, 0))
        except:
            msg = "fail embed"
            self.box.logger.warning(msg)  

    def preview_me(self):

        plt.figure()
        plt.imshow(self.array, cmap='gray', interpolation='nearest')
        plt.show()

    def update_and_preview_me(self):

        self.update()
        self.preview_me()

class OLED_thread(Thread):

    devkind = "Thread"

    def __init__(self, screens, dwell_time=None,
                 stopper=None, keepgoing=None,
                 queobj=None, rotate180=False,
                 npages=8, ADDR=None, datamode=None):

        self.hints = """run()"""

        Thread.__init__(self)

        self.devkind        = "OLED"
        self.stopper        = stopper
        self.keepgoing      = keepgoing
        self.queobj         = queobj
        self.screens        = screens        
        self.rotate180      = rotate180        
        self.npages         = npages
        self.ADDR           = ADDR
        self.datamode       = datamode

        self.bus             = smbus.SMBus(1)  


        if not (type(dwell_time) in (int, float) and 0 < dwell_time < 60):
            dwell_time = 2
        self.dwell_time = dwell_time
        self.twos = 2**np.arange(8)[:, None]  # for _pages_to_bytes
        
    def _pages_to_bytes(self, Zpages):
        Zbinpages = []
        for page in Zpages:
            bytez = (page.astype(bool)*self.twos).sum(axis=0)
            Zbinpages.append(bytez.tolist())
        allbytez = sum(Zbinpages, [])
        return allbytez

    def run(self):
        self.nloop        = 0
        while True:
            with OLED_screens_lock:   # first LOCK!
                arrays    = [s.array.copy() for s in self.screens]
            if self.rotate180:
                arrays    = [np.flipud(np.fliplr(a)) for a in arrays]
            pageses       = [a.reshape(self.npages, 8, -1) for a in arrays]
            bytelists     = [self._pages_to_bytes(p) for p in pageses]

            if not len(arrays):
                time.sleep(self.dwell_time)  # wait until there's at least one array
            else:
                for i, bytelist in enumerate(bytelists):

                    self.bytelist = bytelist   # cache for debugging only

                    with OLED_smbus_lock:    # second LOCK!
                        for i in range(0, len(bytelist), 31):
                            chunk = bytelist[i:i+31]
                            self.bus.write_i2c_block_data(self.ADDR, self.datamode, chunk)
                        
                    time.sleep(self.dwell_time)

            if self.stopper.is_set():
                break
            self.keepgoing.wait()
            self.nloop += 1
        print 'OLED thread has ended!'
        return

class OLEDi2c(GPIO_DEVICE):
    
    devkind = "OLED"

    def __init__(self, box, name=None, rotate180=None):

        self.hints = """start_thread(dwell_time=None)
pause_thread(pause_time=None)
stop_thread()
YAMLsetup(fname)
new_screen(name)
add_screen(s)
Turn_On_Display()
Turn_Off_Display()
Set_Brightness(value)
initiate()
show_black()
show_white()
show_gray()
show_array(blockdata=True)   
get_screen(sc)
preview_all_screens()
save_all_screens()
preview_me()
save_me()
show_screen(showscreen)
preview_screen(previewscreen)
preview_me()
update_and_show_screen(showscreen) 
update_all_screens()
array_stats()
show_image(filename, resize_method, conversion_method, threshold=None)"""

        self.instance_things = locals()

        GPIO_DEVICE.__init__(self, box, name)

        msg = ("OLEDi2c '{}' __init__".format(self.name))
        self.box.logger.info(msg)

        with OLED_screens_lock:
            self.screens        = []

        if rotate180 == None:
            self.rotate180      = False
        else:
            self.rotate180      = rotate180

        # for generating bytes to transfer to display
        self.npages         = 8
        self.nsegs          = 128

        # for generating images using PIL
        self.nx             = self.nsegs
        self.ny             = 8 * self.npages
        self.nxy            = (self.nx, self.ny)

        self.array = (np.zeros(self.nx*self.ny,
                               dtype=int).reshape(self.ny, self.nx))

        # Constants
        self.ADDR           = 0x3C
        self.cmdmode        = 0x00
        self.datamode       = 0x40
        
        msg = (('{x.cname}("{x.name}") \n New instance named "{x.name}" ' +
                'rotate180={x.rotate180} nxy={x.nxy} ').format(x=self))
        self.box.logger.info(msg)

    def start_thread(self, dwell_time=None):
        self.queobj           = Queue.Queue()
        self.stopper          = Event()
        self.keepgoing        = Event()
        self.keepgoing.set()
        print  "keepgoing.isSet(): {}".format(self.keepgoing.isSet())
        self.thread = OLED_thread(self.screens, dwell_time=dwell_time,
                                  stopper=self.stopper,
                                  keepgoing=self.keepgoing, 
                                  queobj=self.queobj,
                                  rotate180=self.rotate180,
                                  npages=self.npages,
                                  ADDR=self.ADDR, datamode=self.datamode)


        time.sleep(0.2)
        self.thread.start()
        return self.thread

    def pause_thread(self, pause_time=None):
        if (type(pause_time) in (float, int)) and (pause_time > 0):
            self.thread.keepgoing.clear();
            print 'OLED pause set for {} sec!'.format(pause_time)
            self.paused_time         = time.time()
            time.sleep(pause_time)
            self.resume_thread()
        else:
            self.thread.keepgoing.clear(); print 'OLED sleeping until resumed later!'
    def resume_thread(self):
        self.thread.keepgoing.set(); print 'OLED resumed!'
        self.resumed_time        = time.time()
        try:
            pause_duration = self.resumed_time - self.paused_time
            print '   after having being paused for {} sec.'.format(pause_duration)
        except:
            pass
    def stop_thread(self):
        self.thread.stopper.set(); print 'OLED stopper has been set!'

    def YAMLsetup(self, fname):

        try:
            with open(fname, 'r') as infile:
                self.yamldict = yaml.load(infile)
            msg = (('{x.cname}("{x.name}") \n OLED YAML setup filename = {}').format(fname, x=self))
            self.box.logger.info(msg)
        except:
            msg = (('{x.cname}("{x.name}") \n OLED YAML setup FAILED filename = {}').format(fname, x=self))
            self.box.logger.warning(msg)

        if True:  # debugging   THE OTHER ONE DOESNT WORK

            for sname, sdef in self.yamldict.items():
                screen = self.new_screen(sname)
                if 'textfields' in sdef:
                    for fname, fdef in sdef['textfields'].items():
                        xy0  = fdef['xy0']
                        args = fdef['args']
                        infopairs = args['info']
                        newpairs  = []
                        for devname, key in infopairs:
                            devicx = self.box.get_device(devname)
                            newpairs.append([devicx, key])
                        args['info'] = newpairs
                            
                        screen.new_textfield(fname, xy0, **args)

                if 'imagefields' in sdef:
                    for fname, fdef in sdef['imagefields'].items():
        
                        xy0  = fdef['xy0']
                        args = fdef['args']
                        infopair = args['info']
                        devname, key = infopair
                        devicx = self.box.get_device(devname)
                        newpair = [devicx, key]
                        args['info'] = newpair
                            
                        screen.new_imagefield(fname, xy0, **args)

            msg = (('{x.cname}("{x.name}") \n OLED YAML setup successful').format(x=self))
            self.box.logger.info(msg)

        else:     # not debugging
            try:
                for sname, sdef in self.yamldict.items():
                    screen = self.new_screen(sname)
                    if 'textfields' in sdef:
                        for fname, fdef in sdef['textfields'].items():
                            xy0  = fdef['xy0']
                            args = fdef['args']
                            infopairs = args['info']
                            newpairs  = []
                            for devname, key in infopairs:
                                devicx = self.box.get_device(devname)
                                newpairs.append([devicx, key])
                            args['info'] = newpairs
                                
                            screen.new_textfield(fname, xy0, **args)

                    if 'imagefields' in sdef:
                        for fname, fdef in sdef['imagefields'].items():
                            xy0  = fdef['xy0']
                            args = fdef['args']
                            infopairs = args['info']
                            newpairs  = []
                            for devname, key in infopairs:
                                devicx = self.box.get_device(devname)
                                newpairs.append([devicx, key])
                            args['info'] = newpairs
                                
                            screen.new_imagefield(fname, xy0, **args)

                msg = (('{x.cname}("{x.name}") \n OLED YAML setup successful').format(x=self))
                self.box.logger.info(msg)

            except:
                msg = (('{x.cname}("{x.name}") \n OLED YAML setup could not be ' + 
                        'implemented successfully for some reason').format(x=self))
                self.box.logger.warning(msg)

    def new_screen(self, name):
        
        s = SCREEN(self.nxy, name)
        
        self.add_screen(s)
        
        return s

    def add_screen(self, s):
        
        with OLED_screens_lock:
            s.box = self.box
            self.screens.append(s)

        return s

    def _send_data_byte(self, one_byte, silent=True):
        successful = False
        try:
            self.bus.write_byte_data(self.ADDR, self.datamode, one_byte)
            successful = True
        except Exception, e:
            if not silent:
                msg = (('{x.cname}("{x.name}") \n problem sending byte "{}" via ' +
                        'pigpio I2Cbb : "{}"').format(one_byte, e, x=self))
                self.box.logger.warning(msg)
        return successful

    def _send_data_chunk(self, chunk, silent=True):
        successful = False
        if (type(chunk) is list) and (1 <= len(chunk) <= 31): 
            try:
                self.bus.write_i2c_block_data(self.ADDR, self.datamode, chunk)
                successful = True
            except Exception, e:
                if not silent:
                    msg = (('{x.cname}("{x.name}") \n problem sending byte "{}" via ' +
                            'pigpio I2Cbb : "{}"').format(chunk, e, x=self))
                    self.box.logger.warning(msg)
        else:
            msg = (('{x.cname}("{x.name}") \n problem sending chunk "{}" via ' +
                    'pigpio I2Cbb, length is bad or datatype wrong').format(one_byte, e, x=self))
            self.box.logger.warning(msg)
        return successful

    def _send_cmd(self, cmd, silent=False):
        successful = False
        try:
            self.bus.write_byte_data(self.ADDR, self.cmdmode, cmd)
            successful = True
        except Exception, e:
            if not silent:
                msg = (('{x.cname}("{x.name}") \n problem sending "{}" via ' +
                        'pigpio I2Cbb : "{}"').format(cmd, e, x=self))
                self.box.logger.warning(msg)
        return successful

    def _SET_DISPLAYALLON_RESUME(self, all_on=False): # self.SSD1306_DISPLAYALLON_RESUME = 0xA4
        """Set Display All On or Resume normal RAM display"""
        successful = False
        if type(all_on) is bool:
            cmd = 0b10100100 + all_on
            successful = self._send_cmd(cmd)
        return successful

    def _SET_SETVCOMDETECT(self, level=2): # self.SSD1306_SETVCOMDETECT = 0xDB
        """Set Vcomh Detect Level"""
        successful = False
        if level in range(8): # for now, but should be (0, 2, 3):
            cmd = 0b00000000 + level << 4
            successful_1 = self._send_cmd(0xDB)
            successful_2 = self._send_cmd(cmd)
            successful = successful_1 & successful_2
        return successful

    def _SET_PRECHARGE(self, phase_1=2, phase_2=2): # self.SSD1306_SETCONTRAST = 0x81
        """Set Constrast"""
        successful = False
        if (1 <= phase_1 <= 15) and (1 <= phase_2 <= 15):
            cmd = (phase_2 << 4) + phase_1
            successful_1 = self._send_cmd(0xD9)
            successful_2 = self._send_cmd(cmd)
            successful = successful_1 & successful_2
        return successful

    def _SET_NORMAL_INVERSE_DISPLAY(self, normal=True): #
        """Set Constrast"""
        successful = False
        if type(normal) is bool:
            cmd = 0b10100110 + (not normal)
            successful = self._send_cmd(cmd)
        return successful

    def _SET_CONTRAST(self, contrast=127): # self.SSD1306_SETCONTRAST = 0x81
        """Set Constrast"""
        successful = False
        if 0 <= contrast <= 255:
            successful_1 = self._send_cmd(0x81)
            successful_2 = self._send_cmd(contrast)
            successful = successful_1 & successful_2
        return successful

    def _SET_COM_PINS_CONFIG(self, normal=True, enable_leftright=False): # self.SSD1306_SETCOMPINS = 0xDA
        """Set COM Output Scan Direction"""
        successful = False
        if (type(normal) is bool) and (type(enable_leftright) is bool):
            cmd = 0b00000010 + (normal<<4) + (enable_leftright<<5)
            successful_1 = self._send_cmd(0xDA)
            successful_2 = self._send_cmd(cmd)
            successful = successful_1 & successful_2
        return successful

    def _SET_COM_SCAN_DEC(self, normal=True): # self.SSD1306_COMSCANDEC = 0xC8
        """Set COM Output Scan Direction"""
        successful = False
        if type(normal) is bool:
            cmd = 0b11000000 + (normal << 3)
            successful = self._send_cmd(cmd)
        return successful

    def _SET_SEGMENT_REMAP(self, setseg0=True): # self.SSD1306_SEGREMAP = 0xA0
        """Set Segment Remap to 0 or 127"""
        successful = False
        if type(setseg0) is bool:
            cmd = 0b10100000 + setseg0
            successful = self._send_cmd(cmd)
        return successful
            
    def _SET_MEMORY_ADDRESSING_MODE(self, mode=0): # self.SSD1306_MEMORYMODE = 0x20
        """Set Memory Addressing Mode"""
        successful = False
        if 0 <= mode <= 2:
            successful_1 = self._send_cmd(0x20)
            successful_2 = self._send_cmd(mode)
            successful = successful_1 & successful_2
        return successful
            
    def Turn_On_Display(self):
        successful = False
        successful_1 = self._ENABLE_CHARGEPUMP()
        successful_2 = self._SET_DISPLAYON()
        successful = successful_1 & successful_2
        return successful

    def Turn_Off_Display(self):
        successful = False
        successful_1 = self._DISABLE_CHARGEPUMP()
        successful_2 = self._SET_DISPLAYOFF()
        successful = successful_1 & successful_2
        return successful

    def Set_Brightness(self, value):
        if type(value) in (int, float):
            value = max(0, min(255, value))
            self._SET_CONTRAST(contrast=value)

    def _ENABLE_CHARGEPUMP(self): # self.SSD1306_CHARGEPUMP = 0x8D
        """Enable Charge Pump with Display ON"""
        successful = False
        cmd = 0b00010000 + 0b00000100
        successful_1 = self._send_cmd(0x8D)
        successful_2 = self._send_cmd(cmd)
        successful = successful_1 & successful_2
        return successful
            
    def _DISABLE_CHARGEPUMP(self): 
        """Didsable Charge Pump with Display ON"""
        successful = False
        cmd = 0b00010000 + 0b00000000
        successful_1 = self._send_cmd(0x8D)
        successful_2 = self._send_cmd(cmd)
        successful = successful_1 & successful_2
        return successful
            
    def _SET_START_LINE(self, line=0): # self.SSD1306_SETSTARTLINE = 0x40
        """Set Start Line for Display RAM"""
        successful = False
        if 0 <= line <= 63:
            cmd = 0b01000000 + line
            successful = self._send_cmd(cmd)
        return successful
            
    def _SET_DISPLAY_OFFSET(self, vertical_shift=0): # self.SSD1306_SETDISPLAYOFFSET = 0xD3
        """Set Vertical Offset"""
        successful = False
        if 0 <= vertical_shift <= 63:
            successful_1 = self._send_cmd(0xD3)
            successful_2 = self._send_cmd(vertical_shift)
        successful = successful_1 & successful_2
        return successful
            
    def _SET_MULTIPLEX_RATIO(self, mux_ratio=63): #  self.SSD1306_SETMULTIPLEX = 0xA8
        """Set Multiplex Ratio"""
        successful = False
        if 15 <= mux_ratio <= 63:
            successful_1 = self._send_cmd(0xA8)
            successful_2 = self._send_cmd(mux_ratio)
        successful = successful_1 & successful_2
        return successful
            
    def _SET_DISPLAY_FREQ_DIVRAT(self, frequency=8, divide_ratio=0): # self.SSD1306_SETDISPLAYCLOCKDIV = 0xD5
        """Set Display Clock Divide Ration and Oscillator Frequency"""
        successful = False
        if (0 <= divide_ratio <= 15) and (0 <= frequency <= 15):
            freqdivrat = frequency << 4 + divide_ratio
            successful_1 = self._send_cmd(0xD5)
            successful_2 = self._send_cmd(freqdivrat)
            successful = successful_1 & successful_2
        return successful
            
    def _SET_DISPLAYOFF(self): # self.SSD1306_DISPLAYOFF = 0xAE
        successful = False
        successful = self._send_cmd(0xAE)
        return successful
            
    def _SET_DISPLAYON(self): # self.SSD1306_DISPLAYON = 0xAF
        successful = False
        successful = self._send_cmd(0xAF)
        return successful

    def initiate(self):

        start_time = time.time()

        print " *** start_time: ", start_time
        
        try:

            self._SET_DISPLAYOFF()

            self._SET_DISPLAY_FREQ_DIVRAT(frequency=8, divide_ratio=0)

            self._SET_MULTIPLEX_RATIO(mux_ratio=63)

            self._SET_DISPLAY_OFFSET(vertical_shift=0)

            self._SET_START_LINE(line=0)

            self._ENABLE_CHARGEPUMP()

            self._SET_MEMORY_ADDRESSING_MODE(mode=0)

            self._SET_SEGMENT_REMAP(setseg0=True)

            self._SET_COM_SCAN_DEC(normal=True)

            self._SET_COM_PINS_CONFIG(normal=True, enable_leftright=False)

            self._SET_CONTRAST()  

            self._SET_PRECHARGE(phase_1=15, phase_2=1) 

            self._SET_SETVCOMDETECT() 

            self._SET_DISPLAYALLON_RESUME(all_on=False)

            self._SET_NORMAL_INVERSE_DISPLAY(normal=True)


        except Exception, e:
            print "type(e): ", type(e)
            print "str(e): ", str(e)

        init_time = time.time() - start_time

        print " *** init_time: ", init_time

        msg = (('{x.cname}("{x.name}") \n OLED initiated in {:0.6f} seconds').format(init_time, x=self))
        self.box.logger.info(msg)

    def show_black(self):
        self.array[:] = 0
        self.show_array()

    def show_white(self):
        self.array[:] = 1
        self.show_array()
        
    def show_gray(self):
        self.array[:] = 0
        self.array[0::2, 0::2] = 1
        self.array[1::2, 1::2] = 1
        self.show_array()
        
    def show_array(self, blockdata=True):   

        if self.rotate180:
            self.showarray = np.flipud(np.fliplr(self.array))
        else:
            self.showarray = self.array.copy()

        self.pages = self.showarray.reshape(self.npages, 8, -1) # [::-1]  # invert it bitwise or this way
        self.bytelist = self._pages_to_bytes(self.pages)

        self.goodies = []

        if blockdata:
            for i in range(0, len(self.bytelist), 31):
                chunk = self.bytelist[i:i+31]
                self._send_data_chunk(chunk, silent=False)  # set to True
                self.goodies.append(chunk)
        else:
            for byte in self.bytelist:
                self._send_data_byte(byte, silent=False)  # set to True
                self.goodies.append(byte)

    def show_array_old(self):

        if self.rotate180:
            self.showarray = np.flipud(np.fliplr(self.array))
        else:
            self.showarray = self.array.copy()

        self.pages = self.showarray.reshape(self.npages, 8, -1) # [::-1]  # invert it bitwise or this way
        self.bytelist = self._pages_to_bytes(self.pages)

        if 1 == 1:    # double-check write_i2c_block_data works also!
            # Write buffer data.
            for byte in self.bytelist:
                self.bus.write_byte_data(self.ADDR, self.datamode, byte)

        else:
            # Write buffer data.
            for i in range(0, len(self.bytelist), 31):
                bus.write_i2c_block_data(self.ADDR, self.datamode, self.bytelist[i:i+31])

    def _pages_to_bytes(self, Zpages):
        twos = 2**np.arange(8)[:, None]
        Zbinpages = []
        for page in Zpages:
            bytez = (page.astype(bool)*twos).sum(axis=0)
            Zbinpages.append(bytez.tolist())

        allbytez = sum(Zbinpages, [])
        return allbytez

    def get_screen(self, sc):

        scrn = None

        with OLED_screens_lock:
            if sc in self.screens:
                scrn = sc           # if it's there, return it
            else:
                try:
                    scrn = [s for s in self.screens if
                           s.name == sc][0] 
                except:
                    pass            

        return scrn

    def _frameit_nicely(self, array, val=255):

        s0, s1 = array.shape
        lr     = val * np.ones_like(array[:, :1])
        array  = np.hstack((lr, array, lr))
        tb     = val * np.ones_like(array[:1, :])
        array  = np.vstack((tb, array, tb))

        return array

    def _stack_arrays_nicely(self, arrays):
        
        N = len(arrays)
        # rowlentests = [0, 0] + [(i*i*2, i) for i in range(1, nperrowmax+1)] 
        if N <= 0:
            return 
        elif N <= 2:
            rowlen = 1
        elif N <= 8:
            rowlen = 2
        elif N <= 18:
            rowlen = 3
        elif N <= 32:
            rowlen = 4
        elif N <= 50:
            rowlen = 5
        else:
            rowlen = 6

        arrays = [self._frameit_nicely(a) for a in arrays]

        nrows = (N-1)/rowlen + 1
        rows  = []
        for irow in range(nrows):
            row     = arrays[rowlen*irow : rowlen*(irow+1)]
            deficit = rowlen - len(row)
            if deficit:
                row += deficit*[np.zeros_like(row[-1])]
            row  = np.hstack(row)
            rows.append(row)
        bigarray = np.vstack(rows)

        return bigarray

    def preview_all_screens(self):

        with OLED_screens_lock:
            arrays = [s.array.copy() for s in self.screens] # make working coppies during lock

        big_array = self._stack_arrays_nicely(arrays)

        if type(big_array) is np.ndarray:
            plt.figure()
            plt.imshow(big_array, cmap='gray', interpolation='nearest')
            plt.show()

    def save_all_screens(self):

        with OLED_screens_lock:
            arrays = [s.array.copy() for s in self.screens] # make working coppies during lock

        big_array = self._stack_arrays_nicely(arrays)

        if type(big_array) is np.ndarray:
            fname = 'OLED ' + self.name + ' all screens'
            plt.imsave(fname, big_array, cmap='gray') # , interpolation='nearest'
            
    def preview_me(self):
            
        plt.figure()
        plt.imshow(self.array, cmap='gray', interpolation='nearest')
        plt.show()
        
    def save_me(self):

        array = self.array
        if type(array) is np.ndarray:
            fname = 'OLED ' + self.name + ' current screen'
            plt.imsave(fname, array, cmap='gray', interpolation='nearest')

    def show_screen(self, showscreen):

        showscreen = self.get_screen(showscreen)

        if showscreen:
            self.array = showscreen.array.copy()
            self.show_array()
        else:
            msg = (('{x.cname}("{x.name}") \n OLED showscreen nor found').format(x=self))
            self.box.logger.warning(msg)
        return showscreen

    def preview_screen(self, previewscreen):

        previewscreen = self.get_screen(previewscreen)   # screen name search

        if previewscreen:
            screen.preview_me()
        
    def preview_me(self):

        plt.figure()
        plt.imshow(self.array, cmap='gray', interpolation='nearest')
        plt.show()
        
    def update_and_show_screen(self, showscreen): 

        showscreen = self.get_screen(showscreen)

        if showscreen:
            showscreen.update()
            self.show_screen(showscreen)

    def update_all_screens(self):

        for s in self.screens:
            s.update()

    def array_stats(self):
        try:
            amin, amax = self.array.min(), self.array.max()
            ashp, adtp = self.array.shape, self.array.dtype
        except:
            amin, amax = None, None
            ashp, adtp = None, None

        msg = (('{x.cname}("{x.name}") \n array_stats min={} max={} shape={} dtype={}')
               .format(amin, amax, ashp, adtp, x=self))
        self.box.logger.info(msg)

    def _embed(self, small_array, big_array, big_index):
        """Overwrites values in big_array starting at big_index with those in small_array"""
        slices = [np.s_[i:i+j] for i,j in zip(big_index, small_array.shape)]
        big_array[slices] = small_array
        try:
            big_array[slices] = small_array
        except:
            msg = "field array embed failed"
            self.box.logger.warning(msg)

    def show_image(self, filename, resize_method, conversion_method, threshold=None):

        self.img0   = Image.open(filename)
        w0, h0 = self.img0.size

        msg = (('{x.cname}("{x.name}") \n showimage opened size w0 h0={x.img0.size}')
               .format(x=self))
        self.box.logger.info(msg)

        if resize_method == 'stretch':

            new_size = (self.nx, self.ny)

            self.imgr = self.img0.resize(new_size)

            msg = (('{x.cname}("{x.name}") \n showimage new_size stretch={}').format(new_size, x=self))
            self.box.logger.info(msg)  

        elif resize_method == 'fit':

            wscale = float(self.nx)/float(w0)
            hscale = float(self.ny)/float(h0)

            if hscale <= wscale:
                shape = 'hscale <= wscale'
                new_size = (int(w0*hscale), self.ny)
            else:
                shape = 'hscale > wscale'
                new_size = (self.nx, int(h0*hscale))

            self.imgr = self.img0.resize(new_size)
            msg = (('{x.cname}("{x.name}") \n showimage {} resize size={}')
                   .format(shape, new_size, x=self))
            self.box.logger.info(msg)  

        else:

            self.imgr = None

            msg = (('{x.cname}("{x.name}") \n showimage new_size failed').format(x=self))
            self.box.logger.warning(msg)  

        if self.imgr:

            if conversion_method in ('default', 'dither'):

                self.imgc  = self.imgr.convert("1")

                msg = (('{x.cname}("{x.name}") \n showimage conversion_method "default"').format(x=self))

            elif conversion_method in ('threshold', ):

                self.imgr8 = self.imgr.convert("L")
                self.imgc  = self.imgr8.point(lambda x: 0 if x < threshold else 255, mode='1')

                msg = (('{x.cname}("{x.name}") \n showimage conversion_method "threshold"').format(x=self))

            self.box.logger.info(msg)  

        else:

            imgc = None

            msg = (('{x.cname}("{x.name}") \n showimage conversion failed').format(x=self))
            self.box.logger.warning(msg)  

        if self.imgc:
            
            wni, hni = self.imgc.size
            array    = np.array(list(self.imgc.getdata())).reshape(hni, wni)

            bigarray = np.zeros(self.ny*self.nx, dtype=int).reshape(self.ny, self.nx)

            sa0, sa1 = array.shape
            sb0, sb1 = bigarray.shape

            hoff = max(0, (sb0-sa0)/2-1)
            woff = max(0, (sb1-sa1)/2-1)
            
            self._embed(array, bigarray, (hoff, woff))
            
            arrayf = bigarray.astype(float)
            arrayf = arrayf / arrayf.max()

            self.array = bigarray.copy()

            self.show_array()

class G3bb(GPIO_DEVICE):

    devkind = "G3"  # this is used for LASS report lookup

    def __init__(self, box, name=None, DATA=None, collect_time=None):

        self.hints = """read()"""

        self.instance_things = locals()

        GPIO_DEVICE.__init__(self, box, name)

        self.box.logger.info("G3bb '{}' __init__".format(self.name))  

        if collect_time == None:
            collect_time      = 3.0

        self.collect_time     = collect_time
        self.DATA             = DATA
        self.baud             = 9600
        self.key              = '424d'

        msg = (('{x.cname}("{x.name}") \n New instance named "{x.name}" ' +
                'DATA={x.DATA} collect_time={x.collect_time} baud={x.baud} ' + 
                'key={x.key}').format(x=self))
        self.box.logger.info(msg)

        self.read()

        if self.ierr == 0:
            msg = (('{x.cname}("{x.name}") \n test read successful! ierr={x.ierr}').format(x=self))
        else:
            msg = (('{x.cname}("{x.name}") \n test read unsuccessful. ierr={x.ierr}').format(x=self))
        self.box.logger.info(msg)

    def read(self):

        self.ierr                              = -1  # initialize
        self.statistics['nreads']             +=  1 
        self.datadict                          =  dict()     # clear old data

        self.datadict['start_read_time'] = time.time()

        try:
            self.pi.bb_serial_read_close(self.DATA)
        except:
            pass

        # start to collect serial data
        self.pi.bb_serial_read_open(self.DATA, self.baud)

        time.sleep(self.collect_time) # allow data to collect

        # now read what's been collected
        self.datadict['read_time']             =  time.time()
        size, data     = self.pi.bb_serial_read(self.DATA)
        data_hexlified = hexlify(data)
        n_hexlified    = len(data_hexlified)

        self.datadict['size']            = size
        self.datadict['data_hexlified']  = data_hexlified
        self.datadict['n_hexlified']     = n_hexlified

        # find first occurence of key
        n0 = data_hexlified.find(self.key)
        n1 = n0 + 64   # make sure length is correct

        found_first  = n0 >= 0
        if found_first:
            found_both = data_hexlified[n1:n1+4] == self.key
        else:
            found_both = False

        self.datadict['n0']              = n0
        self.datadict['n1']              = n1
        self.datadict['found_first']     = found_first
        self.datadict['found_both']      = found_both

        if found_both:

            mydata = data_hexlified[n0:n1]

            thirty_two    = [int(mydata[2*i:2*(i+1)], 16) for i in range(32)]
            six           = thirty_two[4:10]
            three         = [256*six[i] + six[i+1] for i in (0, 2, 4)]

            checksum      = thirty_two[-1]
            checksumcalc  = sum(thirty_two[:30]) % 256

            checksum_okay = checksum == checksumcalc

            self.datadict['six']           = six
            self.datadict['three']         = three

            self.datadict['checksum']      = checksum
            self.datadict['checksumcalc']  = checksumcalc

            PM1, PM25, PM10                = three

            self.datadict['PM1']           = PM1
            self.datadict['PM25']          = PM25
            self.datadict['PM10']          = PM10
            
        else:

            checksum_okay = False

        if len(data_hexlified) == 0:
            self.ierr  = 8 
        else:
            self.ierr  = 0
            self.ierr += 1 * (not found_first)
            self.ierr += 2 * (not found_both)
            self.ierr += 4 * (not checksum_okay)

        # update errors
        self.datadict['ierr']                 = self.ierr
        self._last_twenty_ierr_increment()

        # update statistics
        self.statistics['last_twenty_ierr'] = self.last_twenty_ierr[:] # copy not pointer
        if self.ierr == 0:
            self.statistics['ngoodreads']    += 1 
        else: 
            self.statistics['nbadreads']     += 1

        # update lastdata
        key_keys = ('PM25', 'PM1', 'PM10')
        sdd = self.datadict
        if self.ierr == 0 and all([key in sdd for key in key_keys]):
            self.lastdata=''.join(['{}: {}, '.format(key, sdd[key]) for key in key_keys])


class GPSbb(GPIO_DEVICE):

    devkind = "GPS"   # "u-blox NEO 6, 7" 

    def __init__(self, box, name=None, DATA=None, collect_time=None):

        self.hints = """_read_chunk()
preview_GPS_satmap() 
preview_SNR_plot() 
draw_GPS_satmap()
read()"""

        self.instance_things = locals()

        GPIO_DEVICE.__init__(self, box, name)

        self.box.logger.info("GPSbb '{}' __init__".format(self.name))  

        if collect_time == None:
            collect_time           = 3.0

        self.collect_time          = collect_time
        self.DATA                  = DATA
        self.baud                  = 9600
        self.SNR_histobins         = (0, 22.5, 27.5, 32.5, 37.5,
                                      42.5, 47.5, 100)
        self.SNR_histocategories   = ('--', '25', '30', '35', '40',
                                      '45', '++')
       
        self.nsats_min        = 4   # should be higher than 4 but...
        
        self.sentence_dict = {'$GNGGA':'GPS_position', 
                              '$GNVTG':'GPS_speed', 
                              '$GPGSV':'GPS_satpos'}   # GPS sentences from uBlox
        
        self._make_map_array(empty=True)   # make an empty map for now
        self._make_SNR_barchart(empty=True)   # make an empty map for now
        
        msg = (('{x.cname}("{x.name}") \n New instance named "{x.name}" ' +
                'DATA={x.DATA} collect_time={x.collect_time} baud={x.baud} ' + 
                'sentence_dict={x.sentence_dict}').format(x=self))
        self.box.logger.info(msg)

        self.read()

        if self.ierr == 0:
            msg = (('{x.cname}("{x.name}") \n test read successful! ierr={x.ierr}').format(x=self))
        else:
            msg = (('{x.cname}("{x.name}") \n test read unsuccessful. ierr={x.ierr}').format(x=self))
        self.box.logger.info(msg)
           
    def _read_chunk(self):
                            
        try:
            self.status = self.pi.bb_serial_read_close(self.DATA)
        except:
            self.status = None
            pass

        # start to collect serial data
        self.status  = self.pi.bb_serial_read_open(self.DATA, self.baud)

        time.sleep(self.collect_time) # allow data to collect

        # now read what's been collected
        raw_size, data   = self.pi.bb_serial_read(self.DATA)
        lines        = ''.join([chr(x) for x in data]).splitlines()

        self.status  = self.pi.bb_serial_read_close(self.DATA)

        return lines, raw_size

    def _get_degs(self, string, hemisphere):

        A, B    = string.split('.')

        mins    = float(A[-2:]) + float('0.' + B)
        degs    = float(A[:-2])

        degrees = degs + mins/60.

        if hemisphere in ('S', 'W'):
            degrees *= -1.

        return degrees        

    def _check_uBlox_checksum(self, s):

        try:
            result = re.search('\$(.*)\*', s)
            chksm = 0
            for thing in result.group(1):
                chksm = chksm ^ ord(thing)
            checksum = hex(0x100 + chksm)[-2:]
            reported = s.split('*')[1].lower()
            isokay = checksum == reported
        except:
            isokay = None
        return isokay        

    def _test_and_group_the_lines(self):

        self.linedict     = dict()
        self.passlinedict = dict()
        self.linecounter  = dict()
        for line in self.lines:
            key = line[:6]
            if key in self.sentence_dict:    # keeps it tidy
                newkey = self.sentence_dict[key]
                try:
                    self.linedict[newkey].append(line)
                    self.linecounter[newkey]['nlines']          += 1
                    if self._check_uBlox_checksum(line):
                        self.passlinedict[newkey].append(line)
                        self.linecounter[newkey]['npasslines']  += 1
                except:
                    self.linedict[newkey]                        = [line]
                    self.linecounter[newkey]         = {'nlines':     0,
                                                        'npasslines': 0 }
                    self.linecounter[newkey]['nlines']          += 1
                    if self._check_uBlox_checksum(line):
                        self.passlinedict[newkey]                = [line]
                        self.linecounter[newkey]['npasslines']  += 1

    def _do_GPS_position(self, linez):

        for line in linez:

            try:
                (gpstimstr, lati, lathemi, longi, lonhemi, isfix, nsat, hordil, 
                 alti, altiunit, hgeoi, hgeoiunit) = line.split(',')[1:13]
                
                n_sats, fix = int(nsat), bool(isfix)

                if fix and n_sats >= self.nsats_min:

                    latitude  = self._get_degs(lati, lathemi)
                    longitude = self._get_degs(longi, lonhemi)
                    
                    self.datadict['latitude']        = round(latitude,  6)
                    self.datadict['longitude']       = round(longitude, 6)
                    self.datadict['fix']             = fix
                    self.datadict['n_sats']          = n_sats
                    self.datadict['horizontal_dilution']          = float(hordil)
                    self.datadict['gps_time_string'] = gpstimstr

                    altitude      = float(alti)
                    altitude_unit = altiunit
                    self.datadict['altitude']        = round(altitude, 1)
                    self.datadict['altitude_unit']   = altitude_unit

                    if altitude_unit.lower() == 'm':
                        self.datadict['altitude_meters'] = round(altitude, 1)
                    else:
                        self.datadict['altitude_meters'] = None

                    h_geoid       = float(hgeoi)
                    h_geoid_unit  = hgeoiunit
                    self.datadict['h_geoid']        = round(h_geoid, 1)
                    self.datadict['h_geoid_unit']   = h_geoid_unit

                    if h_geoid_unit.lower() == 'm':
                        self.datadict['h_geoid_meters'] = round(h_geoid, 1)
                    else:
                        self.datadict['h_geoid_meters'] = None

                    self.gps_good_line              = line
                    self.gps_success                = True

                    break
                
            except:

                # print "that position line didn't work"
                pass

    def _do_GPS_speed(self, linez):

        for line in self.passlinedict['GPS_speed']:
            try:
                spd, speed_units   = line.split(',')[7:9]
                speed              = float(spd)

                self.datadict['speed']         = speed   # if float conversion worked, its okay
                self.datadict['speed_units']   = speed_units

                if speed_units.lower() == 'k':
                    self.datadict['speed_kph']         = speed
                else:
                    self.datadict['speed_kph']         = None

                self.gps_good_speed_line               = line
                self.gps_speed_success                 = True

                break
        
            except:
                # print "that speed line didn't work"
                pass

    def _do_GPS_satmap(self, linez):

        self.satdict                           = dict()   # clear old satdict
        self.satuseddict                       = dict()   # clear old satdict

        splitlnz     = [line.split('*')[0].split(',') for line in linez]
        decosplitlnz = [(int(splitln[1]), int(splitln[2]), splitln) for
                        splitln in splitlnz]  # decorate
        
        L = []
        for a, b, splitln in decosplitlnz:
            if a == b:
                L = [(a, b, splitln)]
            elif (len(L) > 0  and b == L[-1][1]-1): 
                L.append((a, b, splitln))
                if b == 1:
                    break
            else:
                L = []

        L = [line[2] for line in L]  # undecorate
        
        for line in L:
            try:
                sats = [line[4*i:4*(i+1)] for i in range(1, 5)]
                for satID, el, az, SNR in sats:
                    try:

                        self.satdict[satID]   = {'elevation':float(el),
                                                 'azimuth':  float(az)}
                        
                        self.satuseddict[satID] = {'elevation':float(el),
                                                   'azimuth':  float(az),
                                                   'SNR_dB':    float(SNR)}
                    except:
                        pass

            except:
                pass

        self.datadict['satdict']                = self.satdict 
        self.datadict['satuseddict']            = self.satuseddict
        self.datadict['n_satsused']             = len(self.satuseddict)

        SNRs = []
        for satID, satdict in self.satuseddict.items():
            SNRs.append(satdict['SNR_dB'])
        numbers, other = np.histogram(SNRs, bins=self.SNR_histobins)
        self.datadict['binned_SNR_dB'] = numbers
        
        self._make_map_array()   # make a new map
        self._make_SNR_barchart()   # make a new barchart

    def _make_map_array(self, empty=False):

        halfpi, pi, twopi = [f*np.pi for f in (0.5, 1, 2)]
        degs, rads = 180./pi, pi/180.

        self.xc        = 32
        self.yc        = 32
        self.rmax      = 30
        self.bbox      = (self.xc-self.rmax, self.yc-self.rmax,
                     self.xc+self.rmax, self.yc+self.rmax)
        self.map_imagesize = (64, 64)
        self.xcross     = ((self.xc-2, self.yc  ), (self.xc+2, self.yc  ))
        self.ycross     = ((self.xc,   self.yc-2), (self.xc,   self.yc+2))

        self.mapimage    = Image.new('1', self.map_imagesize)
        self.mapdraw     = ImageDraw.Draw(self.mapimage)
        self.mapfont     = ImageFont.load_default()

        self.mapdraw.ellipse(self.bbox, fill=None, outline='white')

        self.mapdraw.line(self.xcross, fill='white')
        self.mapdraw.line(self.ycross, fill='white')

        if not empty:
            try:
                for key, sd in self.datadict['satuseddict'].items():
                    theta = rads * sd['azimuth']
                    r     = self.rmax * (90. - sd['elevation']) / 90.
                    xy    = [r*f(theta) + c for f, c in zip((np.cos, np.sin), (self.xc, self.yc))]
                    xy = [int(thing) for thing in xy]
                    xypts = [(xy[0]+a, xy[1]+b) for (a, b) in
                             (0, 0), (0, 1), (1, 0), (1, 1)]
                    for xypt in xypts:
                        self.mapdraw.point(xypt, fill='white')
            except:
                print "incomplete GPS map, missing GPS data."
           
        self.map_array = np.array(list(self.mapimage.getdata())).reshape(self.map_imagesize)
        self.datadict['map_array'] = self.map_array

    def _make_SNR_barchart(self, empty=False):

        self.SNR_barchart_imageshape    = (128, 64)
        self.SNR_barchart_arrayshape    = (64, 128)

        self.plotimage    = Image.new('1', self.SNR_barchart_imageshape)
        self.plotdraw     = ImageDraw.Draw(self.plotimage)
        self.plotfont     = ImageFont.load_default()

        # ncat            = 7
        width           = 10
        dx              = 15
        dy              = 5
        y_start         = 15
        x_start         = 15
        
        x_axis          = ((x_start,           63-y_start),
                           (127,               63-y_start))

        y_axis          = ((x_start,           63-y_start),
                           (x_start,           63-63     ))

        self.plotdraw.line(x_axis, fill='white')           # x-axis



        self.plotdraw.line(y_axis, fill='white')   # y-axis 

        for n in (0, 2, 4, 6, 8):           # y-ticks
            x0, x1    = x_start-2, x_start+2
            y         = y_start + n*dy
            y_tick    = ((x0,           63-y),
                         (x1,           63-y) )
            self.plotdraw.line(y_tick, fill='white')  

        for n in (0, 2, 4, 6, 8):   # y-axis labels
            x    = 2
            y    = y_start + 5 + n*dy
            self.plotdraw.text((x ,63-y), str(n), font=self.plotfont, fill=255)


        self.bars = []
        for i, cat in enumerate(self.SNR_histocategories): 
            x0   = x_start + i*dx
            # x-axis labels
            self.plotdraw.text((x0, 63-9), cat, font=self.plotfont, fill=255)
            # x-ticks
            y0, y1   = y_start-2, y_start+2
            x        = x0 + width/2
            x_tick   = ((x,    63-y0), (x,    63-y1))
            self.plotdraw.line(x_tick, fill='white')  

        if not empty:
            data            = self.datadict['binned_SNR_dB']
            for i, (n, cat) in enumerate(zip(data, self.SNR_histocategories)): 
                x0   = x_start + i*dx
                y0   = y_start
                x1   = x0 + width
                y1   = y0 + n*dy
                y1   = max(y0, min(y1, 63))
                bar = (x0, 63-y0, x1, 63-y1)
                self.bars.append(bar)
                self.plotdraw.rectangle(bar, fill=255, outline='white')

        self.plot_array = np.array(list(self.plotimage.getdata())).reshape(self.SNR_barchart_arrayshape)
        self.datadict['plot_array'] = self.plot_array

    def preview_GPS_satmap(self): 

        plt.figure()
        plt.imshow(self.map_array, cmap='gray', interpolation='nearest')
        plt.show()

    def preview_SNR_plot(self): 

        plt.figure()
        plt.imshow(self.plot_array, cmap='gray', interpolation='nearest')
        plt.show()

    def read(self):

        self.ierr                              = -1  

        self.gps_success                       = False
        self.npassGPSlines                     = 0 
        self.nGPSlines                         = 0  
        self.raw_size                          = 0  

        self.statistics['nreads']             +=  1 
        self.datadict                          =  dict()     # clear old data

        self.datadict['start_read_time'] = time.time()

        lines, raw_size                        = self._read_chunk()

        self.datadict['read_time']             = time.time()
        
        self.lines                = lines
        n_lines                   = len(lines)
        self.n_lines              = n_lines

        self.datadict['n_lines']  = n_lines
        self.raw_size             = raw_size
        self.datadict['raw_size'] = self.raw_size

        self.gps_good_line                     = None
        self.gps_success                       = False
        self.gps_good_speed_line               = None
        self.gps_speed_success                 = False

        self._test_and_group_the_lines()  

        self.datadict['linedict']              = self.linedict
        self.datadict['passlinedict']          = self.passlinedict
        try:
            self.nGPSlines     = self.linecounter['GPS_position']['nlines']
            self.npassGPSlines = self.linecounter['GPS_position']['npasslines']
            self.datadict['nGPSlines']             = self.nGPSlines
            self.datadict['npassGPSlines']         = self.npassGPSlines
        except:
            pass

        # first, do the GPS coordinate lines
        if 'GPS_position' in self.passlinedict:
            linez = reversed(self.passlinedict['GPS_position'])  
            self._do_GPS_position(linez)    # do it
        

        self.ierr  = 0
        self.ierr += 1 * (not self.gps_success     )
        self.ierr += 2 * (self.npassGPSlines  <   1)
        self.ierr += 4 * (self.nGPSlines      <   1)
        self.ierr += 8 * (self.raw_size       <= 50)  # arbitrary

        # update errors
        self.datadict['ierr']                 = self.ierr
        self._last_twenty_ierr_increment()

        # update statistics
        self.statistics['last_twenty_ierr'] = self.last_twenty_ierr[:] # copy not pointer
        if self.ierr == 0:
            self.statistics['ngoodreads']    += 1 
        else: 
            self.statistics['nbadreads']     += 1 

        # update lastdata
        key_keys = ('latitude', 'longitude', 'altitude', 'n_sats')
        sdd = self.datadict
        if self.ierr == 0 and all([key in sdd for key in key_keys]):
            thing = ('lat: {:0.3f} lon: {:0.3f} alt: {} n_sats: {}')
            # print "  HEY! HEY! ", thing
            # print "  HEY! HEY! ", [sdd[key] for key in key_keys]
            # print "  HEY! HEY! ", thing.format(*[sdd[key] for key in key_keys])
            self.lastdata = thing.format(*[sdd[key] for key in key_keys])

        # next, do the GPS speed lines
        if 'GPS_speed' in self.passlinedict:
            linez = reversed(self.passlinedict['GPS_speed'])
            self._do_GPS_speed(linez)

        # next, do the GPS satpos lines
        if 'GPS_satpos' in self.passlinedict:
            linez = reversed(self.passlinedict['GPS_satpos'])
            self._do_GPS_satmap(linez)


class DHT22bb(GPIO_DEVICE):

    devkind = "DHT22"  # this is used for LASS report lookup

    def __init__(self, box, name=None, DATA=None, POWER=None):

        self.hints = """read()"""

        self.instance_things = locals()

        GPIO_DEVICE.__init__(self, box, name)

        self.box.logger.info("DHT22bb '{}' __init__".format(self.name))

        self.DATA  = DATA
        self.POWER = POWER

        # Based on https://github.com/joan2937/pigpio/blob/master/EXAMPLES/Python/DHT22_AM2302_SENSOR/DHT22.py

        self.diffs            = []        # time differences (tics = microseconds)

        self.pi.write(self.POWER, 1)
        time.sleep(2)  # it takes about 2 seconds to activate, should sleep
        
        atexit.register(self.cancel)      # Cancel watchdog on exit

        self.high_tick       =  0
        self.bit             = 40
        self.tick_threshold  = 50        # microseconds (ticks) data is 1 or 0 

        self.pi.set_pull_up_down(self.DATA, pigpio.PUD_OFF)   

        self.pi.set_watchdog(self.DATA, 0)  # Kill any existing watchdogs on the pin

        # Set the callback on the pin now, start a watchdog for each read
        self.cb = self.pi.callback(self.DATA, pigpio.EITHER_EDGE, 
                                   self._cb2)  

        msg = (('{x.cname}("{x.name}") \n New instance named "{x.name}" ' +
                'DATA={x.DATA} POWER={x.POWER} tick_threshold={x.tick_threshold}').format(x=self))
        self.box.logger.info(msg)

        self.read()

        if self.ierr == 0:
            msg = (('{x.cname}("{x.name}") \n test read successful! ierr={x.ierr}').format(x=self))
        else:
            msg = (('{x.cname}("{x.name}") \n test read unsuccessful. ierr={x.ierr}').format(x=self))
        self.box.logger.info(msg)

    def _cb2(self, gpio, level, tick):

        diff = pigpio.tickDiff(self.high_tick, tick)

        if level == 0:

            self.diffs.append(diff)

            if self.bit >= 40: # Message complete.
                self.bit = 40

            elif self.bit == 39:  # 40th bit received.
                
                self.pi.set_watchdog(self.DATA, 0)  # deactivate watchdog

            self.bit += 1

        elif level == 1:
            
            self.high_tick = tick
            
            if diff > 250000:
                
                self.bit = -2

        else: # level == pigpio.TIMEOUT:
            
            self.pi.set_watchdog(self.DATA, 0)    # deactivate watchdog

    def read(self):

        self.ierr                              = -1  # initialize
        self.statistics['nreads']             +=  1 
        self.datadict                          =  dict()     # clear old data

        self.datadict['start_read_time'] = time.time()

        self.diffs    = []         # clear old data
        checksum_okay                          = False
        ndiffs_is_43                           = False
                    
        self.pi.write(self.DATA, pigpio.LOW)
        
        time.sleep(0.017) # 17 ms
        
        self.pi.set_mode(self.DATA, pigpio.INPUT)
        
        self.pi.set_watchdog(self.DATA, 200)

        time.sleep(0.2) 

        self.datadict['read_time']           = time.time()
        
        ndiffs                               = len(self.diffs) 

        ndiffs_is_43                         = ndiffs == 43

        self.datadict['diffs']               = self.diffs
        self.datadict['ndiffs']              = ndiffs
        self.datadict['ndiffs_is_43']        = ndiffs_is_43

        if ndiffs_is_43:
            
            five   = [self.diffs[3+8*i : 3+8*(i+1)] for i in range(5)]

            values = []
            for thing in five:

                value = 0

                for diff in thing:
                    value = (value<<1) + int(diff>self.tick_threshold)

                values.append(value)

            HH, HL, TH, TL, check_sum         = values

            four_sum = sum(values[:4])

            checksum_okay                     = four_sum == check_sum

            self.datadict['values']           = values

            if checksum_okay:

                humidity = HH + 0.1*HL

                if TH & 128:   # temperature is negative
                    sign = -1.
                    TH = TH & 127
                else:
                    sign = +1.

                temperature = sign * (TH + 0.1*TL)

                self.datadict['humidity']       = humidity
                self.datadict['temperature']    = temperature
                self.lastdata = 'last T: {:0.1f}C H: {:0.0f}%'.format(temperature, humidity)
                
            else:
                # print checksum_okay is False
                pass

        else:

            pass

        self.ierr  = 0
        self.ierr += 1 * (not checksum_okay)
        self.ierr += 2 * (not ndiffs_is_43)

        # update errors
        self.datadict['ierr']                     = self.ierr
        self._last_twenty_ierr_increment()

        # update statistics
        self.statistics['last_twenty_ierr'] = self.last_twenty_ierr[:] # copy not pointer
        if self.ierr == 0:
            self.statistics['ngoodreads']        += 1 
        else:
            self.statistics['nbadreads']         += 1 

        # update lastdata
        key_keys = ('temperature', 'humidity')
        if self.ierr == 0 and all([key in self.datadict for key in key_keys]):
            self.lastdata = ('last T: {:0.1f}C H: {:0.0f}%'.format(
                *[self.datadict[key] for key in key_keys]))

    # CANCEL THE WATCHDOG ON EXIT!
    def cancel(self):
        """Cancel the DHT22 sensor."""

        self.pi.set_watchdog(self.DATA, 0)

        if self.cb != None:
            self.cb.cancel()
            self.cb = None

class MCP3008bb(GPIO_DEVICE):
    
    devkind="MCP3008"
    
    def __init__(self, box, name=None, CSbar=None, MISO=None,
                 MOSI=None, SCLK=None, SPI_baud=None, Vref=None):
        
        self.hints = """_digitize_one_channel(i_channel, clear_datadict=False)
_digitize_alt(channels=None)
_digitize_raw(channels=None)
measure_one_voltage(i_channel, clear_datadict=False)
read()"""

        self.instance_things = locals()

        GPIO_DEVICE.__init__(self, box, name)

        self.box.logger.info("MCP3008bb '{}' __init__".format(self.name))  

        self.CSbar         = CSbar
        self.MISO          = MISO
        self.MOSI          = MOSI
        self.SCLK          = SCLK
        if SPI_baud == None:
            SPI_baud = 10000
        self.SPI_baud      = SPI_baud
        self.SPI_MODE      = 0      # this is important!
        self.Vref          = float(Vref)

        self.nbits         = 10   # MCP3008 is always 10 bits?? this may change

        try:
            self.pi.bb_spi_close(self.CSbar)
        except:
            pass

        self.pi.bb_spi_open(self.CSbar, self.MISO, self.MOSI, 
                       self.SCLK, self.SPI_baud, self.SPI_MODE)

        msg = (('{x.cname}("{x.name}") \n New instance named "{x.name}" ' +
                'CSbar={x.CSbar} MISO={x.MISO} MOSI={x.MOSI} SCLK={x.SCLK} ' +
                'SPI_baud={x.SPI_baud} Vref={x.Vref} SPI_MODE={x.SPI_MODE} ' + 
                'nbits={x.nbits}').format(x=self))
        self.box.logger.info(msg)

        self.read()
        if self.ierr == 0:
            msg = (('{x.cname}("{x.name}") \n test read successful! ierr={x.ierr}').format(x=self))
        else:
            msg = (('{x.cname}("{x.name}") \n test read unsuccessful. ierr={x.ierr}').format(x=self))

        self.box.logger.info(msg)
            
    def _digitize_one_channel(self, i_channel, clear_datadict=False):

        if clear_datadict:
            self.datadict = dict()

        three_bytes      = [1, (8+i_channel)<<4, 0]

        adc_convert_time = time.time()

        isaresponse, good, reasonable_limits = 0, 0, 0

        try:
            datacnt, rawdata = self.pi.bb_spi_xfer(self.CSbar, three_bytes) # http://abyz.me.uk/rpi/pigpio/python.html
            adc_value        = ((rawdata[1]<<8) | rawdata[2])    #  & 0x3FF leave off for debugging
        except:
            adc_value = None

        self.datadict['channel ' + str(i_channel) + 'adc_convert_time'] = adc_convert_time
        self.datadict['channel ' + str(i_channel) + ' adc_value'] = adc_value

        return i_channel, adc_value, datacnt, adc_convert_time

    def _digitize_quick(self, channels=None):
        if channels == None:
            channels = range(8)

        values = []
        for i_channel in channels:
            three_bytes      = [1, (8+i_channel)<<4, 0]  
            datacnt, rawdata = self.pi.bb_spi_xfer(self.CSbar, three_bytes) # http://abyz.me.uk/rpi/pigpio/python.html
            adc_value        = ((rawdata[1]<<8) | rawdata[2])    #  & 0x3FF leave off for debugging
            values.append(adc_value)

        return values

    def _digitize_alt(self, channels=None):
        if channels == None:
            channels = range(8)

        results = []
        for i_channel in channels:
            # from  read_adc() https://github.com/adafruit/Adafruit_Python_MCP3008/blob/master/Adafruit_MCP3008/MCP3008.py
            command = 0b11 << 6                  # Start bit, single channel read
            command |= (i_channel & 0x07) << 3  # Channel number (in 3 bits)
            # Note the bottom 3 bits of command are 0, this is to account for the
            # extra clock to do the conversion, and the low null bit returned at
            # the start of the response.
            three_bytes = [command, 0x0, 0x0]
            cnt, resp = self.pi.bb_spi_xfer(self.CSbar, three_bytes)
            result  = (resp[0] & 0x01) << 9
            result |= (resp[1] & 0xFF) << 1
            result |= (resp[2] & 0x80) >> 7
            results.append(result & 0x3FF)
            
        return results

    def _digitize_raw(self, channels=None):
        if channels == None:
            channels = range(8)

        raw = []
        for i_channel in channels:
            three_bytes      = [1, (8+i_channel)<<4, 0]  
            datacnt, rawdata = self.pi.bb_spi_xfer(self.CSbar, three_bytes) # http://abyz.me.uk/rpi/pigpio/python.html
            raw.append(rawdata)

        return raw

    def measure_one_voltage(self, i_channel, clear_datadict=False):

        self.last_read_is_good                 = False
        self.statistics['nreads']             += 1 

        adc_response = self._digitize_one_channel(i_channel, clear_datadict)
        ich, adc_value, datacnt, adc_convert_time = adc_response

        voltage = self.Vref * float(adc_value) / (2**self.nbits - 1.)

        self.datadict['channel ' + str(ich) + ' voltage'] = voltage

        self.last_read_is_good                 = True       

        if self.last_read_is_good:
            self.statistics['ngoodreads']     += 1
        else: 
            self.statistics['nbadreads']      += 1 

        self._last_twenty_ierr_increment()

        return voltage, i_channel, adc_value, datacnt, adc_convert_time

    def read(self):
        """reads all eight, clears all data first"""

        self.ierr                              = -1  # initialize
        self.statistics['nreads']             +=  1 
        self.datadict                          =  dict()     # always clears!

        self.datadict['start_read_time']       = time.time()

        results = []
        values  = []

        for i_channel in range(8):
            adc_response = self._digitize_one_channel(i_channel, clear_datadict=False)
            results.append(adc_response)

        nresponse, ngood, nreasonable = 0, 0, 0
        for response in results:
            nresponse += 1
            if len(adc_response) == 4:
                ngood += 1
                ich, adc_value, datacnt, adc_convert_time = adc_response
                if (adc_value >= 0) and (adc_value <= 1023):
                    nreasonable += 1
                    self.datadict['channel ' + str(ich) + 'adc_convert_time'] = adc_convert_time
                    self.datadict['channel ' + str(ich) + ' adc_value'] = adc_value

        self.ierr  = 0
        self.ierr += 1 * (nreasonable != 8)
        self.ierr += 2 * (nresponse   != 8)
        self.ierr += 4 * (ngood       != 8)

        # update errors
        self.datadict['ierr']                 = self.ierr
        self._last_twenty_ierr_increment()

        # update statistics
        self.statistics['last_twenty_ierr'] = self.last_twenty_ierr[:] # copy not pointer
        if self.ierr == 0:
            self.statistics['ngoodreads']    += 1 
        else: 
            self.statistics['nbadreads']     += 1 

        # update lastdata
        if self.ierr == 0:
            sdd = self.datadict
            key_keys = [key for key in sdd.keys() if 'counts' in key]
            results = []
            for key in key_keys:
                try:
                    results.append('{}: {}, '.format(key, sdd[key]))
                except:
                    pass
            results = ''.join(results)
            if len(results) > 1:
                self.lastdata = results

        return results

class MOS_Gas_Sensor(GPIO_DEVICE):
    
    devkind = "MOS_Gas_Sensor" # used for LASS lookup so it is updated with instance gasname
    
    def __init__(self, box, name=None, ADC=None, channel=None,
                 int_or_float=float, calibdatapairs=None, divider_posn=None, 
                 R_series=None, V_supply=None, units_name=None, gasname=None, 
                 xlog = False, ylog = False, xlimitsok=True):

        self.hints = """read()"""

        self.instance_things = locals()

        GPIO_DEVICE.__init__(self, box, name)

        self.box.logger.info("MOS_gas_sensor '{}' __init__".format(self.name))

        self.interpolate_ok       = False  
        self.interp_available     = True

        if ADC not in self.box.devices:
            ADC                 = self.box.get_device(ADC)

        if int_or_float not in (int, float):
            int_or_float = float

        self.ADC               = ADC

        self.channel           = channel
        self.int_or_float      = int_or_float
        self.rawcalibdatapairs = calibdatapairs
        self.divider_posn      = divider_posn
        self.R_series          = float(R_series)
        self.V_supply          = float(V_supply)
        self.units_name        = units_name
        self.gasname           = gasname
        if type(self.gasname) is str:
            self.devkind           = self.gasname # this is used for LASS report lookup
        self.xlog              = xlog
        self.ylog              = ylog
        self.xlimitsok         = xlimitsok

        try:
            self.sortedcalibdatapairs = sorted(calibdatapairs)
            xrawdata, yrawdata        = zip(*self.sortedcalibdatapairs)
            if len(xrawdata)         >= 2 and min(xrawdata) >= +1E-06:
                self.xcaldata         = xrawdata
                self.ycaldata         = yrawdata
                self.logxcaldata      = np.log10(xrawdata)  # use base 10 
                self.logycaldata      = np.log10(yrawdata)  # everywhere!
                self.xmin             = min(self.xcaldata)
                self.xmax             = max(self.xcaldata)
                msg = "rawcalibdatapairs accepted"
                self.box.logger.info(msg)
                self.interp_available = True
            else:
                msg = "rawcalibdatapairs unsutable, not enough pairs or x < 1E-06"
                self.box.logger.warning(msg) 
        except:
            msg = "unable to understand rawcalibdatapairs"            
            self.box.logger.warning(msg) 

        # initial test read
        self.read()
                   
        if self.ierr == 0:
            msg = (('{x.cname}("{x.name}") \n test read successful! ierr={x.ierr}').format(x=self))
        else:
            msg = (('{x.cname}("{x.name}") \n test read unsuccessful. ierr={x.ierr}').format(x=self))
        self.box.logger.info(msg)
    
    def _interpolate(self, x):    # use base 10 
        if self.interp_available:
            if (self.xlimitsok or
                ((not self.xlimitsok) and self.xmin <= x <= self.xmax)):
                # xx, xdd, ydd = x, self.xcaldata, self.xcaldata
                if self.xlog:
                    xdd  = self.logxcaldata
                    xx   = np.log10(x)  # use base 10 
                else:
                    xdd  = self.xcaldata
                    xx   = x
                    
                if self.ylog:
                    ydd  = self.logycaldata
                    y    = 10**(np.interp(xx, xdd, ydd))
                else:
                    y    = np.interp(xx, xdd, ydd)
                return y  # otherwise None is returned
            else:
                msg = ("xlimitsok={}, x={}, and xmin, xmax = {}, {}"
                       .format((self.xlimitsok), x, self.xmin, self.xmax))
                self.box.logger.warning(msg)
        else:
            msg = ("problem, interpolate_ok={}".format(self.interpolate_ok))
            self.box.logger.warning(msg)

    def _calculate_resistance(self, voltage):

        # print " gas _calculate_resistance voltage: ", voltage

        R_sensor    = None
        v_ratio     = voltage/self.ADC.Vref

        # print " gas _calculate_resistance v_ratio: ", v_ratio
        # print " self.divider_posn.lower()[:3]: ", self.divider_posn.lower()[:3]
        
        try:
            if self.divider_posn.lower()[:3] == 'top':
                # print " gas _calculate_resistance top! "
                R_sensor = (1.-v_ratio) * self.R_series / v_ratio
                # print " got R_sensor: ", R_sensor
            elif self.divider_posn.lower()[:3] == 'bot':
                # print " gas _calculate_resistance bot! "
                R_sensor = v_ratio * self.R_series  / (1.-v_ratio)
                # print " got R_sensor: ", R_sensor
        except:
            # print " except! "
            pass

        return R_sensor

    def read(self):

        adc_range_ok                           = False
        interpolate_ok                         = False
        self.ierr                              = -1  # initialize
        self.statistics['nreads']             +=  1 
        self.datadict                          =  dict()     # clear old data

        self.datadict['start_read_time']       = time.time()

        adc_response = self.ADC.measure_one_voltage(self.channel,
                                                    clear_datadict=False)  # make ADC measurement

        voltage, i_channel, adc_value, datacnt, adc_convert_time = adc_response

        # print " gas voltage, gas adc: ", voltage, adc_value

        # convert to a resistance
        Resistance = self._calculate_resistance(voltage)

        self.datadict['read_time']             = time.time()

        if ((0 <= adc_value <= 2**self.ADC.nbits - 1) and
            self.interp_available):

            adc_range_ok                      = True

            value           = self._interpolate(Resistance)

            if value:
                 interpolate_ok               = True

            if self.int_or_float == int:
                value = int(value)

        self.datadict['read_time']            = time.time()
        self.datadict['adc_value']            = adc_value
        self.datadict['voltage']              = voltage
        self.datadict['value']                = value
        self.datadict['units_name']           = self.units_name
        self.datadict['adc_convert_time']     = adc_convert_time
        self.datadict['adc_range_ok']         = adc_range_ok
        self.datadict['interpolate_ok']       = interpolate_ok
        self.datadict['channel']              = i_channel
        self.datadict[self.units_name]        = value   # needed for LASS

        self.ierr  = 0
        self.ierr += 1 * (not adc_range_ok)
        self.ierr += 2 * (not interpolate_ok)

        # update errors
        self.datadict['ierr']                 = self.ierr
        self._last_twenty_ierr_increment()

        # update statistics
        self.statistics['last_twenty_ierr'] = self.last_twenty_ierr[:] # copy not pointer
        if self.ierr == 0:
            self.statistics['ngoodreads']    += 1 
        else: 
            self.statistics['nbadreads']     += 1 

        # update lastdata
        key_keys = ('voltage', 'adc_value', 'value', 'units_name')
        sdd = self.datadict
        if self.ierr == 0 and all([key in sdd for key in key_keys]):
            thing = ('last voltage: {:0.3f} last adc_value: {} '+
                             'last value: {:0.3f} {}')
            self.lastdata = thing.format(*[sdd[key] for key in key_keys])

        return value, adc_value


class Analog_Device(GPIO_DEVICE):
    
    devkind = "Analog"
    
    def __init__(self, box, name=None, ADC=None, channel=None,
                 int_or_float=float, calibdatapairs=None, units_name=None, 
                 xlog = False, ylog = False, xlimitsok=True):

        self.hints = """read()"""

        self.instance_things = locals()

        GPIO_DEVICE.__init__(self, box, name)

        self.box.logger.info("MOS_gas_sensor '{}' __init__".format(self.name))

        self.nterpolate_ok       = False  
        self.interp_available     = True

        if ADC not in self.box.devices:
            ADC                 = self.box.get_device(ADC)

        if int_or_float not in (int, float):
            int_or_float = float

        self.ADC               = ADC

        self.channel           = channel
        self.int_or_float      = int_or_float
        self.rawcalibdatapairs = calibdatapairs
        self.units_name        = units_name
        self.xlog              = xlog
        self.ylog              = ylog
        self.xlimitsok         = xlimitsok

        try:
            self.sortedcalibdatapairs = sorted(calibdatapairs)
            xrawdata, yrawdata        = zip(*self.sortedcalibdatapairs)
            if len(xrawdata)         >= 2 and min(xrawdata) >= +1E-06:
                self.xcaldata         = xrawdata
                self.ycaldata         = yrawdata
                self.logxcaldata      = np.log10(xrawdata)  # use base 10 
                self.logycaldata      = np.log10(yrawdata)  # everywhere!
                self.xmin             = min(self.xcaldata)
                self.xmax             = max(self.xcaldata)
                msg = "rawcalibdatapairs accepted"
                self.box.logger.info(msg)
                self.interp_available = True
            else:
                msg = "rawcalibdatapairs unsutable, not enough pairs or x < 1E-06"
                self.box.logger.warning(msg) 
        except:
            msg = "unable to understand rawcalibdatapairs"            
            self.box.logger.warning(msg) 

        # initial test read
        self.read()
                   
        if self.ierr == 0:
            msg = (('{x.cname}("{x.name}") \n test read successful! ierr={x.ierr}').format(x=self))
        else:
            msg = (('{x.cname}("{x.name}") \n test read unsuccessful. ierr={x.ierr}').format(x=self))
        self.box.logger.info(msg)
   
    def read(self):

        adc_range_ok                           = False
        interpolate_ok                         = False
        self.ierr                              = -1  # initialize
        self.statistics['nreads']             +=  1 
        self.datadict                          =  dict()     # clear old data

        self.datadict['start_read_time']       = time.time()

        adc_response = self.ADC.measure_one_voltage(self.channel,
                                                    clear_datadict=False)  # make ADC measurement

        voltage, i_channel, adc_value, datacnt, adc_convert_time = adc_response

        self.datadict['read_time']             = time.time()

        if ((0 <= adc_value <= 2**self.ADC.nbits - 1) and
            self.interp_available):

            adc_range_ok                      = True

            value           = self._interpolate(voltage)

            if value:
                 interpolate_ok               = True

            if self.int_or_float == int:
                value = int(value)

        self.datadict['read_time']            = time.time()
        self.datadict['adc_value']            = adc_value
        self.datadict['voltage']              = voltage
        self.datadict['value']                = value
        self.datadict['units_name']           = self.units_name
        self.datadict['adc_convert_time']     = adc_convert_time
        self.datadict['adc_range_ok']         = adc_range_ok
        self.datadict['interpolate_ok']       = interpolate_ok
        self.datadict['channel']              = i_channel

        self.ierr  = 0
        self.ierr += 1 * (not adc_range_ok)
        self.ierr += 2 * (not interpolate_ok)

        # update errors
        self.datadict['ierr']                 = self.ierr
        self._last_twenty_ierr_increment()

        # update statistics
        self.statistics['last_twenty_ierr'] = self.last_twenty_ierr[:] # copy not pointer
        if self.ierr == 0:
            self.statistics['ngoodreads']    += 1 
        else: 
            self.statistics['nbadreads']     += 1 

        # update lastdata
        key_keys = ('voltage', 'adc_value')
        sdd = self.datadict
        if self.ierr == 0 and all([key in sdd for key in key_keys]):
            self.lastdata = ('last voltage: {:0.3f} last adc_value: {}%'.format(
                *[sdd[key] for key in key_keys]))

        return value, adc_value


class RTC_smbus(GPIO_DEVICE):

    devkind = "RTC" 
    
    def __init__(self, box, name=None, baud=None):

        self.hints = """read()
set_time(timeobject)"""

        self.instance_things = locals()

        GPIO_DEVICE.__init__(self, box, name)
    
        self.box.logger.info("RTC_smbus '{}' __init__".format(self.name))  

        if baud == None:
            baud = 9600

        self.devkind          = "DS3231"   

        self.ADDR             = 0x68
        self.start_register   = 0x00
        self.nbytes           = 7
        self.baud             = baud

    def read(self):

        self.datadict = dict()    # assures the old dict has been cleared.

        self.ierr                              = -1  
        self.statistics['nreads']             +=  1 
        self.datadict['read_time']             = time.time()

        # print 'self.nbytes: ', self.nbytes

        seven_bytes = self.bus.read_i2c_block_data(self.ADDR,
                                              self.start_register,
                                              self.nbytes)

        self.datadict['read_time']       = time.time()

        SEC, MIN, HR, DYWK, DYMO, MOCEN, YR = seven_bytes

        seconds      = int(SEC   & 0x0F) + 10 * int((SEC   >> 4) & 0x07)
        minutes      = int(MIN   & 0x0F) + 10 * int((MIN   >> 4) & 0x07)
        hours_is_12H =                          int((HR    >> 6) & 0x01)
        if hours_is_12H:
            hours_value  = int(HR    & 0x0F) + 10 * int((HR    >> 4) & 0x01)
            hours_is_PM  =                          int((HR    >> 5) & 0x01)
            hours_24     = hours_value + 12 * int(hours_is_PM)
        else:
            hours_24  = int(HR    & 0x0F) + 10 * int((HR    >> 4) & 0x03)

        dayofweek    = int(DYWK  & 0x07)
        dayofmonth   = int(DYMO  & 0x0F) + 10 * int((DYMO  >> 4) & 0x03)
        month        = int(MOCEN & 0x0F) + 10 * int((MOCEN >> 4) & 0x01)
        century      =                          int((MOCEN >> 7) & 0x01)
        year         = int(YR    & 0x0F) + 10 * int((YR    >> 4) & 0x0F)

        try:
            RTC_datetime = datetime.datetime(2000+year, month, dayofmonth,
                                             hours_24, minutes, seconds)
        except:
            RTC_datetime  = None

        self.datadict['seconds']       = seconds
        self.datadict['minutes']       = minutes
        self.datadict['hours_is_12H']  = hours_is_12H
        self.datadict['hours_24']      = hours_24
        self.datadict['dayofweek']     = dayofweek
        self.datadict['dayofmonth']    = dayofmonth
        self.datadict['month']         = month
        self.datadict['century']       = century
        self.datadict['year']          = year
        self.datadict['RTC_datetime']  = RTC_datetime
        self.datadict['seconds']       = seconds

        time_str = ':'.join([str(100+hours_24)[1:], str(100+minutes)[1:],
                             str(100+seconds)[1:]])

        date_str = ':'.join([str(year), str(100+month)[1:],
                             str(100+dayofmonth)[1:]])

        self.datadict['time_str']          = time_str
        self.datadict['date_str']          = date_str

        self.ier                           = 0
        # update errors
        self.datadict['ierr']              = self.ierr
        self._last_twenty_ierr_increment()

        # update statistics
        self.statistics['last_twenty_ierr'] = self.last_twenty_ierr[:] # copy not pointer
        if self.ierr == 0:
            self.statistics['ngoodreads']    += 1 
        else: 
            self.statistics['nbadreads']     += 1

        # update lastdata
        key_keys = ('RTC_datetime', )
        sdd = self.datadict
        if self.ierr == 0 and all([key in sdd for key in key_keys]):
            self.lastdata=''.join(['{}: {}, '.format(key, sdd[key]) for key in key_keys])
                            

    def set_time(self, timeobject):

        self.datadict = dict()     # clear old data

        ttup = tuple(timeobject.timetuple())[:7]

        year, month, date, hour, minute, second, weekday = ([int(x) for x in ttup])

        second    = min(59,   max(0,    second))
        minute    = min(59,   max(0,    minute))
        hour      = min(23,   max(0,    hour  ))
        date      = min(32,   max(1,    date   ))
        month     = min(12,   max(1,    month ))
        year      = min(2099, max(2001, year ))
        year      = year - 2000
        century   = 0
        weekday   = min(7,    max(1,    weekday))

        def bcd_it(n, mini=None, maxi=None):
            if mini == None:
                mini = 0
            if maxi == None:
                maxi = 59
            n = min(maxi,   max(mini,    n))
            units = n%10
            tens  = (n - units)/10
            return 16*tens + units

        bcd_second = bcd_it(second,  mini=0, maxi=59)
        bcd_minute = bcd_it(minute,  mini=0, maxi=59)
        bcd_hour   = bcd_it(hour,    mini=0, maxi=23)
        weekday    = bcd_it(weekday, mini=1, maxi=7)
        bcd_date   = bcd_it(date,    mini=1, maxi=31)
        bcd_month  = bcd_it(month,   mini=1, maxi=12)
        bcd_year   = bcd_it(year ,   mini=1, maxi=99)

        if century:
            bcd_year   = bcd_year | 0b10000000

        seven_bytes = [bcd_second, bcd_minute, bcd_hour, weekday,
                       bcd_date, bcd_month, bcd_year]

        seven_ints  = [int(x) for x in seven_bytes]


        self.bus.write_i2c_block_data(self.ADDR, self.start_register,
                                      seven_bytes)

        self.datadict['write_time']       = time.time()


class Dummy(GPIO_DEVICE):

    devkind       = 'Dummy'

    def __init__(self, box, name=None, dummydatadict=None):

        self.hints = """read() # doesn't read anything, it's a dummy!"""

        self.instance_things = locals()

        GPIO_DEVICE.__init__(self, box, name)

        if dummydatadict == None:
            dummydatadict = dict()
        self.dummydatadict                = dummydatadict

        self.lastdata = "No REAL data for this Dummy!" 

        msg = (('{x.cname}("{x.name}") \n New instance named "{x.name}" ' +
                'dummydatadict={x.dummydatadict}').format(x=self))
        self.box.logger.info(msg)

    def read(self): # doesn't read anything, it's a dummy!

        self.ierr                              = -1  # initialize
        self.statistics['nreads']             +=  1 
        self.datadict.update(self.dummydatadict)  # unlike others, does not clear old data

        self.datadict['start_read_time'] = time.time()

        self.datadict['read_time']             = time.time()

        self.ierr                              = 0   # dummys don't make mistakes

        # update errors
        self.datadict['ierr']                  = self.ierr 
        self._last_twenty_ierr_increment()

        # update statistics
        self.statistics['last_twenty_ierr'] = self.last_twenty_ierr[:] # copy not pointer
        if self.ierr == 0:
            self.statistics['ngoodreads']    += 1 
        else: 
            self.statistics['nbadreads']     += 1


class STEPPER_MOTOR(GPIO_DEVICE):

    devkind = "STEPPER"

    def __init__(self, box, name=None, GPIO_A=None, GPIO_B=None,
                 GPIO_C=None, GPIO_D=None,
                 max_steprate=None, ramp_rate=None):

        self.hints = """move(N=+1, method='fullstep', percent=100, ramped=False)"""

        self.instance_things = locals()

        GPIO_DEVICE.__init__(self, box, name)

        self.box.logger.info("STEPPER '{}' __init__".format(self.name)) 


        self.four_GPIOS = GPIO_A, GPIO_B, GPIO_C, GPIO_D

        if max_steprate  == None:
            max_steprate  = 10
        self.max_steprate = max_steprate

        if ramp_rate     == None:
            ramp_rate     = 0.25 * self.max_steprate
        self.ramp_rate    = ramp_rate
        
        self._configure(max_steprate=max_steprate, ramp_rate=ramp_rate)

        # self.Nmax         = int(self.max_steprate**2 / (2. * self.ramp_rate))
        # self.Nsteps       = np.arange(1, self.Nmax+1)
        # self.tstepsramp   = (2. * self.Nsteps * self.ramp_rate)**-0.5

        A, B, C, D        = self.four_GPIOS
        self.singles_R    = ((B, A), (C, B), (D, C), (A, D))
        self.singles_L    = ((C, D), (B, C), (A, B), (D, A))

        self.doubles_R    = ((C, A), (D, B), (A, C), (B, D))
        self.doubles_L    = ((B, D), (A, C), (D, B), (C, A))

        # turn on pull-up resistors
        for G in self.four_GPIOS:
            self.pi.set_pull_up_down(G, pigpio.PUD_UP)

    def _configure(self, max_steprate=None, ramp_rate=None):
        if max_steprate != None:
            self.max_steprate = max_steprate
        if ramp_rate != None:
            self.ramp_rate = ramp_rate

        self.Nmax         = int(self.max_steprate**2 / (2. * self.ramp_rate))
        self.Nsteps       = np.arange(1, self.Nmax+1)
        self.tstepsramp   = (2. * self.Nsteps * self.ramp_rate)**-0.5

    def _wavesteps(self, dts, GoRight):
        if GoRight:
            pairs = self.singles_R
        else:
            pairs = self.singles_L

        for dt in dts:
            for ON, OFF in pairs:
                self.pi.write(ON,  1)
                self.pi.write(OFF, 0)
                time.sleep(dt)

    def _fullsteps(self, dts, GoRight):
        if GoRight:
            pairs = self.singles_R
        else:
            pairs = self.singles_L

        for dt in dts:
            for ON, OFF in pairs:
                self.pi.write(ON,  1)
                self.pi.write(OFF, 0)
                time.sleep(dt)
        
    def _halfsteps(self, dts, GoRight):
        if GoRight:
            pairs = self.singles_R
        else:
            pairs = self.singles_L

        half_dts = [0.5 * dt for dt in dts]

        for half_dt in half_dts:
            for ON, OFF in pairs:
                self.pi.write(ON,  1)
                time.sleep(half_dt)
                self.pi.write(OFF, 0)
                time.sleep(half_dt)

    def move(self, N=+1, method='fullstep', percent=100, ramped=False):

        if method        == 'fullstep':
            self.mover    = self._fullsteps
        elif method      == 'halfstep':
            self.mover    = self._halfsteps
        elif method      == 'wavestep':
            self.mover    = self._wavesteps
        else:
            pass   # trap this later

        self.GoRight           = N >= 0
        self.N                 = abs(N)

        times      = np.array([])   
        nmax       = int(self.max_steprate**2 / (2. * self.ramp_rate))
        tmin       = 1./self.max_steprate

        self.nmax     = nmax
        self.tmin     = nmax

        nn         = np.arange(nmax+1)
        trs        = np.array([0] + [np.sqrt(2.*n/self.ramp_rate) for n in nn[1:]])
        trs        = trs[1:] - trs[:-1]
        self.trs   = trs
        
        if not ramped:
            
            times = tmin * np.ones(N)

        else:

            if N < 2*nmax:
                times  = np.hstack((times, trs[:N/2]))
                times  = np.hstack((times, trs[N/2::-1]))
            elif N >= 2*nmax:
                dN     = N - 2*nmax
                tflats = tmin * np.ones(dN)
                times  = np.hstack((times,       trs))
                times  = np.hstack((times,    tflats))
                times  = np.hstack((times, trs[::-1]))

        if 1 >= percent >= 100:
            self.times * (float(percent)/100.)  # rescale

        self.times    = times
        quartertimes  = 0.25 * times
        self.mover(quartertimes,    self.GoRight)


class HMC5883i2c(GPIO_DEVICE):

    devkind = "HMC5883"

    def __init__(self, box, name=None):

        self.hints = """read()
_read_three()
"""

        self.instance_things = locals()

        GPIO_DEVICE.__init__(self, box, name)

        self.box.logger.info("HMC5883 '{}' __init__".format(self.name)) 

        # REGISTERS:
        self.DEV_ADDR         = 0x1E
        self.DATA_REG         = 0x03

        # HMC5883 address, 0x1E(30)
        # Select configuration register A, 0x00(00)
        #		0x60(96)	Normal measurement configuration, Data output rate = 0.75 Hz

        self.bus.write_byte_data(self.DEV_ADDR, 0x00, 0x60)

        # HMC5883 address, 0x1E(30)
        # Select mode register, 0x02(02)
        #		0x00(00)	Continuous measurement mode

        self.bus.write_byte_data(self.DEV_ADDR, 0x02, 0x00)

        time.sleep(0.5)

        # HMC5883 address, 0x1E(30)
        # Read data back from 0x03(03), 6 bytes
        # 0x3 = X MSB, 0x4 = X LSB
        # 0x5 = Y MSB, 0x6 = Y LSB
        # 0x7 = Z MSB, 0x8 = Z LSB

    def _read_three(self):
        six_bytes = self.bus.read_i2c_block_data(self.DEV_ADDR, self.DATA_REG, 6)
        his, los  = six_bytes[0::2], six_bytes[1::2]
        values    = []
        for hi, lo in zip(his, los):
            value = (hi << 8) + lo
            if value > 32767:
                value -= 65536
            values.append(value)
        
        return values

    def read(self):

        self.ier                               = -1
        self.statistics['nreads']             +=  1 
        self.datadict                          = dict()
        t_read                                 = time.time()

        values      = self._read_three()
        are_three   = len(values) == 3
        allokay     = all([-2048 <= x <= 2048 for x in values])

        self.datadict['t_read']             = t_read
        self.datadict['values']             = values


        self.ierr  = 0
        self.ierr += 1 * (not are_three)
        self.ierr += 2 * (not allokay)

        # update errors
        self.datadict['ierr']                 = self.ierr
        self._last_twenty_ierr_increment()

        # update statistics
        self.statistics['last_twenty_ierr'] = self.last_twenty_ierr[:] # copy not pointer
        if self.ierr == 0:
            self.statistics['ngoodreads']    += 1 
        else: 
            self.statistics['nbadreads']     += 1

        # update lastdata
        key_keys = ('values', )
        sdd = self.datadict
        if self.ierr == 0 and all([key in sdd for key in key_keys]):
            self.lastdata=''.join(['{}: {}, '.format(key, sdd[key]) for key in key_keys])


class MPU6050i2c(GPIO_DEVICE):

    devkind = "MPU6050"

    def __init__(self, box, name=None, I2C_clockdiv=9, DLPF=2, divideby=100,
              recordgyros=True, recordaccs=True, recordtemp=False,
              n_gyro_scale=0, n_acc_scale=0, fifo_block_read=True,
              chunkbytes=24, AD0=1):

        self.hints = """digitize_all(sampletime=3, maxdata=1000, plot_it=True)
read()
get_FIFO_count()
_configure(I2C_clockdiv, DLPF, divideby, recordgyros,
                   recordaccs, recordtemp, n_gyro_scale, n_acc_scale,
                   fifo_block_read, chunkbytes, AD0)"""

        self.instance_things = locals()

        GPIO_DEVICE.__init__(self, box, name)

        self.box.logger.info("MPU6050 '{}' __init__".format(self.name)) 

        self.BWaccs        = (260, 184, 94, 44, 21, 10, 5, None)
        self.BWgyros       = (256, 188, 98, 42, 20, 10, 5, None)
        self.Fsamples      = (8000, 1000, 1000, 1000, 1000, 1000, 1000, 8000)
        self.I2C_clockdivs = range(16, 32)[7:] + range(16, 32)[:7]

        # REGISTERS:
        self.SMPLRT_DIV     = 0x19
        self.CONFIG         = 0x1a
        self.GYRO_CONFIG    = 0x1b
        self.ACCEL_CONFIG   = 0x1c
        self.FIFO_EN        = 0x23
        self.INT_ENABLE     = 0x38
        self.INT_STATUS     = 0x3a
        self.ACC_start      = 0x3b
        self.GYRO_start     = 0x43
        self.USER_CTRL      = 0x6a
        self.PWR_MGMT_1     = 0x6b
        self.PWR_MGMT_2     = 0x6c
        self.FIFO_R_W       = 0x74
        self.FIFO_COUNTH    = 0x72   # two bytes
        self.I2C_MST_CTRL   = 0x24
        # self.INT_PIN_CFG  = 0x37   # this appears to be important


        self._configure(I2C_clockdiv=I2C_clockdiv, DLPF=DLPF, divideby=divideby,
                        recordgyros=recordgyros, recordaccs=recordaccs,
                        recordtemp=recordtemp, n_gyro_scale=n_gyro_scale,
                        n_acc_scale=n_acc_scale, fifo_block_read=fifo_block_read,
                        chunkbytes=chunkbytes, AD0=AD0)


    def _cmdr(self, A):
        return self.bus.read_byte_data(self.ADDR, A)

    def _cmdw(self, A, B):
        return self.bus.write_byte_data(self.ADDR, A, B)

    def _configure(self, I2C_clockdiv, DLPF, divideby, recordgyros,
                   recordaccs, recordtemp, n_gyro_scale, n_acc_scale,
                   fifo_block_read, chunkbytes, AD0):

        if type(AD0) is str:
            AD0 = AD0.lower()[:2]

        if AD0 in (True, 1, 'hi'):
            self.ADDR = 0x69   # this is different than the RTC!
        else:
            self.ADDR = 0x68   # be careful, this is the same as the RTC!

        self.I2C_clockdiv      = I2C_clockdiv
        self.DLPF              = DLPF
        self.divideby          = divideby
        self.recordgyros       = recordgyros
        self.recordaccs        = recordaccs
        self.recordtemp        = recordtemp
        self.n_gyro_scale      = n_gyro_scale
        self.n_acc_scale       = n_acc_scale
        self.fifo_block_read   = fifo_block_read
        self.chunkbytes        = chunkbytes

        # duplicate self.gyro_scale        = 8 * self.n_gyro_scale
        # duplicate self.acc_scale         = 8 * self.n_acc_scale

        self.I2C_clock_freq    = 8E+06 / float(self.I2C_clockdivs[self.I2C_clockdiv])
        print "I2C_clock_freq (kHz): ", self.I2C_clock_freq
        
        self.BWacc             = self.BWaccs[self.DLPF]
        self.BWgyro            = self.BWgyros[self.DLPF]
        self.Fsample           = self.Fsamples[self.DLPF]

        self.samplerate        = float(self.Fsample)/float(self.divideby)
        self.nbytespersample   = (self.recordaccs*6  +
                                  self.recordgyros*6 +
                                  self.recordtemp*2   )

        self.nvaluespersample  = (self.recordaccs*3  +
                                  self.recordgyros*3 +
                                  self.recordtemp*1   )

        self.fifoen            = (self.recordaccs  * 0b00001000 +
                                  self.recordgyros * 0b01110000 +
                                  self.recordtemp  * 0b10000000 )

        self.gyro_scalebits    = 8 * self.n_gyro_scale
        self.acc_scalebits     = 8 * self.n_acc_scale

        self.gyro_scaling      = (250.0/2**15) * 2**self.n_gyro_scale
        self.acc_scaling       = (  2.0/2**15) * 2**self.n_gyro_scale

        # initialize
        self._cmdw(self.PWR_MGMT_1,   0x00) # wake up!
        time.sleep(0.05)
        self._cmdw(self.I2C_MST_CTRL, self.I2C_clockdiv) # set i2c 8 MHz clock didvie
        self._cmdw(self.PWR_MGMT_2,   0x00) # dunno
        self._cmdw(self.INT_ENABLE,   0x00) # bit 0 is for data ready interrupt
        self._cmdw(self.CONFIG,       0x06) # bits 0-2 DLPF  ### ????
        self._cmdw(self.SMPLRT_DIV,   self.divideby) # divide by 255
        self._cmdw(self.GYRO_CONFIG,  self.gyro_scalebits) 
        self._cmdw(self.ACCEL_CONFIG, self.acc_scalebits) 

        self._cmdw(self.FIFO_EN,      self.fifoen)   # should this come earlier?

        self._cmdw(self.USER_CTRL,    0x00) # disable everything
        self._cmdw(self.USER_CTRL,    0x04) # FIFO reset  (bit 2)
        # #self._cmdw(self.USER_CTRL,    0x40) # FIFO enable (bit 6)

        self._cmdw(self.USER_CTRL,    0x00) # disable everything
        self._cmdw(self.USER_CTRL,    0x04) # FIFO reset  (bit 2)

        # print "INT_STATUS: ", self._cmdr(self.INT_STATUS)

    def get_FIFO_count(self):
        hi, lo    = self.bus.read_i2c_block_data(self.ADDR, self.FIFO_COUNTH, 2)
        return (hi << 8) + lo

    def digitize(self, sampletime=3, maxdata=1000, plot_it=True):
        
        self.sampletime         = sampletime
        self.maxdata            = maxdata

        self.datadict           = dict()
        self.data               = []

        tstart  = time.time()

        ndata              = 0
        tnow               = time.time()
        values, fifocounts, tnows = [], [], []

        self._cmdw(self.USER_CTRL,    0x00) # disable everything
        self._cmdw(self.USER_CTRL,    0x04) # FIFO reset  (bit 2)

        self._cmdw(self.USER_CTRL,    0x40) # FIFO enable (bit 6)  START!

        while tnow - tstart <= self.sampletime and ndata <= self.maxdata:

            fifocount = self.get_FIFO_count()
            self.fifocount = fifocount
            
            while fifocount < self.chunkbytes:
                
                fifocount = self.get_FIFO_count()
                fifocounts.append(fifocount)
                # I wonder if we want this waiting loop so tight?

            chunk = self.bus.read_i2c_block_data(self.ADDR,
                                                 self.FIFO_R_W,
                                                 self.chunkbytes)

            vals = []
            for hi, lo in zip(chunk[0::2], chunk[1::2]):
                val = (hi << 8) + lo
                if val >= 0x8000:
                    val = -((65535 - val) + 1)
                vals.append(val)

            tnow = time.time()
            if len(vals) == self.chunkbytes/2:
                values.append(vals)
                fifocounts.append(fifocount)
                tnows.append(tnow)
            
            ndata += 1 # increment to avoid endless loop if somethign is wrong 

        self._cmdw(self.USER_CTRL,    0x00) # disable everything  STOP!
        self._cmdw(self.USER_CTRL,    0x04) # FIFO reset  (bit 2)  CLEAR

        values                        = sum(values, [])
        self.datadict['values']       = values
        self.datadict['fifocounts']   = fifocounts
        self.datadict['tnows']        = tnows
        nvalues                       = len(values)

        N = self.nvaluespersample

        if nvalues/N >= 1:

            things = [values[i::N] for i in range(N)]
            nmin   = min([len(x) for x in things])
            
            self.array = np.array([thing[:nmin] for thing in things])
            self.datadict['array']    = self.array

            if plot_it:
                self._parse_and_plot(self.array) # PLOT_IT!

    def _parse_and_plot(self, array):
        accs, temp, gyros = None, None, None
        arr               = array.copy()
        if self.recordaccs:
            accs    = arr[:3] * self.acc_scaling
            arr     = arr[3:]
        if self.recordtemp:
            temp    = 35.0 + (arr[:1] - (-521))/340.
            arr     = arr[1:]
        if self.recordgyros:
            gyros   = arr[:3] * self.gyro_scaling

        nplots = (1*self.recordaccs + 1*self.recordtemp +
                  1*self.recordgyros)
        
        i      = 0
        plt.figure()
        plt.rcParams['axes.formatter.useoffset'] = False
        things = (accs, temp, gyros)
        names  = ('acceleration (g)', 'temperature(C)', 'angular rate (deg/sec)')
        for thing, name in zip(things, names):
            if thing is not None:
                plt.subplot(nplots, 1, i+1)
                for thing2 in thing:
                    plt.plot(thing2)
                plt.title(name, fontsize=14)
                i += 1
        plt.subplots_adjust(hspace=0.35)
        plt.show()

    def read(self): 

        self.ierr                               = -1

        self.statistics['nreads']              +=  1 

        self.datadict           = dict()
        self.data               = []

        t_read                  = time.time()

        fourteen_bytes = self.bus.read_i2c_block_data(self.ADDR, 0x3b, 14)
        seven_pairs    = zip(fourteen_bytes[0::2], fourteen_bytes[1::2])

        values = []
        for hi, lo in seven_pairs:
            val = (hi << 8) + lo
            if val >= 0x8000:
                val = -((65535 - val) + 1)
            values.append(val)

        if len(values) == 7:
            self.ierr   = 0

        self.datadict['fourteen_bytes']    = fourteen_bytes
        self.datadict['seven_pairs']       = seven_pairs
        self.datadict['values']            = values
        self.datadict['t_read']            = t_read

        # convert T(C) = 35.0 + (t_read - (-521))/340.0 

        # update errors
        self.datadict['ierr']                 = self.ierr
        self._last_twenty_ierr_increment()

        # update statistics
        self.statistics['last_twenty_ierr'] = self.last_twenty_ierr[:] # copy not pointer
        if self.ierr == 0:
            self.statistics['ngoodreads']    += 1 
        else: 
            self.statistics['nbadreads']     += 1

        # update lastdata
        key_keys = ('values', )
        sdd = self.datadict
        if self.ierr == 0 and all([key in sdd for key in key_keys]):
            self.lastdata=''.join(['{}: {}, '.format(key, sdd[key]) for key in key_keys])
