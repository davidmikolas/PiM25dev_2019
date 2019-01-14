## PiM25

PiM25 is currently a Python module that lets you use your Raspberry Pi as a PM2.5 and air monitoring station for the LASS network. Unlike other air boxes, your PiM25 Box is extremely flexible and you can add other measuring routines and sensors to it flexibly, while still reporting standard data to LASS. 

Once you ahve connected your Pi to your devices as described in the PDF:

1. you can use the Walkthrough using a predefined setup script to learn how PiM25 works interactively. This is called an "iBox" or interactive PiM25 Box. 
2. Then you can try the predefined fully automatic looping script that will publish your data to LASS.


To use the module, down load it along with these few auxiliary files to a directory

There are three python scripts.

    PiM25.py        The main module with all methods, imports, and thread locks.
    PiM25_iBox.py   User script to set up your Box and run it interactively.
    PiM25_Box.py    User script to run your Box in the background

A few more files, depending on your setup.

    PiM25_Box.yaml           Text file with detailed setup for all OLED screens
    font(s).ttf              since PIL default is very small, use TTFs for larger and Chinese
    Hello.png (or .jpg, .bmp)   Greeting image for the box on OLED start-up

There are also some helpful files for reverence, such as

    PiM25_14_Generic_Device.py          Template for writng a new class for a new sensor or device

All methods are accessed through the BOX method. Just for example:

```
box  = BOX(*args)
dht  = box.new_DHT22bb('my dht', *args)
oled = box.new_OLEDi2c('my oled', *args)
```

## Walk-through, using the predefined setup script PiM25_iBox.py


Open a terminal and change to the directory where you have downloaded the files, then run an interactive python session from the terminal:

    $ cd PiM25
    $ ls
    $ python -i PiM25_iBox.py

Python logging is saved to a file *and also echoed to the screen* for your viewing pleasure. It will take about a minute to run.

some of the events you will see:

* pigpio and SMBus are started
* WiFi, internet ping, MQTT connect are checked
* Sensors, are instantiated and tested
* OLED is instantiated, screens are setup from a YAML
* OLED_thread for cycling through the screens is started
* LASS and DATALOG are instantiated

Once you have the python cursor `> `, you can experiment with the objects.

In order to see the python objects that represent your devices, you can see them in the script you have run, or you can discover them through box instance by using their unique string names that were given at instantiation.

For example, to get a reference to the MCP3008 ADC:

try typing adc (as it is called in the script)

    adc

or you can create a new reference for it

    _adc = box.get_device('my adc')

If you want to see all the devices:

    box.devices

or just the readable devices that have a `.read()` method:

    box.readables

To read one device

    adc.read()

To read all readable devices

    box.read_all_readables() 

or you can do it with a few lines yourself

    for device in box.readables:
        device.read()
 

Either way, it returns something like

```
[('DHT22bb("my dht") last T: 23.8C H: 95%', 0),
 ('G3bb("my g3") PM25: 14, PM1: 12, PM10: 15, ', 0),
 ('GPSbb("my gps")  ', 1),
 ('Dummy("my dummy gps") No REAL data for this Dummy!', 0),
 ('MCP3008bb("my adc")  ', 0),
 ('Analog_Device("my CO2") last voltage: 0.000 last adc_value: 0%', 0),
 ('Analog_Device("my CO") last voltage: 0.000 last adc_value: 0%', 0),
 ('Dummy("my gpsdum") No REAL data for this Dummy!', 0),
 ('Dummy("sys timedate") No REAL data for this Dummy!', 0)]
```

Each device has a name, and if there is any recent good data availabe it will show as well If anything looks bad you can read again. Often the DHT22 takes time to wake up when you first start your box, so read again using either

    dht.read()
or 
    box.get_device('my dht').read()

Type the name again to see if there is new data

    dht

or look at its data dictionary

    dht.datadict

in long form

    for key, value in dht.datadict.items():
        print key, value


Now look at all the screens that were defined in the yaml setup

    oled.screens

```
[SCREEN("GPSmap"),
 SCREEN("TandH"),
 SCREEN("particles"),
 SCREEN("gasses"),
 SCREEN("GPS")]
 ```

You can always get a reference to a screen using its unique string name

    s = oled.get_screen('TandH')
    s.preview_me() # manual preview on your computer

or just

    oled.get_screen('TandH').preview_me()
and then

    s.update()  # update the screens with newest data from sensors
    s.preview_me() # preview again, to see it change

You can be a "bad person" and add fake data to the dht:

    dht.datadict['temperature'] = 42
    s.preview_me()

it won't show until you update the screen using either

    oled.update_all_screens()

or

    s.update()

You can even preview all of the screens at the same time, and even save the image

    oled.preview_all_screens()  
    oled.save_all_screens()

In fact you may want to save these regularly so you can log into your pi and see the updated screens any time

*When you are finished* there are two ways to stop the box before quitting python

    box.stopall()

or 
    box.disconnect_MQTT()
    oled.stop_thread()

Also, to conserve the lifetime of the OLED display you can turn it on and off

    oled.Turn_Off_Display()
    oled.Turn_On_Display()

You can also dim the display to save battery power, and to extend the lifetime

    oled.Set_Brigtness(value=100)  # 0 to 255

Here is an example of how to cusomize and design your own OLED screens and TEXTFIELD objects:

```
from PiM25 import BOX

# create your BOX object, all other objects are instantiated from it.
box       = BOX('my box')

# make a dummy device to hold some sample data
dummydict = {'x':42, 'y':777}
dummy     = box.new_Dummy('my dummy', dummydict)

# add an OLED object
oled      = box.new_OLEDi2c('my oled', rotate180=True)

oled.initiate() 
oled.Turn_On_Display()

oled.show_image('hello.png', resize_method='fit',
                conversion_method='threshold', threshold=60)

# add a SCREEN object to oled
screen    = oled.new_screen ('my screen')

# add a FIELD object to screen
field_x   = screen.new_textfield('my field x', xy0=(20,10), wh=(100,20),
                                fmt='x={}',fontdef='default',
                                fontsize=None, threshold=None,
                                info=[['my dummy', 'x']])

dummy.read()        # fills the datadict

field_x.update()
field_x.preview_me()

# add a second FIELD object to screen
field_y   = screen.new_textfield('my field y', xy0=(20,30), wh=(100,20),
                                fmt='y={} wow!',fontdef='default',
                                fontsize=None, threshold=None,
                                info=[['my dummy', 'y']])

screen.update()
screen.preview_me()
```

When you are done, you can then type

    box.stopall()
    
After everything is shut down and the OLED thread has stopped, you can exit Python:

    exit()  # to exit Python.

**You have to manually shut down your Pi before disconnecting the power.**


## Run a predefined AUTOMATIC script with looping

$ nohup python PiM25_Box.py &

The `&` is very important, it will allow the script to continue to run if you close the terminal. (If you do this a lot, you can run as a daemon, see for example https://stackoverflow.com/a/2975645 and https://pypi.org/project/python-daemon/)

It will reply with something like this, with the process ID. 

    [1] 2429

You can then check the process using

    $ ps ax | grep 2429

If the process is running you will see two lines, the second one can be ignored.

     2429 pts/0    Sl     0:01 python PiM25_Box_06.py
     2565 pts/0    R+     0:00 grep --color=auto 2429

The output, including the screen-echoed logging messages or any errors or exceptions are directed to the file

     nohup.out

The loop is set short so the process will end in a few minutes. You can replace it with an infinite loop if you would like the process to run permanently.


