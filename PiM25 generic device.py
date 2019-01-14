

# This is a GENERIC template script that you can modify for your own device
# It is meant to work with PiM25.py 

class GENERIC(GPIO_DEVICE):

    devkind = "GENERIC"  # this is used for LASS report lookup

    def __init__(self, box, name=None, DATA=None, setupinfo=None):

        self.hints = """read()"""

        self.instance_things = locals()

        GPIO_DEVICE.__init__(self, box, name)

        self.box.logger.info("GENERIC '{}' __init__".format(self.name))  

        if setupinfo == None:
            setupinfo      = 'your default'

        self.setupinfo            = setupinfo
        self.DATA                 = DATA
        self.ADDR                 = 0x42    # for example

        msg = (('{x.cname}("{x.name}") \n New instance named "{x.name}" ' +
                'DATA={x.DATA} setupinfo={x.setupinfo}').format(x=self))
        self.box.logger.info(msg)

        self.read()  # test it now

        if self.ierr == 0:
            msg = (('{x.cname}("{x.name}") \n test read successful! ierr={x.ierr}').format(x=self))
            self.box.logger.info(msg)
        else:
            msg = (('{x.cname}("{x.name}") \n test read unsuccessful. ierr={x.ierr}').format(x=self))
            self.box.logger.warning(msg)

    def init(self):

        # send your wake-up and configuration to your device

        pass


    def _yourdeviceread(self):

        # put your device read sequence here!

        rawdata = [1, 2, 3]  # for example

        return rawdata


    def _process(self, raw_data):

        values

        values = [2*x+3 for x in raw_data]  # for example

        return values

        
    def read(self):

        self.ierr                              = -1  # initialize
        self.statistics['nreads']             +=  1 
        self.datadict                          =  dict()     # clear old data



        raw_data = self._yourdeviceread()

        self.datadict['read_time']       = time.time()
        self.datadict['raw_data']        = raw_data

        values = _process(raw_data)   # convert your data to useful units

        self.datadict['values']          = values

        has_three_values                 = len(values) == 3    # for example
        
        all_non_negative                 = all([x >=  0 for x in values])
        all_below_100                    = all([x < 100 for x in values])

        if has_three_values:
            x, y, z = values

            self.datadict['x']               = x
            self.datadict['y']               = y
            self.datadict['z']               = z

        self.ierr  = 0
        self.ierr += 1 * (not has_three_values)
        self.ierr += 2 * (not all_non_negative)
        self.ierr += 4 * (not all_below_100)


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
