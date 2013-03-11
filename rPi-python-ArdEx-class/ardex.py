# ardex03.py

# ArdEx 0.3
# Brian Lavery 2013
# rp_13@blavery.com

# ArdEx Python Class and demo code for Raspberry Pi


from smbus import SMBus
from time import *

import time

class Retry(object):
    default_exceptions = (Exception)
    #          http://peter-hoffmann.com/2010/retry-decorator-python.html
    #          https://gist.github.com/470611
    def __init__(self, tries, exceptions=None, delay=0):
        """
        Decorator for retrying function if exception occurs
       
        tries -- num tries
        exceptions -- exceptions to catch
        delay -- wait between retries
        """
        self.tries = tries
        if exceptions is None:
            exceptions = Retry.default_exceptions
        self.exceptions =  exceptions
        self.delay = delay

    def __call__(self, f):
        def fn(*args, **kwargs):
            exception = None
            for _ in range(self.tries):
                try:
                    return f(*args, **kwargs)
                except self.exceptions, e:
                    #print "Retry, exception: "+str(e)
                    #time.sleep(self.delay)
                    exception = e
                    #print e
            #if no success after tries, raise last exception
            #raise exception
            #print e
            return (0,0)   # Instead of exception, return a known code meaning comms fail (0,0)
        return fn

# Notes on retry:
# rPi to Ardex i2c operation gives occasional I/O error (#5).
# This seems to be related to Arduino being interrupt-disabled at the exact moment of trying the i2c transaction.
# A repeat attempt usually is successful.
# ArdEx sketch is written to avoid most disabling of interrupt,
#    but even small things like digitalWrite will disable interrupts a moment.
# Fortunately, all i2c transactions in this library seem safe to simply repeat.
# Using the Retry-Decorator wrapper class above, from Peter Hoffman
# In case of ArdEx -> rPi read (multi-part sequence), repeat the combined operation from start.

 
class ArdEx:
   
    def __init__(self, i2cbus, i2cAddr):
        self.bus = i2cbus
        self.addr = i2cAddr
   
    @Retry(8)   
    def i2cRead(self, cmd, pin, job, val8, bytes):
        msg = [pin, ord(job), val8]
        self.bus.write_block_data(self.addr, ord(cmd), msg)
        self.result = 0
        sleep(0.000015)
        for x in range(30) :                                 # loop until data is ready
            self.bytesready = self.bus.read_byte(self.addr)
            if self.bytesready > 0 :                         # 1 or 2 bytes now await
                break
            sleep(0.00005 + (x * 0.0001))                    # delaying the attempts
        if self.bytesready == 0 :
            return 0, 0xFFFF                                 # dummy result if cant fetch data
        if self.bytesready > 2 :
            return 0, self.bytesready                        # oops - only expect 1 or 2
        for x in range(self.bytesready) :             
            self.result = self.result << 8                   # push msb up into place
            self.result = self.result + self.bus.read_byte(self.addr)
        return self.bytesready, self.result                  # the byte or word wanted
        # return value pair: 
        # SUCCESS:  (bytesFetched, value16Returned)
        # FAIL:  (0, 65535=piTooSlow, or 0=I2cCommsFail)
 
       
    @Retry(8)
    def i2cWrite(self, cmd, pin, job, v1, v2):
        msg = [pin, ord(job), v1, v2]
        self.bus.write_block_data(self.addr, ord(cmd), msg)

##### Below are the 7 class functions you can call:
#     (Note for python class calls, you dont include the "self" parameter)

    # SETUP (write) commands
    def setPinAsOutput(self, dpin):
        self.i2cWrite("M", dpin, "O", 0, 0)
    def setPullup(self, dpin, lohi):   
        self.i2cWrite("D", dpin, "W", lohi & 1, 0)

       
    # WRITE commands
    def digitalWrite(self, dpin, lohi):
        self.i2cWrite("D", dpin, "W", lohi & 1, 0)
    def pwmWrite(self, dpin, value8):
        self.i2cWrite("P", dpin, "W", value8, 0)

       
    # READ commands
    # NOTE - in this 0.3 version the FAIL conditions are ignored.
    def digitalRead(self, dpin):
        return self.i2cRead("D", dpin, "R", 0, 1)[1]
    def analogRead(self, apin):
        return self.i2cRead("A", apin, "R", 0, 2)[1]
    def analogReadVcc(self):
        return self.i2cRead("A", 8, "R", 0, 2)[1]
   

# So your calls would be:

#  setPinAsOutput(dpin)
#  setPullup(dpin, lohi)
#  digitalWrite(dpin, lohi)
#  pwmWrite(dpin, value8)
#  = digitalRead(dpin)
#  = analogRead(apin)
#  = analogReadVcc()

         
##############################################
# All above is the ArdEx Class code.
# Leave that untouched (unless you know what you are doing),
# and write your ArdEx-controlling rPi routines in place of what is below.

##############################################

# Demo for accessing ArdEx device from Raspberry Pi, using the ArdEx class functions from above list:


bus = SMBus(0);     ### Create an i2c bus object
                    ###  NOTE - This applies to a first version rPi board.
                    ###  Use SMBus(1) for newer board with the two mounting holes.
                    ###  (They changed the i2c pinouts.)

       
ax1  = ArdEx(bus, 5)   ## Create our ArdEx object and give it an i2c address (Must match arduino sketch you compiled)
#ax2 = ArdEx(bus, 6)   ## A second ArdEx on a different i2c address?
 
ax1.setPinAsOutput(13)   # Your arduino sketch probably initialised pin13 as OUTPUT anyway??

print ax1.analogReadVcc()  # VCC in millivolts

for x in range (0, 8):
    ax1.digitalWrite(13,1)          # LED on
    print ax1.digitalRead(13)       # Read that same output back in
    sleep(0.2)
    ax1.digitalWrite(13,0)          # LED off
    print ax1.digitalRead(13)       # Read back LED - should be different now
    print ax1.analogRead(1)         # Anything on a1 to be measuring??
    sleep(1.5)
