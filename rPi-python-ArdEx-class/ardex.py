from smbus import SMBus
from time import *

import time

## RETRY CLASS / DECORATOR:
    
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
            #if no success after tries, raise last exception
            raise exception
        return fn

# Notes on retry:
# rPi to Ardex i2c operation gives occasional I/O error (#5). 
# This seems to be related to Arduino being interrupt-disabled at the exact moment of trying the i2c transaction.
# A repeat attempt usually is successful.
# ArdEx sketch is written to avoid most disabling of interrupt, 
#    but even small things like digitalWrite disable interrupts a moment.
# Fortunately, all i2c transactions in this library seem SAFE TO SIMPLY REPEAT. 
# Using the Retry-Decorator wrapper class above, from Peter Hoffman
# In case of ArdEx -> rPi read (multi-part sequence), repeat the COMBINED operation from start.
 
 
 ## ARDEX CLASS:
     
class ArdEx:
    def __init__(self, i2cbus, i2cAddr):
        self.bus = i2cbus
        self.addr = i2cAddr
    
    @Retry(4)    
    def i2cRead(self, cmd, pin, val8, bytes):
        self.bus.write_word_data(self.addr, cmd, val8*256 +pin)
        self.result = 0;
        for x in range(20) :                                 # loop until data is ready
            self.bytesready = self.bus.read_byte(self.addr)
            if self.bytesready > 0 :                         # 1 or 2 bytes now await
                break
            else :
                return 0, 0xFFFF                             # dummy result if cant fetch data
        if self.bytesready > 2 :
            return 0, self.bytesready                        # oops - only expect 1 or 2
        for x in range(self.bytesready) :              
            self.result = self.result << 8                   # push msb up into place
            self.result = self.result + self.bus.read_byte(self.addr) 
        return self.bytesready, self.result                  # the byte or word wanted
        # return values:  
        #     1. success? 0=no, 1,2=bytes RX
        #     2. value (byte or word) returned
        
    @Retry(3)
    def i2cWrite(self,cmd, pin, val8):
        self.bus.write_word_data(self.addr, cmd, val8*256 +pin) 

    # SETUP (write) commands
    def setPinAsOutput(self, dpin):
        self.i2cWrite(0x4D, dpin, 0x4F)
    def setPinAsServo(self, dpin):
        self.i2cWrite(0x4D, dpin, 0x53)
    def setPinAsQuadEncoder(self, apin):
        self.i2cWrite(0x4D, apin, 0x51)
    def setPinAsCounter(self, apin):
        self.i2cWrite(0x4D, apin, 0x45)
    def setPullup(self, dpin, lohi):    ## == dig write
        self.i2cWrite(0x44, dpin, lohi and 1)
    def analogWriteSetpoint(self, apin, value):
        self.i2cWrite(0x41, apin, value)
    def encoderWriteDebounce(self, apin, value):
        self.i2cWrite(0x45, 0xD0+(apin and 1), value)
    def encoderEnableAnalog(self, apin):
        self.i2cWrite(0x45, 0xA0+(apin and 1), 0)
        
    # WRITE commands
    def digitalWrite(self, dpin, lohi):
        self.i2cWrite(0x44, dpin, lohi and 1)
    def digitalWrite4(self, dpin1, lohi1, lohi2, lohi3, lohi4):
        self.i2cWrite(0x44, dpin1, 0xE0+(lohi1>0) + (lohi2>0)*2 + (lohi3>0)*4 + (lohi4>0)*8)
    def servoWrite(self, dpin, value):
        self.i2cWrite(0x53, dpin, value)
    def pwmWrite(self, dpin, value):
        self.i2cWrite(0x50, dpin, value)
    def pulseGen(self, dpin, freq, duration):
        self.i2cWrite(0x47, dpin, freq+(duration*16))
        # freq: 0-7={1, 5, 10, 100, 400, 1000, 10000, 38000}
        # duration: 0-7={stop, 5, 100, 500, 1000, 4000, 10000, inf}
    def flagWrite(self, reg, value):
        self.i2cWrite(0x46, reg and 15, value)
        
    # READ commands
    def digitalRead(self, dpin):
        return self.i2cRead(0x44, dpin, 0xFF, 1)
    def digitalRead16(self):
        return self.i2cRead(0x44, 0xFF, 0, 2)
    def analogRead(self, apin):
        return self.i2cRead(0x41, apin, 0xEF, 2)
    def analogReadVcc(self):
        return self.i2cRead(0x41, 8, 0xEF, 2)
    def analogLogicalRead(self, apin):
        return self.i2cRead(0x41, apin, 0xFF, 1)
    def encoderRead(self, apin):
        return self.i2cRead(0x45, apin, 0, 2)
    def flagRead(self, reg):
        return self.i2cRead(0x46, reg, 0, 1)


         
##############################################

## EXAMPLE OF USE
## This example has long sequence of i2c transactions. 
## (Stress test for seeing i2c correctly "retries")

bus = SMBus(0); // this for rPi early board. Use SMBus(1) for version 2/512 boards
        
ax1 = ArdEx(bus, 5)    // create an ardex object "ax1"

 
ax1.setPinAsOutput(13)

for x in range (0, 2500):
    ax1.digitalRead(13)
    ax1.digitalWrite(13,0)
    ax1.digitalRead(13)
    ax1.digitalWrite(13,1)
    
    #ax1.digitalWrite4(12,0,1,0,0)
