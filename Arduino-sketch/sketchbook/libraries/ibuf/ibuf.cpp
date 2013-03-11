//   ibuf.cpp

#include <Arduino.h>
#include <Wire.h>
#include "ibuf.h"



I2Cbuffer::I2Cbuffer(void)
{
    _I2C_In = 0;
    _I2C_Out = 0;
}

unsigned char I2Cbuffer::put(void)
{
    // This is to be called inside i2c receive event routine
    // RETURN is JOB code. Caller (rcv event) must set readyflag off immediately if a data read is being prepared!!
   
    // throw away data if buffer full. 
    // Speed of i2c repeats vs loop() speed: quite likely buffer never get more than 1 entry!
    // (unless loop() gets too busy on some delayed job.)
   
    unsigned char job = 0;
    int ichr;
    if ((((I2C_BUFSIZE+_I2C_Out)-_I2C_In) & (I2C_BUFSIZE-1)) != 1)
    {
        for (int k=0; k<=5; k++)
            {
            ichr = Wire.read();
            _I2C_Buffer[k][_I2C_In] = ichr;   // doesn't matter if = FFFF
            if (k==3) // ie JOB code
                job = ichr;
            }
        if (++_I2C_In == I2C_BUFSIZE)
             _I2C_In = 0;      // rollover
    }
    while (Wire.available()>0)
        Wire.read();    // flush away anything left? Should be nothing.
    return job;
}

bool I2Cbuffer::get(unsigned char * cmd_, unsigned char * pin_, unsigned char * job_, unsigned char * val1_, unsigned char * val2_)
{
    // This is called later to recall the received 6 bytes
   
    //    exit if buffer empty
    if (_I2C_In == _I2C_Out)
        return false;
    // extract from buffer.   
    *cmd_ = _I2C_Buffer[0][_I2C_Out];
    *pin_ = _I2C_Buffer[2][_I2C_Out];
    *job_ = _I2C_Buffer[3][_I2C_Out];
    *val1_ = _I2C_Buffer[4][_I2C_Out];
    *val2_ = _I2C_Buffer[5][_I2C_Out];
    if (++_I2C_Out == I2C_BUFSIZE)
          _I2C_Out = 0;
    return true;

   
}
