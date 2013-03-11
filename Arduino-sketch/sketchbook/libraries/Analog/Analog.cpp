/*
  Analog.cpp -  Arduino-328 background analog capture library
  Copyright (c) 2012 Brian Lavery.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <wiring_private.h>
#include "Analog.h"


Analog::Analog(void)
{
    Vcc  = 0L;
    for (int k=0; k<=7; k++)
        {
        _value[k] = 0xFFFF;
        _setPoint[k] = 511;
        }

}
   

void Analog::begin(void)
{
    _currentPin = 0;                            // set to start at a0
    _startNextCapture();                        // start up round robin reading
    return;
}

void Analog::begin(uint8_t pin)
{
    _measureVCC();                           
    _captureForced(pin);                        // read initial incoming i2c volts.
    _currentPin = 0;                            // reset to start at a0
    _startNextCapture();                        // start up round robin reading
    return;
}


void Analog::_captureForced(uint8_t pin)
{
    // ANY analog, even 4/5
    // blocks - waits for result. Only intended for initial job.
    // stores into the usual place in array.
    uint16_t valu;
    if (pin > 7)
        return;
    ADMUX = (DEFAULT << 6) | pin;              // tell adc to capture a4
    delay(2);                                  // settle aref?
    sbi(ADCSRA, ADSC);                         // start the conversion
    while ((valu = _checkCapture()) == 0xFFFF) // do we have a finished result?
        ;                                      // wait
    _value[pin] = valu;                         // store a4 (i2c pin)
   
}

void Analog::_startNextCapture(void)
{
    uint8_t pin = this->_currentPin;
    while (true)
    {   
        pin = (pin+1) & 7;         // next try pin: 0-7
        if (pin==4 || pin==5)      // skip i2c pins
            continue;
        break;
        // need a test for pin = output pin     nyi
    }

    ADMUX = (DEFAULT << 6) | pin;  // tell adc what pin to capture

    sbi(ADCSRA, ADSC);             // start the conversion
    this->_currentPin = pin;       // store away what pin is being processed
    return;                        // we will check later to see if conversion is done
}

uint16_t Analog::_checkCapture(void)
{
    uint8_t low, high;

    if (bit_is_set(ADCSRA, ADSC))  // Are we done with capture process yet?
        return 0xFFFF;             // still busy: return FFFF = "not available"
    low  = ADCL;                   // YES, fin. We have to read ADCL first
    high = ADCH;
    return (high << 8) | low;
}




void Analog::_measureVCC(void)
{
    // Read 1.1V reference against AVcc. Inverting that, means vcc becomes known!
    // set the reference to Vcc and the measurement to the internal 1.1V reference
    // ref    http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
    // Use this function before all other reads.
    long result;

    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);   // aref=vcc read=1.1 internal
    delay(2);                                                 // Wait for Aref to settle
    ADCSRA |= _BV(ADSC);                                      // Start conversion
    while ((result = _checkCapture()) == 0xFFFF)
           ;                                                  // wait until done
    ADMUX = (DEFAULT << 6);                                   // Aref to default for every future read
    Vcc = 1125300L / result;                                  // Calculate Vcc (in mV);
    return;                                                   // 1125300 = 1.1*1023*1000
}

int Analog::logicalRead(uint8_t pin)
{  // logical compare with setpoint
    if (pin>7)
        return 0;
    return (_value[pin] >= _setPoint[pin]);
}
   
int Analog::read(uint8_t pin)       // analog value from stored array
{
    if (pin>7)
        return 0xFFFF;
    return _value[pin];
}
 
void Analog::setPoint(uint8_t pin, int setpoint)
{
    if (pin <= 7)
        _setPoint[pin] = setpoint & 0x03FF;
}
   
void Analog::run(void)
{
    // call repeatedly in loop()
    uint16_t valu = _checkCapture();            // do we have a finished result?
    if (valu != 0xFFFF)                         // FFFF = not yet
        {
            _value[_currentPin] = valu;          // store the result
            _startNextCapture();                // and promptly start the next!
        }
}

