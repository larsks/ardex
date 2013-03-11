/*
  QuadEncoder.cpp -  Arduino-328 wheel encoder library
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





#include "wiring_private.h"
#include "Analog.h"
#include "QuadEncoder.h"


int QuadEncoder::_encoderStates[4][4] = {  
        {  0, -1,  1,  0 } ,         // matrix to decide count up or down for 2-pin encoder
        {  1,  0,  0, -1 } ,         // for 1-pin, [0][1] changed to +1 (up only)
        { -1,  0,  0,  1 } ,  
        {  0,  1, -1,  0 }    };


QuadEncoder::QuadEncoder(void)
{
    _state=0;
    _prevState=0;
    _debounceMicros = 0L;
    _analog = NULL;
    _counter = 0L;
    _active = false;


}
    

void QuadEncoder::begin(uint8_t pin0, int isQuad)
{
     pinMode (pin0+14, INPUT) ;   // force to input. Use a separate digitalWrite(HI) if pullup needed.
     _isquad = isQuad;
     if (_isquad)
        pinMode (pin0+16, INPUT) ;   // its quadrature pair
     else
     {
        _encoderStates[0][1] = 1;     // set matrix to work as count-up-only for 1-pin
        _encoderStates[1][0] = 1;     // (for 1-pin only 2x2 upper left portion is used)
        // And so, if one is COUNTER, other can't be QUAD! encoder matrix is static/common
     }
     _active = true;
     _counter = 0;
     _pin = pin0;
     return;
}

void QuadEncoder::run(void)
{

        if (! _active)
            return;
        if (micros() < _debounceMicros)
            return;
        _debounceMicros = micros()+((debounce&0xFF)*10);
        if(_isquad)
        {
            if (_analog != NULL)
                _state = ((_analog->logicalRead(_pin)) << 1) | (_analog->logicalRead(_pin+2));
            else
                _state = (digitalRead(_pin+14) << 1) | digitalRead(_pin+16);
        }
        else
            if (_analog != NULL)
                _state = (_analog->logicalRead(_pin));
            else
                _state = (digitalRead(_pin+14));
        _counter += _encoderStates[_prevState][_state];
        _prevState = _state;
    }

void QuadEncoder::enableAnalog(Analog* ourAnalog)
{
    _analog = ourAnalog;
}
  

uint16_t QuadEncoder::readCounter()
{
    return (uint16_t) _counter;
    
}
    


