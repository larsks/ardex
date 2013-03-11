/*
  QuadEncoder.h
  2012 Brian Lavery.  All right reserved.

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

#ifndef QENC_h
#define QUEC_h

#include <inttypes.h>


///////////////////////////////////////////////////////
// CLASS QuadEncoder: a 2-pin quadrature up/down counter/encoder or a 1-pin up-only-counter 
// Useful for tracking wheel distance using encoder wheel
// Designed to use a0 and/or a1 for 1-pin, or a0/a2 and/or a1/a3 for 2-pin
// Can detect by digitalRead() or by analog-above-a-setpoint (Relying on Analog class)
// Intended for maximum 2 QuadEncoder objects (over a0-a3) 
///////////////////////////////////////////////////////       



class QuadEncoder
{

  public:
    QuadEncoder(void);
    void begin(uint8_t pin0, int isQuad);     // 
    uint16_t readCounter();
    void zeroCounter();
    int debounce;              // use 0-255. debounce dead-time = 10 uSec units
    void run(void);              // call regularly in loop()
    void enableAnalog(Analog * ourAnalog);

  private:
    unsigned long _counter;
    int _isquad;
    int _pin;
    Analog * _analog;
    int _active;
    int _state;
    int _prevState;
    long _debounceMicros;
    static int _encoderStates[4][4];


};

#endif

