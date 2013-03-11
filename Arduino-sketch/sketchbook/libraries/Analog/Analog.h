/*
  Analog.h
  2012 Brian Lavery. 


*/

#ifndef ANALOG_h
#define ANALOG_h

#include <inttypes.h>

///////////////////////////////////////////////////////
// CLASS Analog:  A background ANALOG reading engine
// No blocking for 100 uSec (as standard call analogRead() does).
// Maintains a live array of all analog values, at about 600uSec update cycle.
// A single Analog object handles all analog inputs, exc a4 & a5 which are reserved for i2c
///////////////////////////////////////////////////////       




class Analog
{

  public:
    Analog(void);
    void begin(void);            // starts first regular capture
    void begin(uint8_t pin);     // Reads VCC, and a4 (or other pin) then starts first regular capture.
    void run(void);              // call regularly in loop()
    long Vcc;                    // in milliVolts. Can be read after begin().
    int read(uint8_t pin);       // analog value from stored array
    int logicalRead(uint8_t pin); // logical compare with setpoint
    void setPoint(uint8_t pin, int setpoint);

  private:
    uint8_t _currentPin;
    void _measureVCC(void);
    void _captureForced(uint8_t pin);
    void _startNextCapture(void); 
    uint16_t _checkCapture(void);
    uint16_t _value[8];
    uint16_t _setPoint[8];

};

#endif

