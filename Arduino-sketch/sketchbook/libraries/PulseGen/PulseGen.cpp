/* by BLAVERY
 * Totally untested yet    <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<,,
 * driven from run() in loop()
 * Uses millisecond counter to pulse a pin, either continuous or 1-shot.
*/

#include "PulseGen.h"
#include "Arduino.h"


PulseGen::PulseGen(void)
{

}

void PulseGen::start(int p, int millihi, int millilo, bool start_lo, bool cont)
{
    pin = p;
    mseclo = millilo;
    msechi=millihi;
    currentcount=0;
    continuous=cont;
    currentstate = start_lo ? LOW: HIGH;
    digitalWrite (pin, currentstate);
}


void PulseGen::run(void)
{
    if(pin==0)           // pin==0 means stop strober
        return;
    long m = millis();
    if (m == prevmillis)
        return;    // hang around only once per millisecond
    prevmillis = m; 
    if (++currentcount >= (currentstate ? mseclo : msechi))    // time to toggle?
    {
        currentstate ^= 1;
        digitalWrite (pin, currentstate);
        currentcount = 0;
        if (! continuous)
            pin=0;   // terminates after 1 pulse
    }
    
        
        
}
