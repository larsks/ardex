/* by BLAVERY
 * Totally untested yet
 * Relies on IRremote library
 * Buffers incoming codes
 * Requires regular run() from loop()
*/


#include "IRin.h"


IRin::IRin(IRrecv * ir_recv)
{
    irrec = ir_recv;  // keep a pointer to the IRremote receive object
    return;
}



void IRin::begin(void)
{
     irrec->enableIRIn(); // Start the receiver
     begun = true;
}



void IRin::run(void)
{
    if (! begun)
        return;
    int chr;
    if (irrec->decode(&results)) 
    {
        chr = results.value;
        // is there buffer room?
        if ((((IR_BUFSIZE+IR_Out)-IR_In) & (IR_BUFSIZE-1)) != 1)
        {
            IR_Buffer[IR_In] = chr; 
            if (++IR_In == IR_BUFSIZE)
                IR_In = 0;      // rollover
        }
        irrec->resume(); // Await the next value
    }
    
}
    

int IRin::available(void)
{
    return (IR_In != IR_Out);
}
    
int IRin::getValue(void)
{
    //    exit if buffer empty
    if (! available())
        return 0xFFFF;
    // extract from buffer.    
    int tmp = IR_Buffer[IR_Out];
    if (++IR_Out == IR_BUFSIZE)
          IR_Out = 0;
    return tmp;
}
