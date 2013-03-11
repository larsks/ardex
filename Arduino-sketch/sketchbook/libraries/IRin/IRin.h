/*
  IRrecv.h
  2012 Brian Lavery.  


*/

#ifndef IRIN_h
#define IRIN_h

#include <IRremote.h>
#define IR_PIN             11
#define IR_BUFSIZE         16          // must be power of 2
   
   
class IRin 
{

  public:
    IRin(IRrecv * ir_recv);
    void begin(void);
    void run(void);
    int available(void);
    int getValue(void);



  private:
    IRrecv * irrec;                // pointer to receive object (already constructed)
    decode_results results;        // decoder object
    int   IR_Buffer[IR_BUFSIZE];   // the buffer of stored ir chars waiting for processing
    char  IR_In = 0;               // index into buffer array
    char  IR_Out = 0;              // index into buffer array
    char begun = false;





};

#endif

