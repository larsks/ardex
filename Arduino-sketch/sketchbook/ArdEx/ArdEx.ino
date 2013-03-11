/*
 *  ardex03.ino     BY BLAVERY
 *  "ARDEX 0.3"
 *  Using a 3.3v Meduino Nano (Arduino clone) to do rPi I/O hardware interface expansion.
 *
 *  Allow Pi (i2c master) talking to arduino (i2c slave) to control 18 Arduino IO pins.
 *  (Very modest smarts in the arduino. Real executive work still to do on the Pi!)
 *  Available:  d2-d17 digital i/o  (d14-d17 aka a0-a3)
 *              a0-a3 and a6-a7 as analog in.
 *  Optionally, 6 pins can do PWM,  d13 is commonly user-driven LED,
 *  Unavailable: Uart on d0-d1 considered tied to usb, and a4-a5 are devoted to i2c.
 *
 *
 *
 *   Cmd=device-type  NN=pin/addr  job   param1   param2

 * Write Commands:
 * M  NN  mode        Pin# NN (2-17) Set pin mode. Mode 'O'=output
 * D  NN  W on.off    Pin# NN (2-17) Digital output control. even=off/LOW odd=on/HIGH   eg '0' & '1'
 * P  NN  W val8      Write PWM value (pins 3 5 6 9 10 11 only)
 *
 * Read Preparation Commands:
 * D  NN  R          Place digital value (1 byte) for pin NN into buffer for reading
 * A  nn  R          Analog control. Read analog pin# nn (0-3, 6-7) into buffer. also 8=vcc
 * *
 *
 * Send by i2c as:  cmd  count  pin  job  param1  param2
 * the extra byte "count" (array size, ie usually 4) comes from smbus.write_block_data()
 * "param2" is not currently used
 *
 *
 * Reads:
 * (Read1)           Fetch buffer count. Buffer16 ready for read yet? (Loop here until non zero)
 * (Read2)           Send back buffer16[0-7]
 * (Read3)           Send back buffer16[8-15]
 *
 *********************************************************************************
 */

#define I2C_ADDR    0x05
// could have several ardex devices with different i2c addresses!

#define ArdEx5VoltIsOK false
// Only set this true if the ArdEx arduino connects to rPi via a hardware 3.3V - 5V level convertor
// This option permits 5V operation on the arduino. (Meduino device 3v/5v switchable was assumed.)     

// Commands

#define CMD_SETPINMODE 'M'
#define CMD_DIGITAL    'D'
#define CMD_ANALOG     'A'
#define CMD_PWM        'P'

#define MODE_OUTPUT    'O'
// There is no MODE_INPUT. All pins exc 13 are input by default.
// Philosophy is that rPi sets pins as required JUST ONCE per ArdEx reset, not swapping around!

#define JOB_WRITE      'W'
#define JOB_READ       'R'

#define MIN_APIN     0
#define MAX_APIN     7

#define MIN_DPIN     2
#define MAX_DPIN    17
// Digital 14-17 same physical pins as Analog 0 - 3

// standard arduino libraries:
#include <Wire.h>

// custom classes, part of ArdEx:
#include "Analog.h"
#include "ibuf.h"

// Global/static variables/buffers, class objects:

int buffer16 = 0;                  // results for read request, waiting to transmit to rPi
char readyflag = 0;                // buffer count available for reading by rPi. zero if not ready
char bufpointer = 0;               // 0 means readyflag will tx.  1,2 = byte of buffer16 will transmit next
unsigned int dig16 = 0;            // maintained as 16 digital bit values of pins d2-d17
Analog analog;                     // background analog read system
I2Cbuffer i2cBuffer;               // Holds incoming i2c packets until convenient to process.

void setup ()
{
    int pin ;
    for (pin = MIN_DPIN ; pin < MAX_DPIN ; ++pin)
    {
        digitalWrite (pin, LOW) ;
        pinMode (pin, INPUT) ;
    }     // on reset, all pins are probably already on input mode anyway!
    Serial.begin (115200);
    Serial.println("ARDEX v0.3");   
    Serial.println ("Arduino Hardware Expansion Slave.") ;
    analog.begin(4);
    pinMode(13, OUTPUT);
    if ((analog.Vcc < 3500) || ArdEx5VoltIsOK)          // switched to 3.3 volt?  (3300 mV)
    {
        Wire.begin(I2C_ADDR);         // join i2c bus
        Wire.onRequest(I2CrequestEvent); // register i2c TX request event ISR
        Wire.onReceive(I2CreceiveEvent); // register i2c RX event ISR
        Serial.print("Vcc = ");
        Serial.print(((float)analog.Vcc) / 1000);
        Serial.println (" Volts.");
        Serial.println ("I2C is ready for rPi.");
        digitalWrite (13, HIGH);   
    }
    else
    {
        Serial.print("Incoming i2c pin volts = ");
        Serial.println ((float)(analog.Vcc * analog.read(4)) / 1023000L);
        Serial.print("But Arduino is on ");
        Serial.print(((float)analog.Vcc) / 1000);
        Serial.println (" Volts. Please switch to 3.3");
        while (true)  //  loops forever
        {
            // This flashing LED means VCC switch is on 5V. rPi does not like 5v! DANGER.
            digitalWrite (13, HIGH);   
            delay(100);
            digitalWrite (13, LOW); 
            delay(100);       
        }       
    }
}


// event service function that executes when i2c data packet was received from master (rPi). 
void I2CreceiveEvent(int howMany)
{
    unsigned char job;
    // the bytes are queued for processing later, either to usual buffer or to special lcd buffer
    job = i2cBuffer.put();
    if(job == 'R' || job == 'L')
        // ie read of any kind (incl logical analog read)
        readyflag=0;
        // readyflag MUST be put off immediately, before we exit the interrupt service.
        // readyflag goes >0 only when its fetching routine loads buffer16 with return value;
    return;

}


// event service function that executes whenever data is requested by master (rPi). Sends single byte.
void I2CrequestEvent()
{
    // rPi has requested a byte of data
   
    // bufpointer (0 or 1 or 2) is the controller of what byte is sent back. (state machine)
    // initially (bufpointer=0) only readyflag is returned. If "not ready" then next write will be readyflag again.
    // if "ready" then bufpointer is incremented, and the following 2 writes will be the buffer16 halves

    if (readyflag > 0)   // buffered data is ready for reading by rPi
        bufpointer = 0;

    switch (bufpointer)
    {
    case 0:
        Wire.write(readyflag);
        if (readyflag > 0)   // only get one chance to read this flag
        {
            bufpointer = 1;
            readyflag = 0;
        }       
        break;

    case 1:
        Wire.write(highByte(buffer16));
        bufpointer = 2;
        break;

    case 2:
        Wire.write(lowByte(buffer16));
        bufpointer=0;

    default:
        bufpointer = 0;
        readyflag = 0;
        break;     

    }
}   


void processI2c_Incoming(void)
{
    // This processes one (buffered) 5 byte incoming I2C data packet, if available.
   
    unsigned char pin, cmd, job, val1, val2;

    //    exit if buffer empty
    if (! i2cBuffer.get(&cmd, &pin, &job, &val1, &val2)) // extract from buffer.
        return;
     
    switch (cmd)
    {
    case CMD_SETPINMODE:
        if ((pin < MIN_DPIN) || (pin > MAX_DPIN))
            break;
        switch (job)   
        {
        case MODE_OUTPUT:
            pinMode (pin, OUTPUT) ;   
            break;
           
        // other modes NYI

        }
        break;


    case CMD_DIGITAL:
        if ((job == JOB_READ) && (pin >= MIN_DPIN) && (pin <= MAX_DPIN))
        {
            buffer16 = digitalRead(pin) << 8;
            readyflag=1;
            break;
        }
        if (job == JOB_WRITE)
            digitalWrite (pin, val1 & 1) ;
        break;


    case CMD_ANALOG:
        // uses a0-7 numbering !
        if (pin==4 || pin==5 || pin>8)
            break;
        switch (job)
        {
           
            case JOB_READ:
                if(pin==8)
                    buffer16 = (int) analog.Vcc;
                    // this was read at reset/setup time
                else
                    buffer16 = analog.read(pin);
                    // this is maintained by analog class

                readyflag=2;
                break;
           
        }
        break;


    case CMD_PWM:
        // PWM pin must have been set as OUTPUT. Pin# restrictions are chip function.
        // Note pwm on pins 9/10 will not operate if any servo is used ( timer1 conflict)
        // And pwm on 3 and 11 wont work if infrared is used (timer2 conflict)
       
        if (job == JOB_WRITE)
            if (pin==3 || pin==5 || pin==6 || pin==9 || pin==10 || pin==11)
                analogWrite(pin, val1);       
        break; 
    }
}


void loop ()
{
    analog.run();
    processI2c_Incoming();    // i2c commands queued?
}

/*
 * FUTURE?
 * 16 digital inputs at one read
 * "Logical" read of analog pin - compare to a setpoint
 * RC servo x 2
 * LCD x 1
 * Infrared 3-pin detector x 1
 * Stepper motor x 2
 * Quadrature encoder x 2
 * Pulse Counter x 2
 * Pulse generator x 1
 * SR04 sonar x 5

*/

