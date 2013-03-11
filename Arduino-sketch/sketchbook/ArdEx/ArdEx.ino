/*
 *  ardex02.ino     BY BLAVERY
 *  "ARDEX 0.2.2" 
 *  Using a 3.3v Meduino Nano (Arduino clone) to do rPi I/O hardware interface expansion.
 * 
 *  Allow Pi (i2c master) talking to arduino (i2c slave) to control 18 Arduino IO pins. 
 *  (Very modest smarts in the arduino. Real work still to do on the Pi!)
 *  Available:  d2-d17 digital i/o  (d14-d17 aka a0-a3)
 *              a0-a3 and a6-a7 as analog in.
 *  Optionally, 6 pins can do PWM, 2 pins can drive RC servos, d13 is commonly user-driven LED,
 *              2 pins can be impulse counters, 2 pairs can be quadrature (up/down) counters,
 *              Configured alarms (anl, dig) can set status flags or send interrupt to rPi.
 *  Unavailable: Uart on d0-d1 considered tied to usb, and a4-a5 are devoted to i2c.
 * 
 * Write Commands:
 * M  NN  mode      Pin# NN (2-17) Set pin mode.        Mode 'O' =output     'S' =RC-servo   'I' I'red (d11 only)
 * M  nn  mode      E=counter (1-pin nn: a0 a1 only )  Q=quadencoder (a0 a1 onlt - uses a2&a3 as quad)
 * D  NN  on.off    Pin# NN (2-17) Digital output control. even=off/LOW odd=on/HIGH   eg '0' & '1'
 * D  NN  Eb        Write 4 bits bbbb to outputs on pins NN (d2-9) to NN+3 (ie bridge/stepper control)
 * P  NN  val8      Write PWM value (pins 3 5 6 9 10 11 only)
 * A  nn  val8      val8 1-1023: Set a setpoint for logical read on analog
 * S  NN  val8      Write new position to RC servo (pins 7 - 10 only)
 * E  Dn  val8      Encoder debounce tuning   0=none
 * E  An            Enable encoder by analog logical read instead of digitalread
 * F  0r  val8      Write a flagregister r.  
 * G  NN  bits      Generate square wave on pin (d2-17). 
 * 
 * Read Preparation Commands:
 * D  NN  FF        Place digital value (1 byte) for pin NN into buffer16 for reading
 * D  FF            Place all 16 digital inputs as bits into buffer16.
 * A  nn  EF        Analog control. Read analog pin# nn (0-3, 6-7) into buffer16. also 8-vcc
 * A  nn  FF        Analog. return logical comparison with setpoint. 1 byte 0/1 (equiv digitalRead)
 * E  nn            Encoder. Read 2 bytes of encoder count #0 or #1 for nn into buffer16. a0-a1 
 * F  Fr            Read status. Place statusflags (or other status info?) to buffer16 (1 byte)
 * I  NN            Read infrared char. Place to buffer16. (2 b)
 * 
 * Reads:
 * (Read1)           Fetch buffer count. Buffer16 ready for read yet? (Loop here until non zero)
 * (Read2)           Send back buffer16[0-7]
 * (Read3)           Send back buffer16[8-15]
 * 
 *********************************************************************************
 */

#define I2C_ADDR    0x05

// Commands 

#define CMD_SETPINMODE 'M'
#define CMD_DIGITAL    'D'
#define CMD_ANALOG     'A'
#define CMD_PWM        'P'
#define CMD_SERVO      'S'
#define CMD_ENCODER    'E'
#define CMD_QUAD_ENC   'Q'
#define CMD_FLAGS      'F'
#define CMD_PULSEGEN   'G'
#define CMD_INFRARED   'I'

#define MODE_OUTPUT   'O'
#define MODE_SERVO    'S'
#define MODE_ENCODER  'E'
#define MODE_QUAD_ENC 'Q'
#define MODE_INFRARED 'I'

#define MIN_APIN     0
#define MAX_APIN     7

#define MIN_DPIN     2
#define MAX_DPIN    17
// Digital 14-17 same pins as Analog 0 - 3

#include <Wire.h>
#include <Servo.h>
#include <Analog.h>
#include <QuadEncoder.h>
#include <IRremote.h>
#include <IRin.h>
#include <PulseGen.h>

int buffer16 = 0;              // results for read request, waiting to transmit to rPi
char readyflag = 0;            // buffer count available for reading by rPi. zero if not ready
char bufpointer = 0;           // 0 means readyflag will tx.  1,2 = byte of buffer16 will transmit next
Servo servo[4];                // 4 servo objects reserved in case needed. Not active yet.
unsigned int dig16 = 0;        // maintained as 16 digital bit values of pins d2-d17
Analog analog;                 // background analog read system
QuadEncoder encoder[2];        // Simple or quad counters for wheels
IRrecv irrecv(IR_PIN);
IRin ir(&irrecv);
PulseGen strober;



#define I2C_BUFSIZE         8          // must be power of 2
uint8_t  I2C_Buffer[3][I2C_BUFSIZE];   // the buffer of stored i2c inputs waiting for processing
uint8_t  I2C_In = 0;                   // index into buffer array
uint8_t  I2C_Out = 0;                  // index into buffer array


char flagReg[] = { 
    0,0,0,0,0,0,0,0,0,0, 0, 0, 0, 0, 0, 0};
enum {
    F_outpin, F_mask, F_uptest, F_downtest, F_anlpin, F_anl_testhi, F_anl_msb, F_anl_lsb, 
    F_userflag, F_9, F_10, F_11, F_12, F_13, F_14, F_statusflags };  // 15: in 4 bits!
enum {FB_anl, FB_dig, _fb2, FB_ir, _fb4, _fb5, FB_user};  // flag bits  nyi=stepper, syserror


void setup ()
{
    int pin ;
    for (pin = MIN_DPIN ; pin < MAX_DPIN ; ++pin)
    {
        digitalWrite (pin, LOW) ;
        pinMode (pin, INPUT) ;
    }     // on reset, all pins are probably on input mode anyway!
    Serial.begin (115200);
    Serial.println("ARDEX v0.2.2");   
    Serial.println ("Arduino Hardware Expansion Slave.") ; 
    analog.begin(4);
    pinMode(13, OUTPUT);
    if (analog.Vcc < 3500)          // switched to 3.3 volt?  (3300 mV)
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
        while (0 == 0)  //  loops forever
        {
            // This flashing LED means VCC switch is on 5V. rPi does not like 5v!
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
    // throw away if buffer full.  compute for circular buffer!
    // Speed of i2c repeats vs loop() speed: quite likely buffer never get more than 1 entry!
    if ((((I2C_BUFSIZE+I2C_Out)-I2C_In) & (I2C_BUFSIZE-1)) != 1)
    {
        I2C_Buffer[0][I2C_In] = Wire.read(); // cmd
        I2C_Buffer[1][I2C_In] = Wire.read(); // pin
        I2C_Buffer[2][I2C_In] = Wire.read(); // val8   (=FFFF if it was a 2-byte transfer)
        if (++I2C_In == I2C_BUFSIZE)
             I2C_In = 0;      // rollover
    }
    // the three bytes are queued for processing later
    
    while (Wire.available()>0)
        Wire.read();    // flush away anything left? Should be nothing.
    return;
}





// event service function that executes whenever data is requested by master (rPi). Sends single byte.
void I2CrequestEvent()
{
    // rPi has requested a byte of data 
    
    // bufpointer (0 or 1 or 2) is the controller of what byte is sent back
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


    // This processes one (buffered) 3 byte incoming I2C data packet, if available. 
    
    unsigned int pin, cmd, val8, k, freq ;

    //    exit if buffer empty
    if (I2C_In == I2C_Out)
        return;
    // extract from buffer.    
    cmd = I2C_Buffer[0][I2C_Out];
    pin = I2C_Buffer[1][I2C_Out];
    val8 = I2C_Buffer[2][I2C_Out];
    if (++I2C_Out == I2C_BUFSIZE)
          I2C_Out = 0;

     
    switch (cmd)
    {
    case CMD_SETPINMODE:
        if ((pin < MIN_DPIN) || (pin > MAX_DPIN))
            break;
        switch (val8)   
        {
        case MODE_OUTPUT:
            pinMode (pin, OUTPUT) ;    
            break;

        case MODE_SERVO:
            // servos use timer t1. So does PWM on 9 and 10. 
            if (pin>=7 && pin<=10)             // only pins 7-10
                 servo[pin-7].attach(pin) ;   // magic by the Servo library
            // a slight pause at rPi might be wise before next i2c transaction
            break;  

        case MODE_QUAD_ENC:
        case MODE_ENCODER:
            // only accept a0 or a1. (Recall a0 == d14.)
            // Each encoder uses 2 pins, a0+a2  and/or  a1/a3
            if (pin == 0 || pin == 1)
                encoder[pin].begin(pin, val8 == MODE_QUAD_ENC);
           break;
           
        case MODE_INFRARED:
            if (pin==IR_PIN)
                ir.begin();   
            break;

        } 
        break;


    case CMD_DIGITAL: 
        if (pin == 0xFF)
        {
            buffer16 = dig16;
            readyflag=2;
        }
        if ((pin < MIN_DPIN) || (pin > MAX_DPIN))
            break;
        if (val8 == 0xFF)
        {
            buffer16 = digitalRead(pin) << 8;
            readyflag=1;
        }
        else if ((val8&0xE0) == 0xE0)
        {
            for (k=0 ; k<=3 ; k++)
            {
                digitalWrite (pin+k, (val8 >> k) & 1);
            }
            break;
        }
        else
            digitalWrite (pin, val8 & 1) ; 
        break;


    case CMD_ANALOG:
        // uses a0-7 numbering !

        if ((pin < MIN_APIN) || (pin > (MAX_APIN+1)))
            break;
        if (val8 == 0xEF)
        {
            if(pin==8)
                buffer16 = (int) analog.Vcc;
            else
                buffer16 = analog.read(pin);
            readyflag=2;
        }
        else if(val8==0xFF && pin <= MAX_APIN)
        {
            buffer16 = analog.logicalRead(pin);
            readyflag=1;
        }
        else
            if (pin <= MAX_APIN)
                analog.setPoint(pin, val8 & 0x03FF);
        break;


    case CMD_PWM:
        // PWM pin must have been set as OUTPUT. Pin# restrictions are chip function.
        // Note pwm on pins 9/10 will not operate if any servo is used ( timer1 conflict)
        if (pin==3 || pin==5 || pin==6 || pin==9 || pin==10 || pin==11)
            analogWrite(pin, val8);        
        break;  


    case CMD_SERVO:
        if (pin>=7 && pin<=10)
            servo[pin-7].write(val8) ;
        break;


    case CMD_QUAD_ENC:
    case CMD_ENCODER:
        // we want to fetch the current counter value. pins (a)0 or 1
        if((pin & 0xD0) == 0xD0)
            encoder[pin&1].debounce = val8;
        else if ((pin&0xA0) == 0xA0)
            encoder[pin&1].enableAnalog(&analog);
        else
        {
            buffer16 = (int) encoder[pin&1].readCounter();
            readyflag = 2;   // now rPi can collect
        }
        break;

    case CMD_PULSEGEN:
        strober.start(pin, 100, 100, true, false);    // NYI *********************
        break;

    case CMD_INFRARED:
        buffer16 = ir.getValue();
        readyflag=2;
        break;
        
    case CMD_FLAGS:
        if (pin < F_statusflags)       // not really PIN#, rather a register#
            flagReg[pin] = val8;
        else if (pin == 0xFF)
        {
            buffer16 = flagReg[F_statusflags];
            readyflag=1;
        }
        break;
    }

}



int dpin=0;    
long m, ctr=0L;
void loop ()
{
    int pin, temp, k;

    // 1. 
    processI2c_Incoming();    // i2c commands queued?
    
    
    // 2. Do analog captures in background 
    analog.run();


    // 3. Check quadrature encoder sensors. Adjust counters (+ or -) as required.
  
    encoder[0].run();
    encoder[1].run();
    
    

    // 4. IR in
    ir.run();
    
    

    // 5. Maintain dig16 as bit array of outputs 2-17  -  about 400 uSec update cycle?

    pin = 2 + (0x0F & dpin);                   // 2 to 17
    bitWrite (dig16, pin-2, digitalRead(pin)); // just one read each loop ("cost" = 4 usec)
    dpin++;



    // 6. Test the 8 digitals 5-12 to see if a FLAG should be set
    temp = (dig16 >> 3 )& 0x00FF;           // = 8 bits of d5-d12
    k= ((temp & flagReg[F_uptest]) | ((~temp) & flagReg[F_downtest]));
    // ie k true if any uptested pin is up or any downtested pin is down
    bitWrite(flagReg[F_statusflags], FB_dig, k !=0);




    // 7. Test nominated analog against its alarm setpoint. Set a FLAG?

    if ((flagReg[F_anl_testhi]&0xFE) != 0)                              // is analog test to be done?
    {
        temp = analog.read(flagReg[F_anlpin]&7);
        k = (temp < ((flagReg[F_anl_msb] <<8) & flagReg[F_anl_lsb]));   // k=1 below setpoint, =0 above 
        bitWrite(flagReg[F_statusflags], FB_anl, k ^ (flagReg[F_anl_testhi] & 1) );
    }  


    // 8. IR char?
    bitWrite(flagReg[F_statusflags], FB_ir, (ir.available()));
    
     
    // 9. User flag 
    bitWrite(flagReg[F_statusflags], FB_user, (flagReg[F_userflag]!=0));



    // 10. If any flags set, and an output interrupt for rPi is nominated - assert interrupt pin    
    pin =   flagReg[F_outpin];     // is there an interrupt-out pin set?
    if (pin >=MIN_DPIN and pin <= MAX_DPIN)
        // apply mask against flags. If any still standing, set int-out pin high.
        digitalWrite(pin, ((flagReg[F_mask] & flagReg[F_statusflags]) != 0));


    // 11. Looping monitor
    // keep loops/sec for readout, also recent slowest & fastest loop times. 
    // Target 35 uSec/loop at idle (28,000 loops/sec)

    // loop counter
    if (ctr==0)
        m = millis() + 1000;   
    ctr++; 
    if (millis() >= m)
    {
        //Serial.println(1000000L/ctr);    // in uSec/loop
        ctr = 0L;
    }
    // Rule of thumb timing "costs":
    // analogRead() -  100 uSec -"wiring" standard blocking function: we don't use at all!
    // analogWrite(0 - 18 uSec , ie pwm 
    // digitalRead() - 4 uSec 
    // digitalWrite() - 4 uSec
  
  
  

}


// servo timer1 - 7- 10
// pwm timer0 ( 5 6 )  timer1 ( 9 10 )  timer2 ( 3 11 )
// pulsegen timer2


//////////////////////////////////////////////////////////////


