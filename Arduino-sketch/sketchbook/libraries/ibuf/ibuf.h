//   ibuf.h

#ifndef IBUF_h
#define IBUF_h

#define I2C_BUFSIZE         16          // must be power of 2

class I2Cbuffer
{


    public:
    I2Cbuffer(void);
    unsigned char put(void);
    bool get(unsigned char* cmd, unsigned char* pin, unsigned char* job, unsigned char* v1, unsigned char* v2);
    char cmd;
    char pin;
    char job;
    char val1;
    char val2;
   
    private:
   
    uint8_t  _I2C_Buffer[6][I2C_BUFSIZE];   // the buffer of stored i2c inputs waiting for processing
    uint8_t  _I2C_In;                   // index into buffer array
    uint8_t  _I2C_Out;                  // index into buffer array
   
   
} ;


#endif
