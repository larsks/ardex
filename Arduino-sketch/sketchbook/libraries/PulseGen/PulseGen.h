
#ifndef PGEN_h
#define PGEN_h






class PulseGen
{
  public:
    PulseGen(void);
    void run(void);
    void start(int pin, int millihi, int millilo, bool startlo, bool continuous);
  
  private:
    int msechi;
    int mseclo;
    long prevmillis;
    int pin;
    int currentstate;
    int currentcount;
    bool continuous;
    
};


#endif
