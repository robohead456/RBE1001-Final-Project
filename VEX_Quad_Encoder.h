#ifndef VEX_Quad_Encoder_h
#define VEX_Quad_Encoder_h

#include "Arduino.h"

class VEX_Quad_Encoder {
  public:
    VEX_Quad_Encoder(int pinA, int pinB); // pinA and pinB must be interupt pins
    long getPos();
    void zeroPos();
    void changeA();  //ISR would need to be static -> create ISR in main code to call the method for each instance
    void changeB();
  private:
    static const int tickPerRev = 360;
    int pinA;
    int pinB;
    volatile long pos = 0L; //variables changed during an interrupt must be volatile
};
#endif
