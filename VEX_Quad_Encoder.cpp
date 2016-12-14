#include "Arduino.h"
#include "VEX_Quad_Encoder.h"

int pinA, pinB;
volatile int pos = 0; //variables changed during an interrupt must be volatile

VEX_Quad_Encoder::VEX_Quad_Encoder(int A, int B) {
  pinA = A;
  pinB = B;
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
}

long VEX_Quad_Encoder::getPos(){
  return pos;
}

void VEX_Quad_Encoder::zeroPos(){
  pos = 0;
}

void VEX_Quad_Encoder::changeA() {
  if (digitalRead(pinA)) { 
    if (digitalRead(pinB)) pos++; // A==HIGH && B==HIGH : CW 
    else pos--; // A==HIGH && B==LOW : CCW
  }
  else { 
    if (digitalRead(pinB)) pos--; // A==LOW && B==HIGH : CCW
    else pos++; // A==LOW && B==LOW : CW
  }
}

void VEX_Quad_Encoder::changeB() {
  if (digitalRead(pinB)) {   
    if (digitalRead(pinA)) pos--; // B==HIGH && A==HIGH : CCW
    else pos++; // B==HIGH && A==LOW : CW
  }
  else { 
    if (digitalRead(pinA)) pos++; // B==LOW && A==HIGH : CW
    else pos--; // B==LOW && A==LOW : CCW
  }
}


/* Quadrature Encoder Waveforms
     _____       _____
A   |     |     |     |
____|     |_____|     |_
        _____       _____
B      |     |     |     |
   ____|     |_____|     |_

--> CW
<-- CCW
*/

