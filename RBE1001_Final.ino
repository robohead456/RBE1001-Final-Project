//INCLUDE REQUIRED LIBRARIES AND FILES (<> FOR LIB, "" FOR FILE)
#include "VEX_Quad_Encoder.h"
#include <Servo.h>
#include <DFW.h>

//CONSTANTS OF THE ROBOT
const float w = 11.5; //Wheel track of the robot
const float d = 2.75; //Wheel diameter of the robot
const float kp_turn = 2.2; //Proportional constant for driving
const float kp_drive = 1.0; //Proportional constant for driving straight 
const int driveError = 10;
const int turnError = 8;
const int locked = 32; //servo position for locked arm
const int unlocked = 90; //servo position for unlocked arm
const int lightToggle = 985; //light<lightToggle, dark>lightToggle

//PINS THAT THINGS ARE CONNECTED TO
const int rEncoderA = 2;
const int rEncoderB = 3;
const int lEncoderA = 20;
const int lEncoderB = 21;
const int liftMotorPin = 4;
const int rMotorPin = 5;
const int lMotorPin = 6;
const int sweeperMotorPin = 7;
const int lockPin = 11;
const int ledPin =13;
const int lLine = A0;
const int mLine = A1;
const int rLine = A2;
const int autoSelect1 = 27;
const int autoSelect2 = 28;
const int autoSelect3 = 29;
const int redLed = 22;
const int greenLed =23;
const int blueLed = 24;

//DFW OBJECT
DFW dfw(ledPin);

//ENCODER OBJECTS
VEX_Quad_Encoder rEncoder(rEncoderA,rEncoderB); //r goes - when driving forward
VEX_Quad_Encoder lEncoder(lEncoderA,lEncoderB); //l goes - when driving forward

//MOTOR OBJECTS
Servo rMotor;
Servo lMotor;
Servo liftMotor;
Servo sweeperMotor;
Servo lock;

//Interrupt Service Routine helper methods
//  An ISR can't be a member of a class if it isn't static, so these are the ISRs called by the interupts and they are acting as helper methods to call the method for each object
void ISR_RA() {rEncoder.changeA();}
void ISR_RB() {rEncoder.changeB();}
void ISR_LA() {lEncoder.changeA();}
void ISR_LB() {lEncoder.changeB();}

//ROBOT FUNCTIONS
void turn(int deg) { //MAKE SURE TO ZERO ENCODERS BEFORE CALLING THIS FUNCTION. ccw direction is a positive deg, cw is a negative deg
  //set encoder setpoint to deg*w/d (both + to turn right, both - to turn left)
  float setpoint = deg*w/d;
  Serial.println(setpoint);
  delay(2000);
  int rSpeed;
  int lSpeed;
  while(abs(setpoint-rEncoder.getPos()) > turnError || abs(setpoint+lEncoder.getPos()) > turnError) {
    while(abs(setpoint-rEncoder.getPos()) > turnError|| abs(setpoint+lEncoder.getPos()) > turnError) {
      setpoint = deg*w/d;
      //use proportional control to move the bot to setpoint
      rSpeed = (setpoint-rEncoder.getPos())*kp_turn;
      lSpeed = (setpoint+lEncoder.getPos())*kp_turn;
      if(rSpeed>90) rSpeed =90;
      if(rSpeed<-90) rSpeed =-90;
      if(lSpeed>90) lSpeed =90;
      if(lSpeed<-90) lSpeed =-90;
      if(rSpeed>15) rSpeed-=12;
      if(rSpeed<-15) rSpeed+=12;
      if(lSpeed>15) lSpeed-=12;
      if(lSpeed<-15) lSpeed+=12;
      lMotor.write(90+lSpeed);
      rMotor.write(90+rSpeed);
      Serial.print(rEncoder.getPos());
      Serial.print(", ");
      Serial.println(lEncoder.getPos());
      delay(20);
    }
    delay(50);
  }
  Serial.println(rEncoder.getPos());
  lMotor.write(90);
  rMotor.write(90);
}

/*void drive(int dist) { //MAKE SURE TO ZERO ENCODERS BEFORE CALLING THIS FUNCTION. forward is +, backwards is -
  //set encoder setpoint to dist*w/d
  int setpoint = dist*w/d;
  //use proportional control to move the bot to setpoint
  int rSpeed = (setpoint+rEncoder.getPos())*kp_drive;
  int lSpeed = (setpoint-lEncoder.getPos())*kp_drive;
  lMotor.write(lSpeed);
  rMotor.write(rSpeed);
}*/

//SETUP FUNCTION
void setup() {
  //BEGIN SERIAL FOR DEBUG OUTPUT
  Serial.begin(9600);

  //BEGIN DFW
  dfw.begin(9600,1);

  //ATTACH INTERUPTS TO ENCODER PINS
  attachInterrupt(digitalPinToInterrupt(rEncoderA), ISR_RA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rEncoderB), ISR_RB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(lEncoderA), ISR_LA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(lEncoderB), ISR_LB, CHANGE);

  //ATTACH MOTORS
  rMotor.attach(rMotorPin,1000,2000);
  lMotor.attach(lMotorPin,1000,2000);
  liftMotor.attach(liftMotorPin,1000,2000);
  sweeperMotor.attach(sweeperMotorPin,1000,2000);
  lock.attach(lockPin); //servo, not motor so no 1000,2000

  //SET SENSOR PINS AS INPUT
  pinMode(lLine,INPUT);
  pinMode(mLine,INPUT);
  pinMode(rLine,INPUT);
  pinMode(autoSelect1,INPUT_PULLUP);
  pinMode(autoSelect2,INPUT_PULLUP);
  pinMode(autoSelect3,INPUT_PULLUP);
  pinMode(redLed,OUTPUT);
  pinMode(greenLed,OUTPUT);
  pinMode(blueLed,OUTPUT);

  //WAIT FOR CONTROLLER TO CONNECT
  while (dfw.joystickrv() < 150) {
    digitalWrite(redLed,HIGH);
    dfw.update();
    delay(100);
  }
  digitalWrite(redLed,LOW);
  delay(1000);
}

//AUTON FUNCTION
void autonomous(volatile unsigned long time) {

  //WAIT FOR START BUTTON TO BE PRESSED
  Serial.println("Auton:");
  digitalWrite(blueLed,HIGH);
  while(dfw.start()==1) {
    dfw.update();
    delay(20);
  }
  digitalWrite(blueLed,LOW);
  digitalWrite(greenLed,HIGH);  
  Serial.println("Running...");

  //STATE MACHINES
  int a1= digitalRead(autoSelect1);
  int a2= digitalRead(autoSelect2);
  int a3= digitalRead(autoSelect3);
  
  enum autoModes{noAuto,inner,outer,block} autoMode;
  if(a2 ==0 && a3 ==0) autoMode = block;
  else if (a2 == 0 && a3 ==1) autoMode = inner;
  else if(a2 == 1 && a3 ==0) autoMode = outer;
  else autoMode = noAuto;
  
  enum fieldSides{red,blue} fieldSide;
  if(a1 == 0) fieldSide = red;
  else fieldSide = blue;

  enum autoSteps{step0,step1,step2,step3,step4,step5,step6,step7,step8,step9,step10,step11,step12,step13} autoStep;
  autoStep = step0;

  Serial.println(autoMode);
  Serial.println(fieldSide);
  Serial.println(autoStep);
  
  //START TIMER FOR AUTON
  unsigned long startTime = millis();
  time *= 1000; //seconds to milliseconds
  //CHECK IF AUTON TIME IS DONE OF IF SKIP AUTON BUTTON IS PRESED
  while ((millis() - startTime <= time) && dfw.select()==1) {
    //AUTON CODE HERE

    switch(autoMode) {

      //When starting in the outer positions
      case outer:
        switch(autoStep) {
          //Drive forward to pass red/blue
          case step0:
            rMotor.write(30);
            lMotor.write(150);
            delay(1500);
            autoStep = step1;
            break;

          //Drive until line sensor detects a line
          case step1:
            rMotor.write(30);
            lMotor.write(150);
            if((analogRead(rLine) < lightToggle && fieldSide == red)||(analogRead(lLine) < lightToggle && fieldSide == blue)) {
              rMotor.write(90);
              lMotor.write(90);
              autoStep = step2;
            }
            delay(20);
            break;

          //Turn 90 deg (its set to slightly over 90 to compensate for errors)
          case step2:
            //quick reverse to stop overshoot
            lMotor.write(50);
            rMotor.write(130);
            delay(200);
            rMotor.write(90);
            lMotor.write(90);
            //zero encoder positions and turn
            rEncoder.zeroPos();
            lEncoder.zeroPos();
            if(fieldSide==red) turn(90);
            else turn(-90);
            //Drive forward to ensure the light sensors cross the tape line
            rMotor.write(30);
            lMotor.write(150);
            delay(500);
            rMotor.write(90);
            lMotor.write(90);
            autoStep = step3;
            break;

          //Drive until line sensor detects a line
          case step3:
            rMotor.write(30);
            lMotor.write(150);
            if((analogRead(lLine) < lightToggle && fieldSide == red)||(analogRead(rLine) < lightToggle && fieldSide == blue)) {
              rMotor.write(90);
              lMotor.write(90);
              autoStep = step4;
            }
            delay(20);
            break;

          //Turn 90 deg (opposite direction) (its set to slightly over 90 to compensate for errors)
          case step4:
            //quick reverse to stop overshoot
            lMotor.write(50);
            rMotor.write(130);
            delay(70);
            rMotor.write(90);
            lMotor.write(90);
            //zero encoder positions and turn
            rEncoder.zeroPos();
            lEncoder.zeroPos();          
            if(fieldSide==red) turn(-100);
            else turn(100);
            autoStep = step5;
            break;

          //Do EGG things
          case step5:
            //Drive forward
            rMotor.write(30);
            lMotor.write(150);
            delay(1000);
            //Start the sweeper
            sweeperMotor.write(180);
            delay(1500);
            //Drive backward and start lift
            lMotor.write(30);
            rMotor.write(150);
            liftMotor.write(180);
            delay(500);
            //drive forward
            rMotor.write(40);
            lMotor.write(140);
            delay(1000);
            //Reverse the sweeper and stop driving
            sweeperMotor.write(0);
            rMotor.write(90);
            lMotor.write(90);
            delay(500);
            //Stop lift motor
            liftMotor.write(90);
            autoStep = step6;
            break;

          //defualt/error/finished case
          default:
            delay(20);
        }
        break;

      //When starting in the inner positions
      case inner:
        switch(autoStep) {
          case step0:
            //drive forward set distance
            break;
          case step1:
            //drive to white line
            //drive for a short time to cross the line
            //drive to white line
            //quick reverse to prevent overshooting
            break;
          case step2:
            //drive until a line 
          default:
            break;
        }
        break;

      //When starting on the inside intending to block an opposing auto
      case block:
        switch(autoStep) {
          //only case
          case step0:
            if(fieldSide == red) { //left goes slightly faster
              rMotor.write(30);
              lMotor.write(160);
            }
            else {//right goes slightly faster
            rMotor.write(20);
            lMotor.write(150);
            }
            delay(4000);
            rMotor.write(90);
            lMotor.write(90);
            delay(1500);
            autoStep = step1;
            break;
          default:
            delay(20);
        }

      //If no auto mode or wrong state do nothing
      case noAuto:
      default:
        break;
    }
    
    //END AUTON CODE
    dfw.update(); //used for autonoumous skip
    delay(20); //delay to prevent spamming the serial port and to keep servo and dfw libraries happy
  }
  Serial.println("Auton done.");
}

//TELEOP FUNCTION
void teleop(unsigned long time) {
  
  Serial.println("Teleop:");
  digitalWrite(blueLed,HIGH);
  
  unsigned long startTime2 = millis(); //sets start time of teleop
  time *= 1000; //seconds to milliseconds  
  //CHECK TO SEE IF TELEOP TIME IS DONE
  while (millis() - startTime2 <= time) {
    dfw.update(); //get new values from controller
    //TELEOP CODE HERE
    
    rMotor.write(180-dfw.joystickrv());
    lMotor.write(dfw.joysticklv());

    if(dfw.up()==0) liftMotor.write(180);
    else if(dfw.down()==0) liftMotor.write(0);
    else liftMotor.write(90);

    if(dfw.two()==0) sweeperMotor.write(90);
    else if(dfw.r1()==0) sweeperMotor.write(180);
    else if(dfw.r2()==0) sweeperMotor.write(0);

    if(dfw.select()==0) {
      lock.write(unlocked);
      Serial.println(unlocked);
    }
    else if(dfw.start()==0) {
      lock.write(locked);
      Serial.println(locked);
    }

    if(dfw.four()==0) {
      rMotor.write(40);
      lMotor.write(140);
    }
    
    //END TELEOP CODE
    delay(20); //delay to prevent spamming the serial port and to keep servo and dfw libraries happy
  }
}

//LOOP FUNCTION
void loop() {
  //RUN AUTON FOR 20 SEC
  autonomous(20);
  
  //RUN TELEOP FOR 180 SEC
  teleop(180);
  digitalWrite(greenLed,LOW);
  digitalWrite(blueLed,LOW);
  digitalWrite(redLed,HIGH);
  delay(5000);
  teleop(900000);
  exit(1);
}

