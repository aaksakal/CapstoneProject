#include "pinOut.h"
#include "motor.h"
#include "state.h"
#include "variable.h"
#include "isr.h"


int state= STRAIGHT;


void setup() {
  Serial.begin(115200);
  while(!Serial){
    ;
  }
  delay(1000);
  pinOut.SETUP();
  SetupInterrupts();
  SetupPid();
  SetupMotors();
  
  
}

void loop() {
  
  //Main FSM Starts Here.
  //Each State has a function to carry. We call individual functions for each state.
  //The State Functions are written in state.cpp & state.h files.
  //Motor.cpp & motor.h files are used for motor functions such as move forward, backward or stop.
  //Pinout.h is used for pin definitions for the sensors. They are used to understand which sensor is
  //connected to which pin of Arduino.
  switch (state) {
    case STRAIGHT: {
        Straight(state);    
        break;
    }
    case LEFT_TURN: {
        LeftTurn(state);
        break;
    }
    case RIGHT_TURN: {
        RightTurn(state);
        break;
    }
    case SIGNAL: {
         Signal(state);
         break;
    }
    case STOP: {  
        Stop(state);
        break;
    }
    case UTURN: {
        Uturn(state);
        break;
    }
  }
}

