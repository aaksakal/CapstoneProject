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

  pinOut.SETUP();
  SetupInterrupts();
  SetupPid();
  SetupMotors();
  
  delay(200);
}

void loop() {
  mappingJoystick(); 
  updateRotaryEncoder();
           
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
    case SIGNALL: {
         Signal(state);
         break;
    }
    case UTURN: {
         Uturn(state);
         break; 
    }
  }
}

