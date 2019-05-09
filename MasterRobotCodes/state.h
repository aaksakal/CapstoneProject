#ifndef state_h
#define state_h

#include <NewPing.h>
#include <PID_v1.h>
#include "TimerOne.h"
#include "pinOut.h"


void Straight(int& state);
void LeftTurn(int& state);
void RightTurn(int& state);
void Signal(int& state);
void Stop(int& state);
void Uturn(int& state);

void SetupPid();
void SetupInterrupts();
void SetupMotors();


void mappingJoystick();
void updateRotaryEncoder();


void displaying(int &state);
void ultrasonicDeneme();

#endif //state_h
