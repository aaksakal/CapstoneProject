#ifndef state_h
#define state_h

#include <NewPing.h>
#include <PID_v1.h>
#include "TimerOne.h"
#include "pinOut.h"
#include "variable.h"
#include "isr.h"

#define MAXPWM 100
#define MINPWM  0


void Straight(int& state);
void LeftTurn(int& state);
void RightTurn(int& state);
void Signal(int& state);
void EndOfRoad(int& state);

void Uturn(int& state);
void umappingJoystick();

void mappingJoystick();
void pwmCheck(double &pwmValue);

void displaying(int& state);
void SetupPid();
void SetupInterrupts();
void SetupMotors();
void SetupRotary();

void updateRotaryEncoder();


#endif //state_h
