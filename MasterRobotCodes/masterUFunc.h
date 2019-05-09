#ifndef masterUFunc_h
#define masterUFunc_h

#include <NewPing.h>
#include <PID_v1.h>
#include "TimerOne.h"
#include "pinOut.h"

void straightWithJoystick();
void backWithJoystick();

void sonarGatherData();
void straightWithPID();
void backWithPID();
void left90();
void right90();
void turnBack();
int pathFinder();
void goForaWhile();
void rotaryReset();

#endif
