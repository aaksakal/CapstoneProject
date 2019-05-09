#ifndef isr_h
#define isr_h

#include <NewPing.h>
#include <PID_v1.h>
#include "TimerOne.h"
#include "pinOut.h"
#include "variable.h"

void ISR_motor1();
void ISR_motor2();
void ISR_timerone();

#endif //isr_h
