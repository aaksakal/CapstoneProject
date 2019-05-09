#ifndef motor_h
#define motor_h

#include "variable.h"

class motorClass
{
  public:
    motorClass();

    void motor1_forward(double motor1_pwm);
    void motor2_forward(double motor2_pwm);
    void motor1_backward(double motor1_pwm);
    void motor2_backward(double motor2_pwm);
    void motor1_stop();
    void motor2_stop();
};

extern motorClass motor;

#endif //motor_h
