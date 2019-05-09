//This ifndef part is for the compiler to stop it from calling the motor.h library a million times.
//It is read as if not defined before, define motor_h such as, it has a class motorClass ........
#ifndef motor_h
#define motor_h

//This is the definition of the motor class.
//Our code looks here first and understands what the motor class is.
//And whenever it needs to run a function(method) of the motor class, it will
//check motor.cpp file. .h files are for definition only.
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
