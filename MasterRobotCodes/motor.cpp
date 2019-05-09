#include "Arduino.h"
#include "motor.h"
#include "pinOut.h"

//Code of the motor class. To create motor objects.
//We will create a motor object representing our DC motors.
//Each motor object has methods to define their movements.
motorClass::motorClass(){

};

//These are the motor methods. 
void motorClass::motor1_forward(double motor1_pwm) {
  digitalWrite(in1, HIGH);   //  set motor1 forward
  digitalWrite(in2, LOW);
  analogWrite(enA, motor1_pwm);
}

void motorClass::motor2_forward(double motor2_pwm) {
  digitalWrite(in3, HIGH);   //  set motor2 forward
  digitalWrite(in4, LOW);
  analogWrite(enB, motor2_pwm);
}

void motorClass::motor1_backward(double motor1_pwm) {
  digitalWrite(in1, LOW);   //  set motor1 backward
  digitalWrite(in2, HIGH);
  analogWrite(enA, motor1_pwm);
}

void motorClass::motor2_backward(double motor2_pwm) {
  digitalWrite(in3, LOW);   //  set motor2 backward
  digitalWrite(in4, HIGH);
  analogWrite(enB, motor2_pwm);
}

void motorClass::motor1_stop() {
  digitalWrite(in1, HIGH);   //  set motor1 forward
  digitalWrite(in2, HIGH);
  analogWrite(enA, 0);
}

void motorClass::motor2_stop() {
  digitalWrite(in3, HIGH);   //  set motor2 forward
  digitalWrite(in4, HIGH);
  analogWrite(enB, 0);
}

motorClass motor = motorClass();
