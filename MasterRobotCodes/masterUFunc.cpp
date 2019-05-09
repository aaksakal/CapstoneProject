#include "Arduino.h"
#include "motor.h"
#include "state.h"
#include "masterUFunc.h"
#include "isr.h"
#include "variable.h"

void straightWithJoystick() {
  mappingJoystick();
  Serial.println(y_axis);
    if(y_axis >= 477){
    straightWithPID();
  }else {
     motor.motor1_stop();
     motor.motor2_stop();
  }
};

void backWithJoystick() {
  mappingJoystick();
  Serial.println(y_axis);
    if(y_axis <= 450){
    backWithPID();
  }else {
     motor.motor1_stop();
     motor.motor2_stop();
  }
};

void sonarGatherData() {
  //Gather Data from right-left sensor, and every 10th time from front sensor
  distance_left = sonar_left.ping_cm();
  delayMicroseconds(40);
  if(distance_left==0) {
    distance_left = 135;
  }
  distance_right = sonar_right.ping_cm();
  if(distance_right==0) {
    distance_right = 135;
  }
  delayMicroseconds(40);
  distance_front = sonar_front.ping_cm();
  if(distance_front==0) {
    distance_front = 135;
  }
};


void straightWithPID() {
  Timer1.attachInterrupt( ISR_timerone );
  if(distance_right>12 && distance_left<20) {
        //A right turn is approaching
        side_error = 5 - distance_left; //Go with only the left sensor.
        
      }
      else if(distance_right<20 && distance_left>12) {
        //This means a left turn is approaching.
        side_error = distance_right - 5;
        //Try to move straight with only 1 sensor. Until we get close enough.
        
      }
      else {
        //If non of these are applicable then keep moving straight with both sensors.
        side_error = distance_right - distance_left;
      }
      //Compute PID controller with the side_error data.
      //PID was initiated with this data, so we don't need to feed it to PID again.
      //We just call the Compute method to initiate computing.
      ultra_PID.Compute();
      motor1_pwm = fixed_pwm - Output;
      motor2_pwm = fixed_pwm + Output;
      if(motor1_pwm > 80)
        motor1_pwm = 80;
      else if(motor1_pwm < 0)
        motor1_pwm = 0;
      if(motor2_pwm > 80)
        motor2_pwm = 80;
      else if(motor2_pwm < 0)
        motor2_pwm = 0;
      motor.motor1_forward(motor1_pwm);
      motor.motor2_forward(motor2_pwm);

  delay(20);
};

void backWithPID() {
  Timer1.attachInterrupt( ISR_timerone );
  if(distance_right>12 && distance_left<20) {
        //A right turn is approaching
        side_error = 5 - distance_left; //Go with only the left sensor.
        
      }
      else if(distance_right<20 && distance_left>12) {
        //This means a left turn is approaching.
        side_error = distance_right - 5;
        //Try to move straight with only 1 sensor. Until we get close enough.
        
      }
      else {
        //If non of these are applicable then keep moving straight with both sensors.
        side_error = distance_right - distance_left;
      }
      //Compute PID controller with the side_error data.
      //PID was initiated with this data, so we don't need to feed it to PID again.
      //We just call the Compute method to initiate computing.
      ultra_PID.Compute();
      motor1_pwm = fixed_pwm - Output;
      motor2_pwm = fixed_pwm + Output;
      if(motor1_pwm > 80)
        motor1_pwm = 80;
      else if(motor1_pwm < 0)
        motor1_pwm = 0;
      if(motor2_pwm > 80)
        motor2_pwm = 80;
      else if(motor2_pwm < 0)
        motor2_pwm = 0;
      motor.motor1_backward(motor1_pwm);
      motor.motor2_backward(motor2_pwm);

  delay(20);
};

void left90() {
  motor2_pwm = 72;
  count2= 0;
  while(1)
  {
    Timer1.detachInterrupt();  // Stop the timer
    Serial.println(count2);
    if(count2<16) {
      motor.motor1_stop();
      motor.motor2_forward(motor2_pwm);
    }
    else {
       Timer1.attachInterrupt( ISR_timerone );  // Enable the timer
      break;
    }
  }
};

void right90() {
  motor1_pwm = 72;
  count1=0;
  while(1){
    Timer1.detachInterrupt();  // Stop the timer
    if(count1<15) {
      motor.motor1_forward(motor1_pwm);
      motor.motor2_stop();
    }
    else {
     Timer1.attachInterrupt( ISR_timerone );  // Enable the timer
     break;
    }
  }
};

void turnBack() {
  motor1_pwm = 80;
  motor2_pwm = 80;
  count1= 0;
  count2= 0;
  Timer1.detachInterrupt();  // Stop the timer
  while(1){
    Serial.println(count1);
    Serial.println(count2);
    if(count1<14 && count2<14) {
      motor.motor1_forward(motor1_pwm);
      motor.motor2_backward(motor2_pwm);
    }else if (count1>14 && count2<14){
      motor.motor2_backward(motor2_pwm);
    }else if (count1<14 && count2>14){
      motor.motor1_forward(motor1_pwm);
    }else if(count1>14 && count2>14){
      motor.motor1_stop();
      motor.motor2_stop();
      count1 = 0;
      count2 = 0;
      Timer1.attachInterrupt( ISR_timerone );  // Enable the timer
      break;
    }  
  }
};

int pathFinder() {
  if(distance_right>12 && distance_left<20 && distance_front<10) {
    return 2; //RIGHT TURN
  }
  else if(distance_right<20 && distance_left>12) {
    return 1; //LEFT TURN
  }
  else return 0; //NO TURN, STRAIGHT CORRIDOR
};

void goForaWhile() {
  motor1_pwm = 70;
  motor2_pwm = 70;
  motor.motor1_forward(motor1_pwm);
  motor.motor2_forward(motor2_pwm);
  delay(700); //Go straight for 1.5 delayMicroseconds
  count1 = 0;
  count2 = 0;
};

void rotaryReset() {
  lastEncoded = 0;
  encoderValue = 0;
};
