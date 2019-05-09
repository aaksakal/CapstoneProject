#include "Arduino.h"
#include "motor.h"
#include "state.h"
#include "variable.h"
#include "isr.h"
#include "slaveUFunc.h"
//#include <EnableInterrupt.h>

int ustate = s0;


void Straight(int& state){
  if(debug == 1)
    Serial.println("debug");
  Timer1.attachInterrupt( ISR_timerone );
      loop_count_delayedstart++;
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
       
      if(distance_right>12 && distance_left<20) {
        //A right turn is approaching
        side_error = 7.5 - distance_left;
        if(distance_front<10) {
          //If we got close enough, commence LEFT_TURN.
          motor.motor1_stop();
          motor.motor2_stop();
          delay(50);
          state = RIGHT_TURN;
          ultra_PID.SetTunings(0,0,0);
          Timer1.detachInterrupt();
          count1 = 0;
          count2 = 0;
          return;
        }
      }
      else if(distance_right<20 && distance_left>12) {
        //This means a left turn is approaching.
        side_error = distance_right - 7.5;
        //Try to move straight with only 1 sensor. Until we get close enough.
        if(distance_front<10) {
          //If we got close enough, commence LEFT_TURN.
          motor.motor1_stop();
          motor.motor2_stop();
          delay(50);
          state = LEFT_TURN;
          ultra_PID.SetTunings(0,0,0);
          Timer1.detachInterrupt();
          count1 = 0;
          count2 = 0;
          return;
        }
      }
      else {
        //If non of these are applicable then keep moving straight with both sensors.
        side_error = distance_right - distance_left;
      }
      //Compute PID controller with the distance data
      ultra_PID.Compute();
      motor1_pwm = fixed_pwm - Output;
      motor2_pwm = fixed_pwm + Output;

      pwmCheck(motor1_pwm);
      pwmCheck(motor2_pwm);
      

      motor.motor1_forward(motor1_pwm);
      motor.motor2_forward(motor2_pwm);

     
      Serial.println(abs(encoderValue));
      if(abs(encoderValue) > 4 && y_axis < 460) {
          utimeFlag = 1;

      }
      else if(y_axis > 480 )  {
          utimeFlag = 0;
      }
    if(uturn_completed==0) {
      if(utimeFlag == 0) {
            utime = millis();
       }
      else {
              unowtime = millis();
              if((unowtime - utime) >= 2000){
                    utimeFlag = 0;
                    ustate = s0;
                    state = UTURN;  
               }
          
      }
    }

      delay(20);
      return ;
};
void LeftTurn(int& state){
  //Uses only one wheel to turn. It moves the wheel with respect to the counters from encoder.
     //In this manner we go through a perfect 90 degrees turn.
     right_u_turn=0; // Reset the right_u turn flag since no successive turns are encountered.
     motor1_pwm=0;
     motor2_pwm=70;
     if(count2<14) {
       motor.motor1_stop();
       motor.motor2_forward(motor2_pwm);
     }
     //If the turn is completed stop for 0.75sec and decide on the state.
     else {
       motor.motor1_stop();
       motor.motor2_stop();
       //delay(750);
       //If there was a previous left_turn just now, change state to signal.
         state = STRAIGHT;
         ultra_PID.SetTunings(Kp,Ki,Kd);
         count1=0;
         count2=0;
         Timer1.attachInterrupt( ISR_timerone );
         motor1_pwm=68;
         motor2_pwm=68;
         onthemove = 0;
         motor.motor1_forward(motor1_pwm);
         motor.motor2_forward(motor1_pwm);
         delay(100);
         distance_front = sonar_front.ping_cm();
         if(distance_front==0) {
           distance_front = 135;
         }
         rotaryReset();
       }
     return;
};
void RightTurn(int& state){
  left_u_turn=0;
      motor1_pwm=70;
      motor2_pwm=0;
      if(count1<14) {
        motor.motor1_forward(motor1_pwm);
        motor.motor2_stop();
      }
      else {
        motor.motor1_stop();
        motor.motor2_stop();
        //delay(750);
          state = STRAIGHT;
          ultra_PID.SetTunings(Kp,Ki,Kd);
          count1=0;
          count2=0;
          Timer1.attachInterrupt( ISR_timerone );
          motor1_pwm=68;
          motor2_pwm=68;
          onthemove = 0;
          motor.motor1_forward(motor1_pwm);
          motor.motor2_forward(motor2_pwm);
          delay(100);
          distance_front = sonar_front.ping_cm();
          if(distance_front==0) {
            distance_front = 135;
          }
          rotaryReset();
        }
      return ;
};
void Signal(int& state){
    left_u_turn=0;
    right_u_turn=0;
    motor.motor1_stop();
    motor.motor2_stop();
    delay(10000);
    distance_front = sonar_front.ping_cm();
    if(distance_front==0) {
    distance_front = 135;
    }
    state = STRAIGHT;
    ultra_PID.SetTunings(Kp,Ki,Kd);
    count1=0;
    count2=0;
    Timer1.attachInterrupt( ISR_timerone );
    motor1_pwm=68;
    motor2_pwm=80;
    onthemove = 0;
    motor.motor1_forward(motor1_pwm);
    motor.motor2_forward(motor2_pwm);
    return;
};

void Uturn(int& state)
{
  updateRotaryEncoder();
  //displaying(state);
  
  switch(ustate)
  {
    case s0: {

        count1 = 0; //Reset the count numbers. We will turn according to these.
        count2 = 0;
        turnBack();
        Serial.println("------------------döndü-s0------------------");
        
        while(1) {
            umappingJoystick();
            if(y_axis>477 || abs(527-x_axis) > 30) 
            {
                ustate = s1;
                break;
            }
            else  //  escape path found
            {
                motor.motor1_stop();
                motor.motor2_stop();      
             
            }              
        }
        break;
    }
    case s1: {

        umappingJoystick();

          if(y_axis <= 420) {
              s1Flag = 1;
    
          }
          else {
              s1Flag = 0;
          }
    
          if(s1Flag == 0) {
                utime = millis();
                sonarGatherData();
                straightWithJoystick2();
           }
          else {
              unowtime = millis();
              if((unowtime - utime) >= 1000){
                    s1Flag = 0;
                    ustate = s2; 
               }
          
           } 
        break;
        
    }
    case s2: {
        delay(1000);
        count1 = 0; //Reset the count numbers. We will turn according to these.
        count2 = 0;
        turnBack();
        
        while(1){
          sonarGatherData();
          

          if(distance_right>15 && distance_left<20) {
            //A right turn is approaching
            if(distance_front<10) {
              Serial.println("lolxd");
              //If we got close enough, commence LEFT_TURN.
              motor.motor1_stop();
              motor.motor2_stop();
              delay(50);
              right90();

              motor.motor1_stop();
             motor.motor2_stop();
             delay(8000);
             goForaWhile();
             debug = 1;
             utimeFlag = 0;
             state = STRAIGHT;
             break;
              
            }
          }
          else if(distance_right<20 && distance_left>15) {
            //This means a left turn is approaching.
            //Try to move straight with only 1 sensor. Until we get close enough.
            if(distance_front<10) {
              Serial.println("lolxdzaaaaa");
              //If we got close enough, commence LEFT_TURN.
              motor.motor1_stop();
              motor.motor2_stop();
              delay(50);
              left90();
              Serial.println("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxs");
              motor.motor1_stop();
              motor.motor2_stop();
              delay(8000);
              goForaWhile();
              debug = 1;
              utimeFlag = 0;
              state = STRAIGHT;
              break;
              
          }
          }
              straightWithPID();
        }
        uturn_completed = 1;
        break;
    }
    case stp: {
      motor.motor1_stop();
      motor.motor2_stop();
      ustate = stp;

      break;
    }
    
  }
}

void umappingJoystick(){
  y_axis = analogRead(joystick_Y);
  x_axis = analogRead(joystick_X);
};



void mappingJoystick(){
  y_axis = analogRead(joystick_Y);
  x_axis = analogRead(joystick_X);
  //_____________MAPPING START_____________
  if(y_axis <= 450){
    correction_y = map(y_axis, 0, 450, -20, 0);
  }else if(y_axis >= 451 && y_axis <= 479){
    correction_y = map(y_axis, 451, 480, 0, 0);
  }else if(y_axis >= 480 && y_axis <= 750){
    correction_y = map(y_axis, 480, 750, 0, 2);
  }else if(y_axis >750){
    correction_y = map(y_axis, 750, 1023, 2, 4);
  }else {
    correction_y = 0;
  }
 //_______________MAPPING END_________________
 
  if(motor1_speed <= 15 && motor2_speed <= 15 && y_axis >= 465 && fixed_pwm < 50)
    fixed_pwm = 40;
  if(abs(encoderValue) > 4)
    fixed_pwm = 45;
  else
    fixed_pwm = fixed_pwm + correction_y;
  
  if(fixed_pwm > 75)
    fixed_pwm = 75;
  else if (fixed_pwm <= 0)
    fixed_pwm = 0;
};

void pwmCheck(double &pwmValue){
  if(pwmValue < MINPWM){
    pwmValue = MINPWM;
  }else if(pwmValue > MAXPWM){
    pwmValue = MAXPWM;
  }
};

void displaying(int& state){
  //Serial.println(distance_front);
  Serial.print(distance_right);
  Serial.print( " || ");
  Serial.println(distance_left);
//  if(firstTimeDisplay == 0){
//    Serial.print("fixed   |");
//    Serial.print("Y_axis  |");
//    Serial.print("X_axis  |");
//    Serial.print("State  |");
//    Serial.print("correction_y  |");
//    Serial.println(encoderValue);
//    firstTimeDisplay = 1;
//  }else{
//    Serial.print(fixed_pwm);
//    Serial.print("   || ");
//   Serial.print(y_axis);
//    Serial.print("   || ");
//    Serial.print(x_axis);
//    Serial.print("   || ");
//    Serial.print(correction_y);
//    Serial.print("   || ");
//    Serial.print(encoderValue);
//  }
};

void SetupPid(){
  ultra_PID.SetMode(AUTOMATIC);
  ultra_PID.SetSampleTime(30);
  ultra_PID.SetOutputLimits(-25,25);
};

void SetupInterrupts(){
  Timer1.initialize(200000); // set timer for 0.2sec
  attachInterrupt(0, ISR_motor1, CHANGE);  // Go to ISR_motor1 when speed sensor pin goes High
  attachInterrupt(1, ISR_motor2, CHANGE);  // go to ISR_motor2 when speed sensor pin goes High

  Timer1.attachInterrupt( ISR_timerone ); // Enable the timer
};

void SetupMotors(){
  motor.motor1_forward(motor1_pwm);
  motor.motor2_forward(motor2_pwm);
};

void SetupRotary(){ 
  digitalWrite(rotary1, HIGH);
  digitalWrite(rotary2, HIGH);
  
//  enableInterrupt(rotary1, updateRotaryEncoder, CHANGE);
//  enableInterrupt(rotary2, updateRotaryEncoder, CHANGE);
};

void updateRotaryEncoder(){
  
  int MSB = digitalRead(rotary1); //MSB = most significant bit
  int LSB = digitalRead(rotary2); //LSB = least significant bit
  
  int encoded = (MSB << 1) |LSB;
  //converting the 2 pin value to single number 
  int sum = (lastEncoded << 2) | encoded;
  //adding it to the previous encoded value 
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) 
  encoderValue ++; 
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) 
  encoderValue --; 
  lastEncoded = encoded; 

  //store this value for next time 
};

