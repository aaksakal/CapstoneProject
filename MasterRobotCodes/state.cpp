#include "Arduino.h"
#include "motor.h"
#include "state.h"
#include "variable.h"
#include "isr.h"
#include "masterUFunc.h"

int ustate = s0;

void Straight(int& state){
  //Start the timer interrupt. It will go to ISR when time's up.
  Timer1.attachInterrupt( ISR_timerone );
      loop_count_delayedstart++;
      //Gather Data from right-left and front sensors.
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
        side_error = 7.5 - distance_left; //Go with only the left sensor.
        if(distance_front<11) {
          //If we got close enough, commence RIGHT_TURN.
          motor.motor1_stop();
          motor.motor2_stop();
          delay(500);
          state = RIGHT_TURN;
          ultra_PID.SetTunings(0,0,0); //Set the PID values to 0 to stop cumulative error while turning.
          Timer1.detachInterrupt(); //Stop timer interrupt. It will disturb the turning process.
          count1 = 0; //Reset the count numbers. We will turn according to these.
          count2 = 0;
          return;
        }
      }
      else if(distance_right<20 && distance_left>12) {
        //This means a left turn is approaching.
        side_error = distance_right - 7.5;
        //Try to move straight with only 1 sensor. Until we get close enough.
        if(distance_front<11) {
          //If we got close enough, commence LEFT_TURN.
          motor.motor1_stop();
          motor.motor2_stop();
          delay(500);
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
      //Compute PID controller with the side_error data.
      //PID was initiated with this data, so we don't need to feed it to PID again.
      //We just call the Compute method to initiate computing.
      ultra_PID.Compute();
      motor1_pwm = fixed_pwm - Output;
      motor2_pwm = fixed_pwm + Output;
//      if(loop_count_delayedstart < 23) {
//        motor1_stop();
//        motor2_stop();
//      }
//      else {
        motor.motor1_forward(motor1_pwm);
        motor.motor2_forward(motor2_pwm);
//      }
      delay(20); //Delay the loops 20ms each iteration. Otherwise it goes berzerk.
      return ;
};
void LeftTurn(int& state){
  //Uses only one wheel to turn. It moves the wheel with respect to the counters from encoder.
     //In this manner we go through a perfect 90 degrees turn.
     right_u_turn=0; // Reset the right_u turn flag since no successive turns are encountered.
     motor1_pwm=0;
     motor2_pwm=75;
     if(count2<14) {
       motor.motor1_stop();
       motor.motor2_forward(motor2_pwm);
     }
     //If the turn is completed stop for 0.75sec and decide on the state.
     else {
       motor.motor1_stop();
       motor.motor2_stop();
       delay(100);
       //If there was a previous left_turn just now, change state to signal. This means we are in a U-turn.
       if(left_u_turn==1) {
         state = SIGNAL;
       }
       //Otherwise go back on moving straight again.
       else {
         state = STRAIGHT;
         ultra_PID.SetTunings(Kp,Ki,Kd);
         count1=0;
         count2=0;
         Timer1.attachInterrupt( ISR_timerone );
         motor1_pwm=68;
         motor2_pwm=80;
         onthemove = 0;
         motor.motor1_forward(motor1_pwm);
         motor.motor2_forward(motor1_pwm);
         delay(100);
         distance_front = sonar_front.ping_cm();
         if(distance_front==0) {
           distance_front = 135;
         }
         if(distance_front<40) {
           left_u_turn = 1;
         }
       }
     }
     return;
};
void RightTurn(int& state){
  left_u_turn=0;
      motor1_pwm=75;
      motor2_pwm=0;
      if(count1<16) {
        motor.motor1_forward(motor1_pwm);
        motor.motor2_stop();
      }
      else {
        motor.motor1_stop();
        motor.motor2_stop();
        delay(100);
        if(right_u_turn==1) {
          state = SIGNAL;
        }
        else {
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
          delay(100);
          distance_front = sonar_front.ping_cm();
          if(distance_front==0) {
            distance_front = 135;
          }
          if(distance_front<40) {
            right_u_turn = 1;
          }
        }
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
  
  ustate = s0;
  state = UTURN;
  
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

void Stop(int& state){
    motor.motor1_stop();
    motor.motor2_stop();
    delay(1000);
    state = STRAIGHT;
    return;
};

void Uturn(int& state)
{
  updateRotaryEncoder();
  
  
  switch(ustate)
  {
    case s0: {

        //goForaWhile();
        
        while(1) {
            sonarGatherData();
            if(distance_front>10) //  no escape path
            {
               straightWithPID();
             }
            else  //  escape path found
            {       displaying(ustate);
                    if(distance_right>15) 
                    {
                    rightu = 1;   
                    ustate = s1;
                    }if (distance_left>15)
                    {
                    rightu = 0;
                    ustate = s1;
                    counterEscape = 0;
                    }
             break;
            }   
            //ustate = s1;
        }
        count1 = 0; //Reset the count numbers. We will turn according to these.
        count2 = 0;
        break;
    }
    case s1: {

        motor.motor1_stop();
        motor.motor2_stop();
        delay(100);
        Serial.println("lol");
        if(rightu) {
            Serial.println("xd");
            right90();
        }
        else {
          Serial.println("xf");
            left90();
        }
        while(1) {
          mappingJoystick();
          Serial.print(y_axis);
          Serial.print(" || ");
          Serial.println(x_axis);
            if(y_axis < 250) {  //  <-----------WARNING
                ustate = s2;
                break;
            }
            else {
                straightWithJoystick(); //  <-----------WARNING
            }
        }
        break;
        
    }
    case s2: {
        count1 = 0; //Reset the count numbers. We will turn according to these.
        count2 = 0;
        turnBack();
        Serial.println("------------------------------döndü");
        while(1){
          sonarGatherData();
          Serial.println(distance_front);
          if(distance_front>7)
          {
             straightWithPID();
             Serial.println("anan");
          }
          else  
          {    
            Serial.println("baban");
             if(rightu) 
            {
               left90();
            }else
            {
              right90();
            }
            ustate = s3;
            break;
          }   
        }
        break;
    }
    case s3: {
        mappingJoystick();
        sonarGatherData();
        if(y_axis >= 477) {
            straightWithJoystick(); //  <-----------WARNING
            utimeFlag = 0;
        }
        else if(y_axis <= 450) {
           // backWithJoystick(); //  <-----------WARNING
          motor.motor1_stop();
          motor.motor2_stop();
          if(utimeFlag == 0){
            utime = millis();
            utimeFlag = 1;
            Serial.println(utime);
          }else {
            unowtime = millis();
            Serial.println("**********************************************************");
            Serial.println(unowtime - utime);
            if((unowtime - utime) >= 10000){
              turnBack();
              state = STRAIGHT;
            }
            
          }   
        }

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





void mappingJoystick(){
  y_axis = analogRead(joystick_Y);
  x_axis = analogRead(joystick_X);
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


















void displaying(int& state){
  Serial.print("distance_left: ");
  Serial.println(distance_left);
  Serial.print("distance_right: ");
  Serial.println(distance_right);
//  Serial.print("distance_front: ");
//  Serial.println(distance_front);
//  Serial.print("motor1_pwm: ");
//  Serial.println(motor1_pwm);
//  Serial.print("motor2_pwm: ");
//  Serial.println(motor2_pwm);
   Serial.println(state);
//  Serial.println("__________");
//  Serial.println(distance_front);
};

void ultrasonicDeneme(){

  distance_front = sonar_front.ping_cm();
  delayMicroseconds(100);
  distance_left = sonar_left.ping_cm();
  delayMicroseconds(100);
  distance_right = sonar_right.ping_cm();
  delayMicroseconds(100);
};

