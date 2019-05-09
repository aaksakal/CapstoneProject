#include "Arduino.h"
#include "motor.h"
#include "state.h"
#include "variable.h"
#include "isr.h"


/*  _____________________ INTERRUPT SERVICE SUBROUTINES _____________________ */
// Motor 1 pulse count ISR
void ISR_motor1()
{
  detachInterrupt(0);
      if(count1_flag==1 && digitalRead(MOTOR_LEFT)==HIGH) {
        time_now1 = micros();
        time_interval1 = time_now1 - time_previous1;
        motor1_speed = ((3*1000000)/time_interval1); //speed in rpm
        if(motor1_speed > 60)
          motor1_speed = motor1_speed_prev;
        if(motor1_speed_prev == 0){
          motor1_speed_prev = motor1_speed;
        }else {
          motor1_speed = (motor1_speed + motor1_speed_prev) / 2;
          motor1_speed_prev = motor1_speed;
        }
        time_previous1 = time_now1;
        count1++;
        count1_flag=0;
      }
      else if (count1_flag==0 && digitalRead(MOTOR_LEFT)==LOW) {
        count1_flag=1;
      }
   attachInterrupt(0, ISR_motor1, RISING);
};

// Motor 2 pulse count ISR
void ISR_motor2()
{
  detachInterrupt(1);
  if(count2_flag==1 && digitalRead(MOTOR_RIGHT)==HIGH) {
        time_now2 = micros();
        time_interval2 = time_now2 - time_previous2;
        motor2_speed = ((3*1000000)/time_interval2); //speed in rpm
        if(motor2_speed > 60)
          motor2_speed = motor2_speed_prev;
        if(motor2_speed_prev == 0){
          motor2_speed_prev = motor2_speed;
        }else {
          motor2_speed = (motor2_speed + motor2_speed_prev) / 2;
          motor2_speed_prev = motor2_speed;
        }
        time_previous2 = time_now2;
        count2++;
        count2_flag=0;
      }
      else if (count2_flag==0 && digitalRead(MOTOR_RIGHT)==LOW) {
        count2_flag=1;
      }
  attachInterrupt(1, ISR_motor2, CHANGE);
};

// TimerOne ISR
void ISR_timerone()
{
  Timer1.detachInterrupt();  // Stop the timer
  detachInterrupt(0);
  detachInterrupt(1);
  if(count1 == 0)
    motor1_speed = 0;
  else
    motor1_speed = ((5 * count1) + motor1_speed)/2;
  if(count2 == 0)
    motor2_speed = 0;
  else
    motor2_speed = ((5 * count2) + motor2_speed)/2;

  motor1_speed_prev = motor1_speed;
  motor2_speed_prev = motor2_speed;
  count1 = 0;
  count2 = 0;
  if(motor1_speed>6 || motor2_speed>6) {
    onthemove = 1;
  }
  if((motor1_speed < 15 || motor2_speed < 15) && (onthemove==1)) {
    fixed_pwm = fixed_pwm + 5;
  }
  else {
    fixed_pwm = fixed;
  }
  attachInterrupt(0, ISR_motor1, CHANGE);  // Go to ISR_motor1 when speed sensor pin goes High
  attachInterrupt(1, ISR_motor2, CHANGE);
  Timer1.attachInterrupt( ISR_timerone );  // Enable the timer
};
/*  _____________________END OF INTERRUPT SERVICE SUBROUTINES _____________________ */

