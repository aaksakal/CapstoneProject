#ifndef VAR_H
#define VAR_H

#include <stdint.h>
#include "Arduino.h"
#include <NewPing.h>
#include "pinOut.h"
#include <PID_v1.h>


/*--------------------- UTURN Variables------------------------*/
extern int debug;
extern int leftu;
extern int rightu;
extern int counterEscape;
extern int utimeFlag;
extern int uturn_completed;
extern uint32_t utime;
extern uint32_t unowtime;
extern unsigned int s1Flag;
/*--------------------------------------------------------------*/



/*--------------------- Display Variables------------------------*/
extern int firstTimeDisplay;
/*--------------------------------------------------------------*/

/*--------------------- Rotary Variables------------------------*/
extern volatile int lastEncoded;
extern volatile long encoderValue;
/*--------------------------------------------------------------*/


/*  _____________________ MOTOR VARIABLES _____________________ */
//state names

#define STRAIGHT 0
#define LEFT_TURN 1
#define RIGHT_TURN 2
#define END_OF_ROAD 3
#define SIGNALL 4
#define UTURN 5

//ustate names
#define s0 0
#define s1 1
#define s2 2
#define stp 3


/*_____________________ Joystick Variables _____________________*/
extern int correction_x ;
extern int correction_y ;
extern int y_axis ;
extern int x_axis ;
/*-------------------------------------------------------------*/

//U-turn flag. 1 when a u-turn is detected
extern int left_u_turn;
extern int right_u_turn;

//On the move flag. 1 when the robot is already on the move.
extern int onthemove;

//MOTOR1 = LEFT MOTOR, MOTOR2 = RIGHT MOTOR
//speed values of motors in rpm
extern double motor1_speed;
extern double motor2_speed;
extern double motor1_speed_prev;
extern double motor2_speed_prev;

//Pwm values for motors
extern double motor1_pwm;
extern double motor2_pwm ;

//Desired PWM Value
extern int fixed_pwm;
extern int fixed;
/*  _____________________ END OF MOTOR VARIABLES _____________________ */

/*  _____________________ ENCODER VARIABLES _____________________ */
//Counters for ISRs
extern int count1;
extern int count2;

//It goes into interrupt million times, this flags prevents this.
extern int count1_flag;
extern int count2_flag;

//time measurement until one slot in encoder for motor 1
extern uint32_t time_previous1;
extern uint32_t time_now1;
extern uint32_t time_interval1;

//time measurement until one slot in encoder for motor 2
extern uint32_t time_previous2;
extern uint32_t time_now2;
extern uint32_t time_interval2;
/*  _____________________ END OF ENCODER VARIABLES _____________________ */

/*  _____________________ ULTRASONIC VARIABLES _____________________ */
//Maximum Distance values for Ultrasonic Sensor Measurements
#define MAX_DIST_FRONT 135
#define MAX_DIST_SIDE 135

//Create Sonar objects for NewPing library
extern NewPing sonar_front;
extern NewPing sonar_left;
extern NewPing sonar_right;

//Distance variables
extern double distance_left;
extern double distance_right;
extern double distance_front;
extern double side_error;
extern double side_sum;
extern double desired_distance;

/*  _____________________ END OF ULTRASONIC VARIABLES _____________________ */

//Loop Count
extern int loop_count_delayedstart;

//Ultrasonic PID Controller Parameters
extern double Kp;
extern double Ki;
extern double Kd;

//Values used to calculate the loop duration
extern uint32_t time_previous;
extern uint32_t time_now;
extern uint32_t time_interval;

//PID Controller Object
extern double Output;
extern PID ultra_PID;
//NOTE: It does (DESIRED - INPUT) to calculate OUTPUT. i.e if DESIRED>INPUT then OUTPUT>0
#endif
