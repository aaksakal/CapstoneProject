#include "variable.h"


/*--------------------- UTURN Variables------------------------*/
int leftu = 0;
int rightu = 0;

int counterEscape = 0;
int utimeFlag = 0;
unsigned int utime = 0;
unsigned int unowtime = 0;
/*--------------------------------------------------------------*/


/*--------------------- Display Variables------------------------*/
int firstTimeDisplay = 0;
/*--------------------------------------------------------------*/

/*--------------------- Rotary Variables------------------------*/
volatile int lastEncoded = 0;
volatile long encoderValue = 0;
/*--------------------------------------------------------------*/

/*_____________________ Joystick Variables _____________________*/
int correction_x = 0;
int correction_y = 0;
int y_axis = 0;
int x_axis = 0;
/*--------------------------------------------------------------*/

//fixed value for the pwm
int fixed = 50;

//U-turn flag. 1 when a u-turn is detected
int left_u_turn=0, right_u_turn;

//On the move flag. 1 when the robot is already on the move.
int onthemove = 0;

//MOTOR1 = LEFT MOTOR, MOTOR2 = RIGHT MOTOR
//speed values of motors in rpm
double motor1_speed = 0;
double motor2_speed = 0;
double motor1_speed_prev = 0;
double motor2_speed_prev = 0;

//Pwm values for motors
double motor1_pwm = 65;
double motor2_pwm = 65;

//Desired PWM Value
int fixed_pwm = fixed;
/*  _____________________ END OF MOTOR VARIABLES _____________________ */

/*  _____________________ ENCODER VARIABLES _____________________ */
//Counters for ISRs
int count1=0;
int count2=0;

//It goes into interrupt million times, this flags prevents this.
int count1_flag = 0;
int count2_flag = 0;

//time measurement until one slot in encoder for motor 1
uint32_t time_previous1 = micros();
uint32_t time_now1 = micros();
uint32_t time_interval1 = time_now1 - time_previous1;

//time measurement until one slot in encoder for motor 2
uint32_t time_previous2 = micros();
uint32_t time_now2 = micros();
uint32_t time_interval2 = time_now2 - time_previous2;
/*  _____________________ END OF ENCODER VARIABLES _____________________ */

/*  _____________________ ULTRASONIC VARIABLES _____________________ */

//Create Sonar objects for NewPing library
NewPing sonar_front = NewPing(front_trig, front_echo, MAX_DIST_FRONT);
NewPing sonar_left = NewPing(left_trig, left_echo, MAX_DIST_SIDE);
NewPing sonar_right = NewPing(right_trig, right_echo, MAX_DIST_SIDE);

//Distance variables
double distance_left=0;
double distance_right=0;
double distance_front=135;
double side_error;
double side_sum;
double desired_distance=0;

/*  _____________________ END OF ULTRASONIC VARIABLES _____________________ */

//Loop Count
int loop_count_delayedstart =0;

//Kp for now
double Kp=2.4, Ki=0.15, Kd=1.7;

//Values used to calculate the loop duration
uint32_t time_previous = 0;
uint32_t time_now = 0;
uint32_t time_interval = 0;

//PID Controller Object
double Output=0;
PID ultra_PID(&side_error, &Output, &desired_distance, Kp, Ki, Kd, DIRECT);
//NOTE: It does (DESIRED - INPUT) to calculate OUTPUT. i.e if DESIRED>INPUT then OUTPUT>0

