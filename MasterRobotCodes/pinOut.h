#ifndef pinOut_h
#define pinOut_h

/*  _____________________ PIN DEFINITIONS ____________________ */
//Encoder Pins
#define MOTOR_LEFT  2  // Motor left Interrupt Pin - INT 0 //left
#define MOTOR_RIGHT 3  // Motor right Interrupt Pin - INT 1 //right

//Motor Driver Control Pins
#define in1 7
#define in2 8
#define in3 9
#define in4 10

//Motor Driver Enable Pins
#define enA 5
#define enB 6

//Ultrasonic Sensor Pins
#define front_echo 11
#define front_trig 12
#define left_echo 18
#define left_trig 19
#define right_echo 16
#define right_trig 17

//Joystick Module Pins
#define joystick_X 14
#define joystick_Y 15

//Rotary Sensor Pins
#define rotary1 4
#define rotary2 13
/*  _____________________ END OF PIN DEFINITIONS ____________________ */

class pinOutClass
{
  public:
    pinOutClass();
    void SETUP(){
      //Select Pin Modes
  //OUTPUT Pins
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(front_trig, OUTPUT);
  pinMode(left_trig,  OUTPUT);
  pinMode(right_trig, OUTPUT);

  //INPUT Pins
  pinMode(front_echo,  INPUT);
  pinMode(left_echo,   INPUT);
  pinMode(right_echo,  INPUT);
  pinMode(joystick_X,  INPUT);
  pinMode(joystick_Y,  INPUT);
  pinMode(MOTOR_LEFT,  INPUT);
  pinMode(MOTOR_RIGHT, INPUT);
  pinMode(rotary1, INPUT_PULLUP);
  pinMode(rotary2, INPUT_PULLUP);
 };
};

extern pinOutClass pinOut;


#endif //pinOut_h
