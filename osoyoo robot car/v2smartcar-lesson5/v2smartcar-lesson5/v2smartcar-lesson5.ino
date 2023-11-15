#include <Arduino.h>
#include <Servo.h>

#define speedPinR 9 //~ Pin for the speed of the right motors  
#define RightMotorDirPin1  12  //~ Pin for the direction of the right motor 1
#define RightMotorDirPin2  11  //~ Pin for the direction of the right motor 2
#define speedPinL 6 //~ Pin for the speed of the left motors
#define LeftMotorDirPin1  7  //~ Pin for the direction of the left motor 1
#define LeftMotorDirPin2  8  //~ Pin for the direction of the left motor 2 

#define SCAN_LOOP_COUNTER 2 //~ Scan loop counter

#define SERVO_PIN 5 //~ Pin for the servo

#define Echo_PIN 2 //~ Pin for the echo of the ultrasonic sensor
#define Trig_PIN 3 //~ Pin for the trigger of the ultrasonic sensor

#define BUZZ_PIN 13 //~ Pin for the buzzer

#define FAST_SPEED 350  //~ Speed of the motors when going fast
#define SPEED 220       //~ Speed of the motors when going at normal speed
#define TURN_SPEED 300  //~ Speed of the motors when turning
#define BACK_SPEED1 355 //~ Speed of the motors when going backwards
#define BACK_SPEED2 190 //~ Speed of the motors when going backwards

int leftscanval, centerscanval, rightscanval, ldiagonalscanval, rdiagonalscanval; //~ Variables for the distance of the obstacles
const int distancelimit = 3;     // distance limit for obstacles in front //~ Distance limit for obstacles in front
const int sidedistancelimit = 3; // minimum distance in cm to obstacles at both sides (the car will allow a shorter distance sideways) //~ Distance limit for obstacles at the sides
int distance; //~ Variable for the distance of the obstacles
int numcycles = 0; //~ Variable for the number of cycles
const int turntime = 250; // Time the robot spends turning (miliseconds) //~ Time the robot spends turning
const int backtime = 300; // Time the robot spends turning (miliseconds) //~ Time the robot spends going backwards

int thereis; //~ Variable for the number of obstacles
Servo head; //~ Servo for the ultrasonic sensor
//~ Setting RightMotorDirPin1 to HIGH makes BOTH right wheels/motors turn clockwise             RightMotorDirPin1 => HIGH => Clockwise           Setting it LOW turns it off
//~ Setting RightMotorDirPin2 to HIGH makes BOTH right wheels/motors turn anticlockwise         RightMotorDirPin2 => HIGH => Anticlockwise       Setting it LOW turns it off
//~ Setting LeftMotorDirPin1 to HIGH makes BOTH left wheels/motors turn clockwise               LeftMotorDirPin1 => HIGH => Clockwise            Setting it LOW turns it off
//~ Setting LeftMotorDirPin2 to HIGH makes BOTH left wheels/motors turn anticlockwise           LeftMotorDirPin2 => HIGH => Anticlockwise        Setting it LOW turns it off

//& Function to make car go forward
void go_forward() //~ Forward
{
  digitalWrite(RightMotorDirPin1, HIGH); //~ Turns clockwise right motors on
  digitalWrite(RightMotorDirPin2,LOW); //~ Turns anticlockwise right motors off
  digitalWrite(LeftMotorDirPin1,HIGH); //~ Turns clockwise left motors on
  digitalWrite(LeftMotorDirPin2,LOW); //~ Turns anticlockwise left motors off

}

//& Function to make car turn left
void go_Left()  //~ Left
{
  digitalWrite(RightMotorDirPin1, HIGH); //~ Turns clockwise right motors on
  digitalWrite(RightMotorDirPin2,LOW); //~ Turns anticlockwise right motors off
  digitalWrite(LeftMotorDirPin1,LOW); //~ Turns clockwise left motors off
  digitalWrite(LeftMotorDirPin2,HIGH); //~ Turns anticlockwise left motors on

}

//& Function to make car turn right
void go_Right()  //~ Turn right
{
  digitalWrite(RightMotorDirPin1, LOW); //~ Turns clockwise right motors off
  digitalWrite(RightMotorDirPin2,HIGH); //~ Turns anticlockwise right motors on
  digitalWrite(LeftMotorDirPin1,HIGH); //~ Turns clockwise left motors on
  digitalWrite(LeftMotorDirPin2,LOW); //~ Turns anticlockwise left motors off

}

//& Function to make car go backward
void go_Back()  //~ Reverse
{
  digitalWrite(RightMotorDirPin1, LOW); //~ Turns clockwise right motors off
  digitalWrite(RightMotorDirPin2,HIGH); //~ Turns anticlockwise right motors on
  digitalWrite(LeftMotorDirPin1,LOW); //~ Turns clockwise left motors off
  digitalWrite(LeftMotorDirPin2,HIGH); //~ Turns anticlockwise left motors on
}

//& Function to make car stop
void stop_Stop()    //~ Stop
{
  digitalWrite(RightMotorDirPin1, LOW); //~ Turns clockwise right motors off
  digitalWrite(RightMotorDirPin2,LOW); //~ Turns anticlockwise right motors off
  digitalWrite(LeftMotorDirPin1,LOW); //~ Turns clockwise left motors off
  digitalWrite(LeftMotorDirPin2,LOW); //~ Turns anticlockwise left motors off
}

//& Function to set the speed of the motors
void set_Motorspeed(int speed_L,int speed_R)
{
  analogWrite(speedPinL,speed_L); //~ Writes the speed of the left motor to speed_L
  analogWrite(speedPinR,speed_R); //~ Writes the speed of the right motor to speed_R
}

/*detection of ultrasonic distance*/
int watch()
{
  long echo_distance; //~ Value to store echo distance - distance of the object
  digitalWrite(Trig_PIN, LOW); //~ Writes Trig_PIN to LOW/OFF
  delayMicroseconds(5);
  digitalWrite(Trig_PIN, HIGH);//~ Writes Trig_PIN to HIGH/ON
  delayMicroseconds(15);
  digitalWrite(Trig_PIN, LOW);//~ Writes Trig_PIN to LOW/OFF
  echo_distance = pulseIn(Echo_PIN, HIGH); //~ Measures the length (in microseconds) of a pulse on the pin
  echo_distance = echo_distance * 0.01657; //~ how far away is the object in cm
  if (echo_distance < 2) //~ If the distance less than 2
  {
    echo_distance = 0; //~ Set echo_distance to 0
  }
  Serial.println(echo_distance);
  return round(echo_distance);
}

//Meassures distances to the right, left, front, left diagonal, right diagonal and asign them in cm to the variables rightscanval,
//leftscanval, centerscanval, ldiagonalscanval and rdiagonalscanval (there are 5 points for distance testing)
String watchsurrounding()
{
  /*  obstacle_status is a binary integer, its last 5 digits stands for if there is any obstacles in 5 directions,
   *   for example B101000 last 5 digits is 01000, which stands for Left front has obstacle, B100111 means front, right front and right ha
   */

  int obstacle_status = B100000;
  centerscanval = watch();
  if (centerscanval < distancelimit)
  {
    stop_Stop();
    alarm();
    obstacle_status = obstacle_status | B100;
  }
  head.write(120);
  delay(100);
  ldiagonalscanval = watch();
  if (ldiagonalscanval < distancelimit)
  {
    stop_Stop();
    alarm();
    obstacle_status = obstacle_status | B1000;
  }
  head.write(170); // Didn't use 180 degrees because my servo is not able to take this angle
  delay(300);
  leftscanval = watch();
  if (leftscanval < sidedistancelimit)
  {
    stop_Stop();
    alarm();
    obstacle_status = obstacle_status | B10000;
  }

  head.write(90); // use 90 degrees if you are moving your servo through the whole 180 degrees
  delay(100);
  centerscanval = watch();
  if (centerscanval < distancelimit)
  {
    stop_Stop();
    alarm();
    obstacle_status = obstacle_status | B100;
  }
  head.write(40);
  delay(100);
  rdiagonalscanval = watch();
  if (rdiagonalscanval < distancelimit)
  {
    stop_Stop();
    alarm();
    obstacle_status = obstacle_status | B10;
  }
  head.write(0);
  delay(100);
  rightscanval = watch();
  if (rightscanval < sidedistancelimit)
  {
    stop_Stop();
    alarm();
    obstacle_status = obstacle_status | 1;
  }
  head.write(90); // Finish looking around (look forward again)
  delay(300);
  String obstacle_str = String(obstacle_status, BIN);
  obstacle_str = obstacle_str.substring(1, 6);

  return obstacle_str; // return 5-character string standing for 5 direction obstacle status
}


//& Function to initialize the GPIO pins
void init_GPIO()
{

  //~ Init Servo
  head.attach(SERVO_PIN); //~ Attaches the servo to SERVO_PIN
  // head.write(90); //~ Turns Servo 90 degrees - this makes the ultrasonic sensor look forward
  delay(2000);

	pinMode(RightMotorDirPin1, OUTPUT); //~ Sets the pin mode of RightMotorDirPin1 to OUTPUT
	pinMode(RightMotorDirPin2, OUTPUT); //~ Sets the pin mode of RightMotorDirPin2 to OUTPUT
	pinMode(speedPinL, OUTPUT); //~ Sets the pin mode of speedPinL to OUTPUT 
 
	pinMode(LeftMotorDirPin1, OUTPUT); //~ Sets the pin mode of LeftMotorDirPin1 to OUTPUT
  pinMode(LeftMotorDirPin2, OUTPUT); //~ Sets the pin mode of LeftMotorDirPin2 to OUTPUT
  pinMode(speedPinR, OUTPUT); //~ Sets the pin mode of speedPinR to OUTPUT
	stop_Stop();

  //~ Init Ultrasonic Sensor
  pinMode(Trig_PIN, OUTPUT); //~ Sets the pin mode of Trig_PIN to OUTPUT
  pinMode(Echo_PIN, INPUT); //~ Sets the pin mode of Echo_PIN to INPUT
  digitalWrite(Trig_PIN, LOW); //~ Writes Trig_PIN to LOW


  Serial.begin(9600);
}

void setup()
{
	init_GPIO();
  set_Motorspeed(200,200);
	go_forward();//Forward
 
  delay(2000);
  
  go_Back();//Reverse
 
  delay(2000);
  
  go_Left();//Turn left
 
  delay(2000);
  
  go_Right();//Turn right
 
  delay(2000);
  
  stop_Stop();//Stop


}

void loop(){
  // watch();
}