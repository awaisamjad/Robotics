// #include <Arduino.h>
// #include <Servo.h>

// #define speedPinR 10         //~ Pin for the speed of the right motors
// #define RightMotorDirPin1 12 //~ Pin for the direction of the right motor 1
// #define RightMotorDirPin2 11 //~ Pin for the direction of the right motor 2
// #define speedPinL 6          //~ Pin for the speed of the left motors
// #define LeftMotorDirPin1 7   //~ Pin for the direction of the left motor 1
// #define LeftMotorDirPin2 8   //~ Pin for the direction of the left motor 2
// #define WHEEL_DIAMETER 66   //~ Diameter of the wheels in cm
// #define SCAN_LOOP_COUNTER 2 //~ Scan loop counter

// #define SERVO_PIN 9 //~ Pin for the servo

// #define Echo_PIN 2 //~ Pin for the echo of the ultrasonic sensor
// #define Trig_PIN 3 //~ Pin for the trigger of the ultrasonic sensor

// #define BUZZ_PIN 13 //~ Pin for the buzzer

// #define FAST_SPEED 350  //~ Speed of the motors when going fast
// #define SPEED 220       //~ Speed of the motors when going at normal speed
// #define TURN_SPEED 300  //~ Speed of the motors when turning
// #define BACK_SPEED1 355 //~ Speed of the motors when going backwards
// #define BACK_SPEED2 190 //~ Speed of the motors when going backwards

// // int leftscanval, centerscanval, rightscanval, ldiagonalscanval, rdiagonalscanval; //~ Variables for the distance of the obstacles
// // const int distancelimit = 3;     // distance limit for obstacles in front //~ Distance limit for obstacles in front
// // const int sidedistancelimit = 3; // minimum distance in cm to obstacles at both sides (the car will allow a shorter distance sideways) //~ Distance limit for obstacles at the sides
// // int distance; //~ Variable for the distance of the obstacles
// // int numcycles = 0; //~ Variable for the number of cycles
// // const int turntime = 250; // Time the robot spends turning (miliseconds) //~ Time the robot spends turning
// // const int backtime = 300; // Time the robot spends turning (miliseconds) //~ Time the robot spends going backwards

// int thereis; //~ Variable for the number of obstacles
// Servo head;  //~ Servo for the ultrasonic sensor
// //~ Setting RightMotorDirPin1 to HIGH makes BOTH right wheels/motors turn clockwise             RightMotorDirPin1 => HIGH => Clockwise           Setting it LOW turns it off
// //~ Setting RightMotorDirPin2 to HIGH makes BOTH right wheels/motors turn anticlockwise         RightMotorDirPin2 => HIGH => Anticlockwise       Setting it LOW turns it off
// //~ Setting LeftMotorDirPin1 to HIGH makes BOTH left wheels/motors turn clockwise               LeftMotorDirPin1 => HIGH => Clockwise            Setting it LOW turns it off
// //~ Setting LeftMotorDirPin2 to HIGH makes BOTH left wheels/motors turn anticlockwise           LeftMotorDirPin2 => HIGH => Anticlockwise        Setting it LOW turns it off

// //& Function to make car go forward
// void go_forward() //~ Forward
// {
//   digitalWrite(RightMotorDirPin1, HIGH); //~ Turns clockwise right motors on
//   digitalWrite(RightMotorDirPin2, LOW);  //~ Turns anticlockwise right motors off
//   digitalWrite(LeftMotorDirPin1, HIGH);  //~ Turns clockwise left motors on
//   digitalWrite(LeftMotorDirPin2, LOW);   //~ Turns anticlockwise left motors off
// }

// //& Function to make car turn left
// void go_Left() //~ Left
// {
//   digitalWrite(RightMotorDirPin1, HIGH); //~ Turns clockwise right motors on
//   digitalWrite(RightMotorDirPin2, LOW);  //~ Turns anticlockwise right motors off
//   digitalWrite(LeftMotorDirPin1, LOW);   //~ Turns clockwise left motors off
//   digitalWrite(LeftMotorDirPin2, HIGH);  //~ Turns anticlockwise left motors on
// }

// //& Function to make car turn right
// void go_Right() //~ Turn right
// {
//   digitalWrite(RightMotorDirPin1, LOW);  //~ Turns clockwise right motors off
//   digitalWrite(RightMotorDirPin2, HIGH); //~ Turns anticlockwise right motors on
//   digitalWrite(LeftMotorDirPin1, HIGH);  //~ Turns clockwise left motors on
//   digitalWrite(LeftMotorDirPin2, LOW);   //~ Turns anticlockwise left motors off
// }

// //& Function to make car go backward
// void go_Back() //~ Reverse
// {
//   digitalWrite(RightMotorDirPin1, LOW);  //~ Turns clockwise right motors off
//   digitalWrite(RightMotorDirPin2, HIGH); //~ Turns anticlockwise right motors on
//   digitalWrite(LeftMotorDirPin1, LOW);   //~ Turns clockwise left motors off
//   digitalWrite(LeftMotorDirPin2, HIGH);  //~ Turns anticlockwise left motors on
// }

// //& Function to make car stop
// void stop_Stop() //~ Stop
// {
//   digitalWrite(RightMotorDirPin1, LOW); //~ Turns clockwise right motors off
//   digitalWrite(RightMotorDirPin2, LOW); //~ Turns anticlockwise right motors off
//   digitalWrite(LeftMotorDirPin1, LOW);  //~ Turns clockwise left motors off
//   digitalWrite(LeftMotorDirPin2, LOW);  //~ Turns anticlockwise left motors off
// }

// //& Function to set the speed of the motors
// void set_Motorspeed(int speed_L, int speed_R)
// {
//   analogWrite(speedPinL, speed_L); //~ Writes the speed of the left motor to speed_L
//   analogWrite(speedPinR, speed_R); //~ Writes the speed of the right motor to speed_R
// }

// /*detection of ultrasonic distance*/
// // int watch()
// // {
// //   long echo_distance; //~ Value to store echo distance - distance of the object
// //   digitalWrite(Trig_PIN, LOW); //~ Writes Trig_PIN to LOW/OFF
// //   delayMicroseconds(5);
// //   digitalWrite(Trig_PIN, HIGH);//~ Writes Trig_PIN to HIGH/ON
// //   delayMicroseconds(15);
// //   digitalWrite(Trig_PIN, LOW);//~ Writes Trig_PIN to LOW/OFF
// //   echo_distance = pulseIn(Echo_PIN, HIGH); //~ Measures the length (in microseconds) of a pulse on the pin
// //   echo_distance = echo_distance * 0.01657; //~ how far away is the object in cm
// //   if (echo_distance < 2) //~ If the distance less than 2
// //   {
// //     echo_distance = 0; //~ Set echo_distance to 0
// //   }
// //   Serial.println(echo_distance);
// //   return round(echo_distance);
// // }

// // Meassures distances to the right, left, front, left diagonal, right diagonal and asign them in cm to the variables rightscanval,
// // leftscanval, centerscanval, ldiagonalscanval and rdiagonalscanval (there are 5 points for distance testing)
// // String watchsurrounding()
// // {
// //   /*  obstacle_status is a binary integer, its last 5 digits stands for if there is any obstacles in 5 directions,
// //    *   for example B101000 last 5 digits is 01000, which stands for Left front has obstacle, B100111 means front, right front and right ha
// //    */

// //   int obstacle_status = B100000;
// //   centerscanval = watch();
// //   if (centerscanval < distancelimit)
// //   {
// //     stop_Stop();
// //     alarm();
// //     obstacle_status = obstacle_status | B100;
// //   }
// //   head.write(120);
// //   delay(100);
// //   ldiagonalscanval = watch();
// //   if (ldiagonalscanval < distancelimit)
// //   {
// //     stop_Stop();
// //     alarm();
// //     obstacle_status = obstacle_status | B1000;
// //   }
// //   head.write(170); // Didn't use 180 degrees because my servo is not able to take this angle
// //   delay(300);
// //   leftscanval = watch();
// //   if (leftscanval < sidedistancelimit)
// //   {
// //     stop_Stop();
// //     alarm();
// //     obstacle_status = obstacle_status | B10000;
// //   }

// //   head.write(90); // use 90 degrees if you are moving your servo through the whole 180 degrees
// //   delay(100);
// //   centerscanval = watch();
// //   if (centerscanval < distancelimit)
// //   {
// //     stop_Stop();
// //     alarm();
// //     obstacle_status = obstacle_status | B100;
// //   }
// //   head.write(40);
// //   delay(100);
// //   rdiagonalscanval = watch();
// //   if (rdiagonalscanval < distancelimit)
// //   {
// //     stop_Stop();
// //     alarm();
// //     obstacle_status = obstacle_status | B10;
// //   }
// //   head.write(0);
// //   delay(100);
// //   rightscanval = watch();
// //   if (rightscanval < sidedistancelimit)
// //   {
// //     stop_Stop();
// //     alarm();
// //     obstacle_status = obstacle_status | 1;
// //   }
// //   head.write(90); // Finish looking around (look forward again)
// //   delay(300);
// //   String obstacle_str = String(obstacle_status, BIN);
// //   obstacle_str = obstacle_str.substring(1, 6);

// //   return obstacle_str; // return 5-character string standing for 5 direction obstacle status
// // }

// //& Function to initialize the GPIO pins
// void init_GPIO()
// {

//   //~ Init Servo
//   // head.attach(SERVO_PIN); //~ Attaches the servo to SERVO_PIN
//   // head.write(90); //~ Turns Servo 90 degrees - this makes the ultrasonic sensor look forward
//   delay(2000);

//   pinMode(RightMotorDirPin1, OUTPUT); //~ Sets the pin mode of RightMotorDirPin1 to OUTPUT
//   pinMode(RightMotorDirPin2, OUTPUT); //~ Sets the pin mode of RightMotorDirPin2 to OUTPUT
//   pinMode(speedPinL, OUTPUT);         //~ Sets the pin mode of speedPinL to OUTPUT

//   pinMode(LeftMotorDirPin1, OUTPUT); //~ Sets the pin mode of LeftMotorDirPin1 to OUTPUT
//   pinMode(LeftMotorDirPin2, OUTPUT); //~ Sets the pin mode of LeftMotorDirPin2 to OUTPUT
//   pinMode(speedPinR, OUTPUT);        //~ Sets the pin mode of speedPinR to OUTPUT
//   stop_Stop();

//   //~ Init Ultrasonic Sensor
//   pinMode(Trig_PIN, OUTPUT);   //~ Sets the pin mode of Trig_PIN to OUTPUT
//   pinMode(Echo_PIN, INPUT);    //~ Sets the pin mode of Echo_PIN to INPUT
//   digitalWrite(Trig_PIN, LOW); //~ Writes Trig_PIN to LOW
//   Serial.begin(9600);
// }

// float calculateLinearSpeed(int speed_L, int speed_R) {
//   // Calculate linear speed in centimeters per second
//   float linearSpeed = (WHEEL_DIAMETER * PI * (speed_L + speed_R)) / (2.0 * 255.0);

//   Serial.print("Linear Speed: ");
//   Serial.print(linearSpeed);
//   Serial.println(" centimeters per second");
//   return linearSpeed;
// }

// unsigned long startTime = 0;

// // void calculateLinearDistance(int speed_L, int speed_R) {
// //   // Calculate elapsed time
// //   unsigned long currentTime = millis();
// //   unsigned long elapsedTime = currentTime - startTime;

// //   // Calculate linear distance in centimeters
// //   float linearSpeed = calculateLinearSpeed(speed_L, speed_R);
// //   float linearDistanceCM = linearSpeed * (elapsedTime / 1000.0);

// //   Serial.print("Linear Distance: ");
// //   Serial.print(linearDistanceCM);
// //   Serial.println(" centimeters");
// // }

// void setup()
// {
//   init_GPIO();
//   startTime = millis();
//   set_Motorspeed(255, 255);
//   calculateLinearSpeed(255, 255);
//   go_forward(); // Forward

//   delay(2000);
//   unsigned long currentTime = millis();
//   unsigned long elapsedTime = currentTime - startTime;
//   float linearSpeed = calculateLinearSpeed(255, 255);
//   float linearDistanceCM = linearSpeed * (elapsedTime / 1000.0);
//   if (linearDistanceCM > 200) {
//     stop_Stop();
//   }
//   Serial.print("Linear Distance: ");
//   Serial.print(linearDistanceCM);
//   Serial.println(" centimeters");
//   Serial.println(elapsedTime);
//   // go_Back();//Reverse

//   // delay(2000);

//   // go_Left();//Turn left

//   // delay(2000);

//   // go_Right();//Turn right

//   // delay(2000);

//   stop_Stop();//Stop

// }

// void loop()
// {
//   // watch();
// }

//! Lesson 5
#include <Arduino.h>
#include <Servo.h>
/*Declare L298N Dual H-Bridge Motor Controller directly since there is not a library to load.*/
// Define L298N Dual H-Bridge Motor Controller Pins
#define speedPinR 13       // RIGHT PWM pin connect MODEL-X ENA
#define RightDirectPin1 12 //  Right Motor direction pin 1 to MODEL-X IN1
#define RightDirectPin2 11 // Right Motor direction pin 2 to MODEL-X IN2
#define speedPinL 6        //  Left PWM pin connect MODEL-X ENB
#define LeftDirectPin1 7   // Left Motor direction pin 1 to MODEL-X IN3
#define LeftDirectPin2 8   /// Left Motor direction pin 1 to MODEL-X IN4
#define LPT 2              // scan loop coumter

#define SERVO_PIN 9 // servo connect to D9

#define Echo_PIN 2 // Ultrasonic Echo pin connect to D11
#define Trig_PIN 3 // Ultrasonic Trig pin connect to D12

#define BUZZ_PIN 13
#define FAST_SPEED 250  // both sides of the motor speed
#define SPEED 220       // both sides of the motor speed
#define TURN_SPEED 200  // both sides of the motor speed
#define BACK_SPEED1 255 // back speed
#define BACK_SPEED2 90  // back speed

#define LFSensor_0 A0 // OLD D3
#define LFSensor_1 A1
#define LFSensor_2 A2
#define LFSensor_3 A3
#define LFSensor_4 A4 // OLD D10

int leftscanval, centerscanval, rightscanval, ldiagonalscanval, rdiagonalscanval;
const int distancelimit = 10;     // distance limit for obstacles in front
const int sidedistancelimit = 10; // minimum distance in cm to obstacles at both sides (the car will allow a shorter distance sideways)
int distance;
int numcycles = 0;
const int turntime = 250; // Time the robot spends turning (miliseconds)
const int backtime = 300; // Time the robot spends turning (miliseconds)

int thereis;
Servo head;

/*set motor speed */
void set_Motorspeed(int speed_L, int speed_R)
{
  analogWrite(speedPinL, speed_L);
  analogWrite(speedPinR, speed_R);
}

/*motor control*/
void go_Advance(void) // Forward
{
  digitalWrite(RightDirectPin1, HIGH);
  digitalWrite(RightDirectPin2, LOW);
  digitalWrite(LeftDirectPin1, HIGH);
  digitalWrite(LeftDirectPin2, LOW);
}
void go_Left() // Turn left
{
  digitalWrite(RightDirectPin1, HIGH);
  digitalWrite(RightDirectPin2, LOW);
  digitalWrite(LeftDirectPin1, LOW);
  digitalWrite(LeftDirectPin2, HIGH);
}
void go_Right() // Turn right
{
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2, HIGH);
  digitalWrite(LeftDirectPin1, HIGH);
  digitalWrite(LeftDirectPin2, LOW);
}
void go_Back() // Reverse
{
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2, HIGH);
  digitalWrite(LeftDirectPin1, LOW);
  digitalWrite(LeftDirectPin2, HIGH);
}
void stop_Stop() // Stop
{
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2, LOW);
  digitalWrite(LeftDirectPin1, LOW);
  digitalWrite(LeftDirectPin2, LOW);
  set_Motorspeed(0, 0);
}

void buzz_ON() // open buzzer
{

  for (int i = 0; i < 100; i++)
  {
    digitalWrite(BUZZ_PIN, LOW);
    delay(2); // wait for 1ms
    digitalWrite(BUZZ_PIN, HIGH);
    delay(2); // wait for 1ms
  }
}
void buzz_OFF() // close buzzer
{
  digitalWrite(BUZZ_PIN, HIGH);
}
void alarm()
{
  buzz_ON();

  buzz_OFF();
}
//? -------------------------- LINE TRACKING --------------------------
//~ reads value from line tracking sensor
char sensor[5];
/*read sensor value string, 1 stands for black, 0 starnds for white, i.e 10000 means the first sensor(from left) detect black line, other 4 sensors detected white ground */
String read_sensor_values()
{
  int sensorvalue = 32;
  sensor[0] = !digitalRead(LFSensor_0);

  sensor[1] = !digitalRead(LFSensor_1);

  sensor[2] = !digitalRead(LFSensor_2);

  sensor[3] = !digitalRead(LFSensor_3);

  sensor[4] = !digitalRead(LFSensor_4);
  sensorvalue += (sensor[0] * 16) + (sensor[1] * 8) + (sensor[2] * 4) + (sensor[3] * 2) + (sensor[4]);

  String senstr = String(sensorvalue, BIN);
  senstr = senstr.substring(1, 6);

  return senstr;
}

void auto_tracking()
{
  String sensorval = read_sensor_values();
  Serial.println(sensorval);


    if (sensorval == "10000")
    {
      // The black line is in the left of the car, need  left turn
      go_Left(); // Turn left
      set_Motorspeed(SPEED, SPEED);
    }
    if (sensorval == "10100" || sensorval == "01000" || sensorval == "01100" || sensorval == "11100" || sensorval == "10010" || sensorval == "11010")
    {
      go_Advance(); // Turn slight left
      set_Motorspeed(0, SPEED);
    }
    if (sensorval == "00001" && 2 == 2)
    {             // The black line is  on the right of the car, need  right turn
      go_Right(); // Turn right
      set_Motorspeed(SPEED, SPEED);
    }
    if (sensorval == "00011" || sensorval == "00010" || sensorval == "00101" || sensorval == "00110" || sensorval == "00111" || sensorval == "01101" || sensorval == "01111" || sensorval == "01011" || sensorval == "01001")
    {
      go_Advance(); // Turn slight right
      set_Motorspeed(SPEED, 0);
    }

    if (sensorval == "11111")
    {

      stop_Stop(); // The car front touch stop line, need stop
      set_Motorspeed(0, 0);
    }
  
}

//? -------------------------- ULTRASONIC SENSOR --------------------------

//~ Detects objects in front of the robot and returns the distance in cm
int watch()
{
  long echo_distance;
  digitalWrite(Trig_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(Trig_PIN, HIGH);
  delayMicroseconds(15);
  digitalWrite(Trig_PIN, LOW);
  echo_distance = pulseIn(Echo_PIN, HIGH);
  echo_distance = echo_distance * 0.01657; // how far away is the object in cm
  return round(echo_distance);
}
// Meassures distances to the right, left, front, left diagonal, right diagonal and asign them in cm to the variables rightscanval,
// leftscanval, centerscanval, ldiagonalscanval and rdiagonalscanval (there are 5 points for distance testing)
String watchsurrounding()
{
  /*  obstacle_status is a binary integer, its last 5 digits stands for if there is any obstacles in 5 directions,
   *   for example B101000 last 5 digits is 01000, which stands for Left front has obstacle, B100111 means front, right front and right ha
   */
  //& Obstacle Status Understanding
  //& 5 digits = 00000
  //& 1st digit = right
  //& 2nd digit = right diagonal
  //& 3rd digit = front
  //& 4th digit = left diagonal
  //& 5th digit = left
  //&   0         0        0         0          0
  //& Left Left Diagonal Front Right Diagonal Right
  //& ===> No obstacle
  //&   0         0        1         0          0
  //& Left Left Diagonal Front Right Diagonal Right
  //& ===> Object in front
  int obstacle_status = B100000; //~ obstacle

  //~ Obstacle in Front
  centerscanval = watch(); //~ gets distance in front and assigns it to centerscanval
  if (centerscanval < distancelimit)
  {
    stop_Stop();
    alarm();
    obstacle_status = obstacle_status | B100; //~ Bitwise OR operator, sets the 3rd bit to 1
  }

  //~ Obstacle in Left Diagonal
  head.write(120); //~ This turns the ultrasonic sensor to check the left diagonal
  delay(100);
  ldiagonalscanval = watch();
  if (ldiagonalscanval < distancelimit)
  {
    stop_Stop();
    alarm();
    obstacle_status = obstacle_status | B1000;
  }

  //~ Obstacle in Left
  head.write(180);
  delay(300);
  leftscanval = watch();
  if (leftscanval < sidedistancelimit)
  {
    stop_Stop();
    alarm();
    obstacle_status = obstacle_status | B10000;
  }

  //~ Moves Object back to Front
  head.write(90); // use 90 degrees if you are moving your servo through the whole 180 degrees
  delay(100);
  centerscanval = watch();
  if (centerscanval < distancelimit)
  {
    stop_Stop();
    alarm();
    obstacle_status = obstacle_status | B100;
  }

  //~ Obstacle in Right Diagonal
  head.write(40);
  delay(100);
  rdiagonalscanval = watch();
  if (rdiagonalscanval < distancelimit)
  {
    stop_Stop();
    alarm();
    obstacle_status = obstacle_status | B10;
  }

  //~ Obstacle in Right
  head.write(0);
  delay(100);
  rightscanval = watch();
  if (rightscanval < sidedistancelimit)
  {
    stop_Stop();
    alarm();
    obstacle_status = obstacle_status | 1;
  }
  //~ Moves Object back to Front
  head.write(90); // Finish looking around (look forward again)
  delay(300);
  String obstacle_str = String(obstacle_status, BIN);
  obstacle_str = obstacle_str.substring(1, 6); //~ Turns string to 5 digit binary

  return obstacle_str; // return 5-character string standing for 5 direction obstacle status
}

void auto_avoidance()
{

  ++numcycles; //~ Increases the number of cycles by 1
  if (numcycles >= LPT)
  { // Watch if something is around every LPT loops while moving forward
    stop_Stop();
    String obstacle_sign = watchsurrounding(); // 5 digits of obstacle_sign binary value means the 5 direction obstacle status
    Serial.print("begin str=");
    Serial.println(obstacle_sign);
    if (obstacle_sign == "10000")
    {
      Serial.println("SLIT right");
      set_Motorspeed(FAST_SPEED, SPEED);
      go_Advance();

      delay(turntime);
      stop_Stop();
    }
    else if (obstacle_sign == "00001")
    {
      Serial.println("SLIT LEFT");
      set_Motorspeed(SPEED, FAST_SPEED);
      go_Advance();

      delay(turntime);
      stop_Stop();
    }
    else if (obstacle_sign == "11100" || obstacle_sign == "01000" || obstacle_sign == "11000" || obstacle_sign == "10100" || obstacle_sign == "01100" || obstacle_sign == "00100" || obstacle_sign == "01000")
    {
      Serial.println("hand right");
      go_Right();
      set_Motorspeed(TURN_SPEED, TURN_SPEED);
      delay(turntime);
      stop_Stop();
    }
    else if (obstacle_sign == "00010" || obstacle_sign == "00111" || obstacle_sign == "00011" || obstacle_sign == "00101" || obstacle_sign == "00110" || obstacle_sign == "01010")
    {
      Serial.println("hand left");
      go_Left(); // Turn left
      set_Motorspeed(TURN_SPEED, TURN_SPEED);
      delay(turntime);
      stop_Stop();
    }

    else if (obstacle_sign == "01111" || obstacle_sign == "10111" || obstacle_sign == "11111")
    {
      Serial.println("hand back right");
      go_Left();
      set_Motorspeed(FAST_SPEED, SPEED);
      delay(backtime);
      stop_Stop();
    }
    else if (obstacle_sign == "11011" || obstacle_sign == "11101" || obstacle_sign == "11110" || obstacle_sign == "01110")
    {
      Serial.println("hand back left");
      go_Right();
      set_Motorspeed(SPEED, FAST_SPEED);
      delay(backtime);
      stop_Stop();
    }

    else
    {
      Serial.println("no handle");
      numcycles = 0;
    } // Restart count of cycles
  }

  else
  {
    set_Motorspeed(FAST_SPEED, FAST_SPEED);
    go_Advance(); // if nothing is wrong go forward using go() function above.
    delay(backtime);
    stop_Stop();
  }

  // else  Serial.println(numcycles);

  distance = watch(); // use the watch() function to see if anything is ahead (when the robot is just moving forward and not looking around it will test the distance in front)
  if (distance < distancelimit)
  { // The robot will just stop if it is completely sure there's an obstacle ahead (must test 25 times) (needed to ignore ultrasonic sensor's false signals)
    Serial.println("final go back");
    go_Right();
    set_Motorspeed(SPEED, FAST_SPEED);
    delay(backtime * 3 / 2);
    ++thereis;
  }
  if (distance > distancelimit)
  {
    thereis = 0;
  } // Count is restarted
  if (thereis > 25)
  {
    Serial.println("final stop");
    stop_Stop(); // Since something is ahead, stop moving.
    thereis = 0;
  }
}

void setup()
{
  /*setup L298N pin mode*/
  pinMode(RightDirectPin1, OUTPUT);
  pinMode(RightDirectPin2, OUTPUT);
  pinMode(speedPinL, OUTPUT);
  pinMode(LeftDirectPin1, OUTPUT);
  pinMode(LeftDirectPin2, OUTPUT);
  pinMode(speedPinR, OUTPUT);
  stop_Stop(); // stop move
  /*init HC-SR04*/
  pinMode(Trig_PIN, OUTPUT);
  pinMode(Echo_PIN, INPUT);
  /*init buzzer*/
  pinMode(BUZZ_PIN, OUTPUT);
  digitalWrite(BUZZ_PIN, HIGH);
  buzz_OFF();

  digitalWrite(Trig_PIN, LOW);
  /*init servo*/
  head.attach(SERVO_PIN);
  head.write(90);
  delay(2000);

  Serial.begin(9600);
}

void loop()
{
  auto_avoidance();
}
