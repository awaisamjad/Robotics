#include <Servo.h>
/*Declare L298N Dual H-Bridge Motor Controller directly since there is not a library to load.*/
// Define L298N Dual H-Bridge Motor Controller Pins
#define speedPinR 13        // RIGHT PWM pin connect MODEL-X ENA
#define RightDirectPin1 12  //  Right Motor direction pin 1 to MODEL-X IN1
#define RightDirectPin2 11  // Right Motor direction pin 2 to MODEL-X IN2
#define speedPinL 6         //  Left PWM pin connect MODEL-X ENB
#define LeftDirectPin1 7    // Left Motor direction pin 1 to MODEL-X IN3
#define LeftDirectPin2 8    /// Left Motor direction pin 1 to MODEL-X IN4
#define LPT 2               // scan loop coumter

#define SERVO_PIN 9  // servo connect to D9

#define Echo_PIN 2  // Ultrasonic Echo pin connect to D11
#define Trig_PIN 3  // Ultrasonic Trig pin connect to D12

#define BUZZ_PIN 13
#define FAST_SPEED 250   // both sides of the motor speed
#define SPEED 220        // both sides of the motor speed
#define TURN_SPEED 200   // both sides of the motor speed
#define BACK_SPEED1 255  // back speed
#define BACK_SPEED2 90   // back speed

#define LFSensor_0 A0  // OLD D3
#define LFSensor_1 A1
#define LFSensor_2 A2
#define LFSensor_3 A3
#define LFSensor_4 A4  // OLD D10

int leftscanval, centerscanval, rightscanval, ldiagonalscanval, rdiagonalscanval;
const int distancelimit = 10;      // distance limit for obstacles in front
const int sidedistancelimit = 10;  // minimum distance in cm to obstacles at both sides (the car will allow a shorter distance sideways)
int distance;
int numcycles = 0;
const int turntime = 250;  // Time the robot spends turning (miliseconds)
const int backtime = 300;  // Time the robot spends turning (miliseconds)

int thereis;
Servo head;

/*set motor speed */
void set_Motorspeed(int speed_L, int speed_R) {
  analogWrite(speedPinL, speed_L);
  analogWrite(speedPinR, speed_R);
}

/*motor control*/
void go_Advance(void)  // Forward
{
  digitalWrite(RightDirectPin1, HIGH);
  digitalWrite(RightDirectPin2, LOW);
  digitalWrite(LeftDirectPin1, HIGH);
  digitalWrite(LeftDirectPin2, LOW);
}
void go_Left()  // Turn left
{
  digitalWrite(RightDirectPin1, HIGH);
  digitalWrite(RightDirectPin2, LOW);
  digitalWrite(LeftDirectPin1, LOW);
  digitalWrite(LeftDirectPin2, HIGH);
}
void go_Right()  // Turn right
{
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2, HIGH);
  digitalWrite(LeftDirectPin1, HIGH);
  digitalWrite(LeftDirectPin2, LOW);
}
void go_Back()  // Reverse
{
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2, HIGH);
  digitalWrite(LeftDirectPin1, LOW);
  digitalWrite(LeftDirectPin2, HIGH);
}
void stop_Stop()  // Stop
{
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2, LOW);
  digitalWrite(LeftDirectPin1, LOW);
  digitalWrite(LeftDirectPin2, LOW);
  set_Motorspeed(0, 0);
}

//? -------------------------- LINE TRACKING --------------------------

char sensor[5];
/*read sensor value string, 1 stands for black, 0 stands for white, i.e 10000 means the first sensor(from left) detect black line, other 4 sensors detected white ground */
String read_sensor_values() {
  int sensorvalue = 32; // Stands for 100000 in binary => We have 6 digits as we need to change many of them and then substring them to get 5. this is done by dropping the leftmost value
  //~ All digitalRead have a ! as this gives the NOT value. wo it we get everything as 0 standing for black and 1 for white but we want the opposite so we add the !
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

void auto_line_tracking()
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
    //~ If it detects all BLACK it stops
    if (sensorval == "11111")
    {

      stop_Stop(); // The car front touch stop line, need stop
      set_Motorspeed(0, 0);
    }
    //~ If it detects all WHITE it stops
    if (sensorval == "00000")
    {

      stop_Stop(); // The car front touch stop line, need stop
      set_Motorspeed(0, 0);
    }
}

unsigned long startTime = 0;  // Variable to store the start time
unsigned long duration = 5000;  // Duration of the function in milliseconds (5 seconds)
unsigned long restTime = 1000;  // Rest time in milliseconds (1 second)

void algorithm(){
  // watchsurrounding();
  auto_line_tracking();
}


//? -------------------------- ULTRASONIC SENSOR --------------------------

int watch(){
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
    obstacle_status = obstacle_status | B100; //~ Bitwise OR operator, sets the 3rd bit to 1
  }

  //~ Obstacle in Left Diagonal
  head.write(120); //~ This turns the ultrasonic sensor to check the left diagonal
  delay(100);
  ldiagonalscanval = watch();
  if (ldiagonalscanval < distancelimit)
  {
    stop_Stop();
    obstacle_status = obstacle_status | B1000;
  }

  //~ Obstacle in Left
  head.write(180);
  delay(300);
  leftscanval = watch();
  if (leftscanval < sidedistancelimit)
  {
    stop_Stop();
    obstacle_status = obstacle_status | B10000;
  }

  //~ Moves Object back to Front
  head.write(90); // use 90 degrees if you are moving your servo through the whole 180 degrees
  delay(100);
  centerscanval = watch();
  if (centerscanval < distancelimit)
  {
    stop_Stop();
    obstacle_status = obstacle_status | B100;
  }

  //~ Obstacle in Right Diagonal
  head.write(40);
  delay(100);
  rdiagonalscanval = watch();
  if (rdiagonalscanval < distancelimit)
  {
    stop_Stop();
    obstacle_status = obstacle_status | B10;
  }

  //~ Obstacle in Right
  head.write(0);
  delay(100);
  rightscanval = watch();
  if (rightscanval < sidedistancelimit)
  {
    stop_Stop();
    obstacle_status = obstacle_status | 1;
  }
  //~ Moves Object back to Front
  head.write(90); // Finish looking around (look forward again)
  delay(300);
  String obstacle_str = String(obstacle_status, BIN);
  obstacle_str = obstacle_str.substring(1, 6); //~ Turns string to 5 digit binary

  return obstacle_str; // return 5-character string standing for 5 direction obstacle status
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

  digitalWrite(Trig_PIN, LOW);
  /*init servo*/
  head.attach(SERVO_PIN);
  head.write(90);
  delay(2000);

  Serial.begin(9600);
}

void loop()
{
  algorithm();
}