#define speedPinL 6    // ENB
#define leftWheelPin1  7    //Left Motor direction pin 1 to MODEL-X IN3 
#define leftWheelPin2  8   //Left Motor direction pin 2 to MODEL-X IN4
#define speedPinR 5    //  ENA
#define rightWheelPin1  12    //Right Motor direction pin 1 to MODEL-X IN1 
#define rightWheelPin2 11    //Right Motor direction pin 2 to MODEL-X IN2

#define LFSensor_0 A0 // We define LFSensor_0 to hold the left most value of the IR sensor. This IR sensor is connected to pin A0 as an analog pin is required to read continous IR values. This is then repeated for each subsequent IR sensor

#define LFSensor_1 A1 // Holds second IR sensor value
#define LFSensor_2 A2 // Holds third IR sensor value
#define LFSensor_3 A3 // Holds fourth IR sensor value
#define LFSensor_4 A4 // Holds fifth IR sensor value


#define FAST_SPEED 150
#define MID_SPEED 140
#define SLOW_SPEED 130   //back speed

#define trigPin 2
#define echoPin 3

#include <Servo.h>
Servo myservo;

void set_Motorspeed(int speed_L,int speed_R)
{
  analogWrite(speedPinL,speed_L); 
  analogWrite(speedPinR,speed_R);   
}

void go_Forward(){
  digitalWrite(rightWheelPin1,HIGH);
  digitalWrite(rightWheelPin2,LOW);
  digitalWrite(leftWheelPin1,HIGH);
  digitalWrite(leftWheelPin2,LOW);
  set_Motorspeed(200,200);
  
}

void go_Back(){
  digitalWrite(rightWheelPin1,LOW);
  digitalWrite(rightWheelPin2,HIGH);
  digitalWrite(leftWheelPin1,LOW);
  digitalWrite(leftWheelPin2,HIGH);
  set_Motorspeed(200,200);

}

void go_Left(){
  digitalWrite(rightWheelPin1,HIGH);
  digitalWrite(rightWheelPin2,LOW);
  digitalWrite(leftWheelPin1,LOW);
  digitalWrite(leftWheelPin2,HIGH);
  set_Motorspeed(200,200);
}

void go_Right(){
  digitalWrite(rightWheelPin1,LOW);
  digitalWrite(rightWheelPin2,HIGH);
  digitalWrite(leftWheelPin1,HIGH);
  digitalWrite(leftWheelPin2,LOW);
  set_Motorspeed(200,200);
}

void stop_Robot(){
  digitalWrite(rightWheelPin1,LOW);
  digitalWrite(rightWheelPin2,LOW);
  digitalWrite(leftWheelPin1,LOW);
  digitalWrite(leftWheelPin2,LOW);
}
char sensor[5];
// — — — — — — Explanation of how the TSM values are stored and handled — — — — — — 
// To represent our line detection input, we use binary. If the Tracking Sensor Module (TSM) detects black, it registers as 1; otherwise, it's 0. 
// For instance, if the TSM detects black on the leftmost sensor, it will read '10000', with '1' representing black and '0' representing white. 
// We work with binary numbers to represent the sensor's view of black and white, so our calculations start in decimal and later convert to binary.

// A function that reads sensor values and returns them as type 'String'.
String read_sensor_values() {
    int sensorvalue = 32; // Initialize a variable called sensorvalue and set it equal to 32 (100000 in binary). We use 6 digits for accurate binary addition; the 6th digit, the '1', will be dropped, leaving a 5-digit sensor reading.

  // Read the values of the five sensors and invert them using the '!' (NOT) operator. Assign these inverted values to the corresponding positions in the sensor array.
    sensor[0] = !digitalRead(LFSensor_0);
    sensor[1] = !digitalRead(LFSensor_1);
    sensor[2] = !digitalRead(LFSensor_2);
    sensor[3] = !digitalRead(LFSensor_3);
    sensor[4] = !digitalRead(LFSensor_4);

// ^^^
// According to the information on [3], black is active at LOW (0), and white is active at HIGH (1). We want the opposite, so we use the exclamation mark when reading the value. 
// We use digitalRead instead of analogRead as we need an integer value of either 1 or 0. 
// DigitalRead provides either HIGH (1) or LOW (0) states, while analogRead can have any value between 0 and 1023.

// The binary values we obtain must be converted to decimal by multiplying each bit by 2 to the power of its index.
// Even though are values of type 'char' we can perform mathematical operations on them as C++ will convert them into 'int' types
// sensor[0] is a char type that can be either 1 or 0. when multiplied by 16 it becomes either 16 or 0 which is added to sensor value. This is then repeated for all other values
    sensorvalue += (sensor[0] * 16) + (sensor[1] * 8) + (sensor[2] * 4) + (sensor[3] * 2) +      (sensor[4]);

// Powers of 2 are used as these numbers are requied to convert to binary
//Then, convert the sensorvalue to binary and remove the last digit using substring. Finally, return the result.
    String senstr = String(sensorvalue, BIN);
    senstr = senstr.substring(1, 6);
    return senstr;
}


void lineFollowing(){
 String sensorval= read_sensor_values();
  //Serial.println(sensorval);
  if (   sensorval=="10000" )
  { 
    //The black line is in the left of the car, need  left turn 
    go_Left();  //Turn left
    set_Motorspeed(FAST_SPEED,FAST_SPEED);
  }
  if (sensorval=="10100"  || sensorval=="01000" || sensorval=="01100" || sensorval=="11100"  || sensorval=="10010" || sensorval=="11010")
  {
    go_Forward();  //Turn slight left
    set_Motorspeed(0,FAST_SPEED);
  }
  if (    sensorval=="00001"  ){ //The black line is  on the right of the car, need  right turn 
    go_Right();  //Turn right
    set_Motorspeed(FAST_SPEED,FAST_SPEED);
  }
  if (sensorval=="00011" || sensorval=="00010"  || sensorval=="00101" || sensorval=="00110" || sensorval=="00111" || sensorval=="01101" || sensorval=="01111"   || sensorval=="01011"  || sensorval=="01001")
  {
    go_Forward();  //Turn slight right
    set_Motorspeed( FAST_SPEED,0);
  }
 
  if (sensorval=="11111"){
    stop_Robot();   //The car front touch stop line, need stop
  }
    
}
float sendPing(){
  float distance;
  long duration;
  digitalWrite(trigPin,LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin,LOW);
  duration = pulseIn(echoPin,HIGH);
  distance = duration*0.034/2;
  return (distance);
}

void obstacleAvoidance(){
  go_Right();
  delay(500);
  go_Forward();
  delay(460);
  go_Left();
  delay(500);
  go_Forward();
  delay(600);
  go_Left();
  delay(400);
  go_Forward();
  delay(300);

}


void setup() {

  myservo.attach(13);
  myservo.write(90);
  delay(500);

  pinMode(speedPinL,OUTPUT);
  pinMode(leftWheelPin1,OUTPUT);
  pinMode(leftWheelPin2,OUTPUT);
  pinMode(speedPinR,OUTPUT);
  pinMode(rightWheelPin1,OUTPUT);
  pinMode(rightWheelPin2,OUTPUT);

  stop_Robot(); 

  digitalWrite(speedPinL,HIGH);
  digitalWrite(speedPinR,HIGH);

  pinMode(trigPin,OUTPUT);
  pinMode(echoPin,INPUT);

  
}

void loop() {

    lineFollowing();
    if(sendPing()<8){
      obstacleAvoidance();
    }

}
