/*This code is designed to make the robot car capable of  avoiding obsticles*/

//UltraSonic
#define trigPin 13    //defining the arduino's digital pin 13 as the trigger pin
#define echoPin 12    //defining the arduino's digital pin 12 as the echo pin
float duration, distance ;    //allowing for the value of the duration and distance to be represented as a float value

//Servo
#include <Servo.h>    //activating the servo library
Servo myservo;    //giving the digital object "Servo" from the library the name "myservo"

//Motor A
int speedPinA= 5;   //defining the arduino's digital pin 5 as the speed pin for motor A
int dirPinA= 7;   //defining the arduino's digital pin 7 as the direction pin for motor A

//Motor B
int speedPinB= 6;   //defining the arduino's digital pin 6 as the speed pin for motor B
int dirPinB= 8;   //defining the arduino's digital pin 8 as the direction pin for motor B

//Common motor Stuff
int Speed= 100;   //Stateing that the motor speed is at 100
int STBY= 3;   //defining the arduino's digital pin 3 as the StandBy pin

//Sensor
int rightSens= A0;    //the line tracker's right sensor is connected to the analog pin 0
int middleSens= A1;    //the line tracker's middle sensor is connected to the analog pin 1
int leftSens= A2;    //the line tracker's left sensor is connected to the analog pin 2

int sensDis= 25;   //Stateing that the UltraSonic sensor will detect an object at 25 cm away

//Function that sends a pulse to the ultra sonic sensor so that it could sens an object
void checkDis() {
digitalWrite(trigPin,LOW); 
delayMicroseconds(2); 
digitalWrite(trigPin,HIGH); 
delayMicroseconds(10); 
digitalWrite(trigPin, LOW); 
duration = pulseIn(echoPin, HIGH); 
distance = ( duration/2) * 0.0343; 
}

//Function that runs the motors so the car drives forward 
void forward(){
digitalWrite(dirPinA, 1);
analogWrite(speedPinA, Speed);
digitalWrite(dirPinB, 1);
analogWrite(speedPinB, Speed);
}

//Function that runs the motors so the car drives backward 
void backward(){
digitalWrite(dirPinA, 0);
analogWrite(speedPinA, Speed);
digitalWrite(dirPinB, 0);
analogWrite(speedPinB, Speed);
}

//Function that runs the motors so the car turns to the left 
void left(){
digitalWrite(dirPinA, 1);
analogWrite(speedPinA, Speed);
digitalWrite(dirPinB, 0);
analogWrite(speedPinB, Speed);
}

//Function that runs the motors so the car turns to the right 
void right(){
digitalWrite(dirPinA, 0);
analogWrite(speedPinA, Speed);
digitalWrite(dirPinB, 1);
analogWrite(speedPinB, Speed);
}

//Function that runs the motors so the car is both turning left and driving forward
void leftForward(){
digitalWrite(dirPinA, 1);
analogWrite(speedPinA, Speed);
digitalWrite(dirPinB, 1);
analogWrite(speedPinB, Speed/2);
}

//Function that runs the motors so the car is both turning left and driving backward
void leftBackward(){
digitalWrite(dirPinA, 0);
analogWrite(speedPinA, Speed);
digitalWrite(dirPinB, 0);
analogWrite(speedPinB, Speed/2);
}

//Function that runns the motors so the car is both turning right and driving forward
void rightForward(){
digitalWrite(dirPinA, 1);
analogWrite(speedPinA, Speed/2);
digitalWrite(dirPinB, 1);
analogWrite(speedPinB, Speed);
}

//Function that runs the motors so the car is both turning right and driving backward
void rightBackward(){
digitalWrite(dirPinA, 0);
analogWrite(speedPinA, Speed/2);
digitalWrite(dirPinB, 0);
analogWrite(speedPinB, Speed);
}

//Function that runs the motors so the car stops moving 
void stop(){
digitalWrite(dirPinA, 1);
analogWrite(speedPinA, 0);
digitalWrite(dirPinB, 1);
analogWrite(speedPinB, 0);
}


void setup() {  
//UltraSonic
pinMode(trigPin, OUTPUT);   //defining the trigger pin of the ultrasonic sensor as an OUTPUT
pinMode(echoPin, INPUT);   //defining the echo pin of the ultrasonic sensor as an INPUT

//Servo
myservo.attach(10);   //Stating that the servo motor is connected to the digital pin 10

//motor
digitalWrite(STBY, 1);    //giving the stand by pin of the motors a high value
pinMode(speedPinA, OUTPUT);   //defining the speed pin of motor A as an OUTPUT
pinMode(dirPinA, OUTPUT);   //defining the direction pin of motor A as an OUTPUT
pinMode(speedPinB, OUTPUT);      //defining the speed pin of motor B as an OUTPUT
pinMode(dirPinB, OUTPUT);   //defining the direction pin of motor B as an OUTPUT

Serial.begin (9600);    //starting the serial monitor
} 

void loop() { 
Serial.print("Distance=");    //printing the word "distance=" on the serial monitor
Serial.print("\t");   //printing a TAB space after "distance="
Serial.println(distance);   //printing the actual value of the distance

int RsensVal= analogRead(rightSens);    //saving the analog value read by the line follower's right sensor in the integer called "RsensVal"
int MsensVal= analogRead(middleSens);    //saving the analog value read by the line follower's middle sensor in the integer called "MsensVal"
int LsensVal= analogRead(leftSens);    //saving the analog value read by the line follower's left sensor in the integer called "LsensVal"

//if the line following sensors does'nt sense a surface(floor) less than 1000 away the "stop" function will be called 
if (LsensVal, MsensVal, RsensVal >= 1000){
  stop();
  Serial.println("Stop");
  delay(100);
}

checkDis();   //calling the checkDis function to see if there is an object ahead of the car

if(distance <= sensDis)   //if the distence sensed is less than the sensDis than the car will stop
{
  stop();   //calling the stop function
  myservo.write(10);    //turning servo to the left
  delay(1000);
  checkDis();   //checking the object distance while the servo is turned to the left
  if (distance <= sensDis)    //if an object is sensed while the servo is in facing to the left the servo turns to the right
  {
    myservo.write(170);   //turning the servo to the right
    delay(1000);
    checkDis();   //checking the object distance while the servo is turned to the right
   if (distance <= sensDis)   //if an object is sensed while the servo is in facing to the right that means there's an objec infront and to both sides of the car the car will then turn to the left until its facing 180deg and it will go back
    {
    myservo.write(90);    //the servo goes back to facing forward
    backward();   //the car reverses for 1 second
    delay(1000);
    left();   //then turns left for 1.35 seconds(turning 180deg)
    delay(1350);
  
    }else{    //if there is no object detected to the right of the car it will turn to the right and drive in that direction
    myservo.write(90);    //the servo goes back to facing forward
    delay(100);
    left();   //turning the car left for 0.6 seconds
    delay(600);
    }
  }else{    //if there is no object detected to the left of the car it will turn to the left and drive in that direction
  myservo.write(90);    //the servo goes back to facing forward
  delay(100);
  right();   //turning the car right for 0.6 seconds
  delay(600);
  }
}else{    //if there is no object detected infront of the car it will continue driving forward
forward();    //continue driving forward
}
}