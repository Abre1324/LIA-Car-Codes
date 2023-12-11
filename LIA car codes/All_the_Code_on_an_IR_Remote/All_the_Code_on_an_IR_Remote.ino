/*This code is designed to control 3 different modes using the IR remote: line following, obsticle avoidance and controlled movement*/
#include <IRremote.h>   //activating the library for the IR remote
#define IR_RECEIVE_PIN 9    //stateing that the IR recever is connected to digital pin 9 on the arduino

//Motor A
int speedPinA= 5;   //defining the arduino's digital pin 5 as the speed pin for motor A
int dirPinA= 7;   //defining the arduino's digital pin 7 as the direction pin for motor A

//Motor B
int speedPinB= 6;   //defining the arduino's digital pin 6 as the speed pin for motor B
int dirPinB= 8;   //defining the arduino's digital pin 8 as the direction pin for motor B

//Common Stuff
int Speed;    //stating that the Speed is an integer
int STBY= 3;   //defining the arduino's digital pin 3 as the StandBy pin
int sensDis= 25;   //Stateing that the UltraSonic sensor will detect an object at 25 cm away
int command;    //the value of "command" is an integer

//Sensor
int rightSens= A0;    //the line tracker's right sensor is connected to the analog pin 0
int middleSens= A1;    //the line tracker's middle sensor is connected to the analog pin 1
int leftSens= A2;    //the line tracker's left sensor is connected to the analog pin 2

//UltraSonic
#define trigPin 13    //defining the arduino's digital pin 13 as the trigger pin
#define echoPin 12     //defining the arduino's digital pin 12 as the echo pin
float duration, distance ;

//Servo
#include <Servo.h>    //activating the servo library
Servo myservo;    //giving the digital object "Servo" from the library the name "myservo"

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

//this function is similar to the regular left function but the speed is 50
void lnLeft(){
digitalWrite(dirPinA, 1);
analogWrite(speedPinA, 50);
digitalWrite(dirPinB, 0);
analogWrite(speedPinB, 50);
}

//this function is similar to the regular right function but the speed is 50
void lnRight(){
digitalWrite(dirPinA, 0);
analogWrite(speedPinA, 50);
digitalWrite(dirPinB, 1);
analogWrite(speedPinB, 50);
}

//Function that runs the motors so the car stops moving
void stop(){
digitalWrite(dirPinA, 1);
analogWrite(speedPinA, 0);
digitalWrite(dirPinB, 1);
analogWrite(speedPinB, 0);
}


void setup() {
Serial.begin(9600);    //starting the serial monitor

//IR
IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);    //initializing the IR reciver                                                      

//Motor
digitalWrite(STBY, 1);    //giving the stand by pin of the motors a high value
pinMode(speedPinA, OUTPUT);   //defining the speed pin of motor A as an OUTPUT
pinMode(dirPinA, OUTPUT);   //defining the direction pin of motor A as an OUTPUT
pinMode(speedPinB, OUTPUT);      //defining the speed pin of motor B as an OUTPUT
pinMode(dirPinB, OUTPUT);   //defining the direction pin of motor B as an OUTPUT

//UltraSonic
pinMode(trigPin, OUTPUT);   //defining the trigger pin of the ultrasonic sensor as an OUTPUT
pinMode(echoPin, INPUT);   //defining the echo pin of the ultrasonic sensor as an INPUT

//Servo
myservo.attach(10);   //Stating that the servo motor is connected to the digital pin 10
}


void loop() {
  uint16_t command = IrReceiver.decodedIRData.command;

  //this reads the value of the IR controler and printis it to the serial monitor
  if (IrReceiver.decode()) {
    Serial.println(command);
    delay(100);  // wait a bit
    IrReceiver.resume();
  }

int RsensVal= analogRead(rightSens);    //saving the analog value read by the line follower's right sensor in the integer called "RsensVal"
int MsensVal= analogRead(middleSens);    //saving the analog value read by the line follower's middle sensor in the integer called "MsensVal"
int LsensVal= analogRead(leftSens);    //saving the analog value read by the line follower's left sensor in the integer called "LsensVal"

Serial.println(Speed); //prints the value od "Speed" on the serial monitor

  if(command== 74){   //if the button with a value of 74 is pressed the car will go at max speed
    Speed= 255;
  }
  else if(command== 82){   //if the button with a value of 82 is pressed the car will go at a speed of 100
    Speed= 100;
  }
  else if(command== 66){   //if the button with a value of 66 is pressed the car will go at a speed of 50
    Speed= 50;
  }

  if(command == 70){   //if the button with a value of 70 is pressed the car will drive forward
    forward();
  }
  else if (command == 67){   //if the button with a value of 67 is pressed the car will turn to the right
    right();
  }
  else if (command == 68){   //if the button with a value of 68 is pressed the car will turn to the left
    left();
  }
  else if (command == 64){   //if the button with a value of 64 is pressed the car will stop moving
    stop();
  }
  else if (command == 21){   //if the button with a value of 21 is pressed the car will drive backward
    backward();
  }

  if(command == 22){   //if the button with a value of 22 is pressed the line following code will start
    Serial.print("line follow");    //this will print the "line follow" on the serial monitor
    Serial.print("\t");   //printing a TAB space after "line follow"

    if (MsensVal >= 300 && MsensVal <= 900){    //if the middle sensor senses a line between this distance the car will move forward
      forward();
      Serial.println("foward");   //will print "forward" to the serial monitor (for testing)
      delay(100);
    }
    else if (LsensVal, MsensVal, RsensVal >= 1000){   //if there is no object less than 1000 away the car will stop
      stop();
      Serial.println("Stop");   //will print "Stop" to the serial monitor (for testing)
      delay(100);
    }
    else if (LsensVal >= 300 && LsensVal <= 900){    //if the left sensor senses a line between this distance the car will correct its angle
      lnLeft();
      Serial.println("left");   //will print "left" to the serial monitor (for testing)
      delay(100);
    }
    else if (RsensVal >= 300 && RsensVal <= 900){    //if the right sensor senses a line between this distance the car will correct its angle
      lnRight();
      Serial.println("right");   //will print "right" to the serial monitor (for testing)
      delay(100);
    }
    else if (LsensVal, MsensVal, RsensVal <= 100){    //if the car cant sens any lines it will continue turning to the right into it senses a line
      lnRight();
    }  
    else{    //if all the conditions arent't true the car will stop moving
      stop();
    }
  }
  else if(command == 25){   //if the button with a value of 25 is pressed the car will go in obsticle avoidance mode
    Serial.print ("obsticle avoid");    //printing the words "obsticle avoid" on the serial monitor
    Serial.print("\t");   //printing a TAB space distance after "obsticle avoid"
    Serial.print("Distance=");    //printing the words "Distance=" on the serial monitor
    Serial.print("\t");   //printing a TAB space distance after "obsticle avoid"
    Serial.println(distance);   //printing the distance value on the serial monitor

    checkDis();   //this pulses a signal to the UltraSonic sensor
      if(distance <= sensDis){    //checks if the distance sensed by the UltraSonic sensor is less than or equal to 25
        stop();   //if the sensor senses an object the car will stop
        myservo.write(10);    //the servo is turned to the left position
        delay(1000);
        checkDis();   //this pulses a signal to the UltraSonic sensor
      
        if (distance <= sensDis){    //checks if the distance sensed by the UltraSonic sensor is less than or equal to 25 while facing the sensor to the left
        myservo.write(170);    //if the sensor senses an objed while facing to the left the servo is turns to the right position
        delay(1000);
        checkDis();   //this pulses a signal to the UltraSonic sensor
    
          if (distance <= sensDis){    //checks if the distance sensed by the UltraSonic sensor is less than or equal to 25 while facing the sensor to the right
            myservo.write(90);    //the servo is turned to the forward position
            backward();   //this makes the car drive backwards because there are objects ahead and on both sides of the car
            delay(1000);
            left();   //this makes the car turn to the left and turns 1.35 seconds so the car turns 180deg
            delay(1350);
      
          }else{
          myservo.write(90);    //the servo is turned to the forward position
          delay(100);
          left();   //this turns the car to the left
          delay(600);
          }
        }else{
        myservo.write(90);    //the servo is turned to the forward position
        delay(100);
        right();   //this turns the car to the right
        delay(600);
        }
  
      }else{
        forward();    //drives the car forward
      }

  }
  Serial.println(command);    //prints the value of command (for testing)
}