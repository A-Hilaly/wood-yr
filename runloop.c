//______Importing Adafruit MotoShield Libraries______//

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"


// Create the motor shield object with the default I2C address

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Creating ports for MotorRight & MotorLeft

Adafruit_DCMotor *MotorR = AFMS.getMotor(1); // Right
Adafruit_DCMotor *MotorL = AFMS.getMotor(2); // Left

//connect gp2d120x sensor to A1

#define pin A0


//____________________________setup_____________________________//


void setup() {
  Serial.begin(9600);   // set up Serial library at 9600 bps
  Serial.println("DC Motor start up");

  // Sensor pin
  pinMode(pin, INPUT);



  AFMS.begin();  // Create with the default frequency 1.6KHz

  // Set the speed to start [0 : 255]
  // Initial Speed Test

  MotorR->setSpeed(150);
  MotorL->setSpeed(150);

  MotorR->run(FORWARD);
  MotorL->run(FORWARD);

  // Turn on motor
  MotorR->run(RELEASE);
  MotorL->run(RELEASE);
}

//_______________________loop_________________________//

void loop() {
  int RANDOM_MOVE;

  Serial.println(" START MOVING ");

  /* Both of the DC motors run with an initial speed until
  the sensor reach a level where the DC motors should stop
  and run BACKWARD */

  // While distance >30 CM
  while(ReadSens()==true){
    Serial.println(" KEEP MOVING ");
    Move_FORWARD(MotorR,MotorL);
    delay(20);
  }
  // While breaked <=> distance < 30cm
  // Stop motors
  Serial.println(" OBSTACLE ");
  StopMotors(MotorR,MotorL);
  delay(50);
  // try to find issues
  // use of the random() function
  Serial.println(" SEARCHING ");
  while(ReadSens()==false){
    RANDOM_MOVE=random(0,50); // generate a random number bitween 0 and 49
    RANDOM_MOVE%=2; // Modulo 2


    // RANDOM_MOVE = 0 OR 1

    if (RANDOM_MOVE==0) {
      TurnLeft(MotorR,MotorL);
      delay(200);
      Serial.println(" LEFT ");
    }
    TurnRight(MotorR,MotorL);
    delay(200);
    Serial.println(" RIGHT ");

  }
  delay(50);
  Serial.println(" MOVING AGAIN ");
  delay(100);
}


//___________________Preconstructed functions__________________//

//SENSOR


//return distance (cm)

double get_gp2d120x (uint16_t value) {
  if (value < 16)  value = 16;
  return 2076.0 / (value - 11.0);
}

// Boolean check road
// Average of 5 checks

boolean ReadSens(){
  int i;
  int result=0;
  // 5 * Analog read
  for(i=0;i<5;i++){
    result+=get_gp2d120x(analogRead(pin));
    delay(5);
  }

  result/=5;
  if (result<30) return false;
  return true;
}

//Controling DC Motors

// STOP MOTORS
void StopMotors(Adafruit_DCMotor *MotorR,Adafruit_DCMotor *MotorL){
  MotorL->run(RELEASE);
  MotorR->run(RELEASE);

}

// RUNNING MOTORS
void runMotors_speed_direction(Adafruit_DCMotor *MotorR,Adafruit_DCMotor *MotorL,int Speed,boolean DirectL,boolean DirectR){
  // SET SPEED
  MotorR->setSpeed(Speed);
  MotorL->setSpeed(Speed);
  delay(20);
  // APPLY THE DIRECTION
  if (DirectL==true){
    MotorL->run(FORWARD);
  }else{
    MotorL->run(BACKWARD);
  }
  if (DirectR==true){
    MotorR->run(FORWARD);
  }else{
    MotorR->run(BACKWARD);
  }

}

// Move_FORWARD
void Move_FORWARD(Adafruit_DCMotor *MotorR,Adafruit_DCMotor *MotorL){

  runMotors_speed_direction(MotorR,MotorL,200,true,true);
}

//Move_BACKWARD
void Move_BACKWARD(Adafruit_DCMotor *MotorR,Adafruit_DCMotor *MotorL){

  runMotors_speed_direction(MotorR,MotorL,130,false,false);
}

//TurnLeft
void TurnLeft(Adafruit_DCMotor *MotorR,Adafruit_DCMotor *MotorL){

  runMotors_speed_direction(MotorR,MotorL,100,false,true);
}

//TurnRight
void TurnRight(Adafruit_DCMotor *MotorR,Adafruit_DCMotor *MotorL){

  runMotors_speed_direction(MotorR,MotorL,100,true,false);
}
