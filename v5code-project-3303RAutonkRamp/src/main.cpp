/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// frontMogo            digital_out   A               
// backMogo1            digital_out   B
// backMogo2            digital_out   D               
// liftMotor            motor         8               
// ringIntake           motor         5             
// rightBackMotor       motor         12               
// rightMiddleMotor     motor         10               
// rightFrontMotor      motor         21               
// leftMiddleMotor      motor         16              
// leftBackMotor        motor         18              
// leftFrontMotor       motor         1               
// ---- END VEXCODE CONFIGURED DEVICES ----



#include "vex.h"
#include <iostream>

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonLiftFunction(double speed, double dist){
  liftMotor.rotateTo(dist, rotationUnits::deg, speed, velocityUnits::pct);
}

void conveyorIntake(double speed){
  if(speed!=0){
    ringIntake.spin(directionType::fwd, speed, percentUnits::pct);
  }
  else{
    ringIntake.stop(coast);
  }
}

// PID$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

//PID variables - must modify to work with your robot - also need several values depending on how many cubes
double kP = 0.07; // 0.08, 0.075, 0.06 These are other kP that work well depending on how many cubes
double kI = 0.00; // not using this, but you can experiment if you'd like
double kD = 0.2;     //.12, 0.2  These are other kD that work well depending on how many cubes
double kM = 0.18;    //.1  This is another value that works well depending on how many cubes
double turnkP = 0.3; // 0.3
double turnkI = 0.0;
double turnkD = 0.00;
int maxTurnIntegral = 300; // These cap the integrals
int maxIntegral = 300;
int integralBound =3; // If error is outside the bounds, then apply the integral. This is a buffer with +-integralBound degrees

// Autonomous Settings
int desiredValue = 0;
int desiredTurnValue = 0;

int error;          // SensorValue - DesiredValue : Position
int prevError = 0;  // Position 20 miliseconds ago
int derivative;     // error - prevError : Speed
int totalError = 0; // totalError = totalError + error

int turnError;          // SensorValue - DesiredValue : Position
int turnPrevError = 0;  // Position 20 miliseconds ago
int turnDerivative;     // error - prevError : Speed
int turnTotalError = 0; // totalError = totalError + error


// Variables modified for use
bool enableDrivePID = true;
bool resetDriveSensors = false;
int x = 0;
int speed;
int turnspeed;
int z;

// Pasted from a C++ resource
double signnum_c(double x) {
  if (x > 0.0)
    return 1.0;
  if (x < 0.0)
    return -1.0;
  return x;
}

//This is the PID Task 
int drivePID() {

  while (enableDrivePID) {

    if (resetDriveSensors) {
      resetDriveSensors = false;
      leftFrontMotor.setPosition(0, degrees);
      rightFrontMotor.setPosition(0, degrees);
    }

    // Get the position of both motors
    int leftMotorPosition = leftFrontMotor.position(degrees);
    int rightMotorPosition = rightFrontMotor.position(degrees);

    //double Odom =  Odometer.position(deg);

    ///////////////////////////////////////////
    // Lateral movement PID
    /////////////////////////////////////////////////////////////////////
    // 

    error = desiredValue - leftMotorPosition; // forward with postitive encoder value 
    //error = leftMotorPosition - desiredValue; //reverse with negative encoder value

    // Derivative
    derivative = error - prevError;

    // Integral
    if (abs(error) > integralBound) {
      totalError += error;  // totalError = totalError + error;
    } else {
      totalError = 0;
    }
    
    // This would cap the integral
    totalError = abs(totalError) > maxIntegral ? signnum_c(totalError) * maxIntegral : totalError;
    // this is short hand if else in C++ that is capable of setting a variable based on the if else decision. See below
    /*
    if (abs(totalError) > maxIntegral)
    {
      singnum_c(totalError)*maxIntegral;
    }
    else
    {
      totalError;
    }
    */

    double lateralMotorPower = error * kP + derivative * kD + totalError * kI;
    /////////////////////////////////////////////////////////////////////

    if (z == 0) {
      if (lateralMotorPower > speed) {
        lateralMotorPower = speed;
      }
    }
    else if(z == 1) {
      if(lateralMotorPower < speed) {
        lateralMotorPower = speed;
      }
    }

    ///////////////////////////////////////////
    // Turning movement PID
    /////////////////////////////////////////////////////////////////////
    // Get difference of the two motors
    int turnDifference = leftMotorPosition - rightMotorPosition;
    int turnDifference1 = leftMotorPosition - rightMotorPosition;

    // Potential
    // turnError = turnDifference - desiredTurnValue;
    turnError = desiredTurnValue - turnDifference;

    // Derivative
    turnDerivative = turnError - turnPrevError;

    // Integral
    if (abs(error) < integralBound) {
      turnTotalError += turnError;
    } else {
      turnTotalError = 0;
    }
    // turnTotalError = turnTotalError + turnError;

    // This would cap the integral
    turnTotalError = abs(turnTotalError) > maxIntegral ? signnum_c(turnTotalError) * maxIntegral : turnTotalError;

    double turnMotorPower = turnError * turnkP + turnDerivative * turnkD + turnTotalError * turnkI;

    if (z == 0) {
      if (turnMotorPower > turnspeed) {
        turnMotorPower = turnspeed;
      }
    }
    else if(z == 1) {
      if(turnMotorPower < turnspeed) {
        turnMotorPower = turnspeed;
      }
    }    
    /////////////////////////////////////////////////////////////////////

    if (x == 0) {
      leftFrontMotor.spin(forward, lateralMotorPower - turnDifference1 * kM, voltageUnits::volt);
      leftMiddleMotor.spin(forward, lateralMotorPower - turnDifference1 * kM, voltageUnits::volt);
      leftBackMotor.spin(forward, lateralMotorPower - turnDifference1 * kM, voltageUnits::volt);
      rightFrontMotor.spin(forward, lateralMotorPower + turnDifference1 * kM, voltageUnits::volt);
      rightMiddleMotor.spin(forward, lateralMotorPower + turnDifference1 * kM, voltageUnits::volt);
      rightBackMotor.spin(forward, lateralMotorPower + turnDifference1 * kM, voltageUnits::volt);
    }

    else if (x == 1) {
      leftFrontMotor.spin(forward, turnMotorPower , voltageUnits::volt);
      leftMiddleMotor.spin(forward, turnMotorPower , voltageUnits::volt);
      leftBackMotor.spin(forward, turnMotorPower, voltageUnits::volt);
      rightFrontMotor.spin(forward, -turnMotorPower, voltageUnits::volt);
      rightMiddleMotor.spin(forward, -turnMotorPower, voltageUnits::volt);
      rightBackMotor.spin(forward, -turnMotorPower, voltageUnits::volt);
    }

    printf("LE %d\tRE %d\terror %d\tturnerror %d\tpreviouserror %d\tderivative %d\ttotalerror %d\tLMP %f\tTMP %f\n", leftMotorPosition, rightMotorPosition, error, turnError, prevError, derivative, totalError, lateralMotorPower, turnMotorPower);
    // This displays variables. Inside the quotes: First give the variable a name, then a space, then either %d for integers or %f for decimals, then \t to tab over to the next variable name. Repeat for other variables.
    // Outside the quotes and after the comma: make sure the the variable you want to display is matched up the variable name you gave it in the quotes. Separate all variables you want to display with a comma between them.

    prevError = error;
    turnPrevError = turnError;
    vex::task::sleep(20);        // don't hog CPU. This makes all tasks run faster.
  }
  return 1;
}




void moveForward(double dist, double s, double sleepTime){
  x = 0;
  z = 0;
  speed = s;
  resetDriveSensors = true;
  desiredValue = dist;
  vex::task::sleep(sleepTime);    // the drivePID task will run for only 2ooo ms.
  leftBackMotor.spin(directionType:: rev, speed, velocityUnits::pct);
  leftMiddleMotor.spin(directionType:: fwd, speed, velocityUnits::pct);
  leftFrontMotor.spin(directionType:: fwd, speed, velocityUnits::pct);
  rightBackMotor.spin(directionType::fwd, speed, velocityUnits::pct);
  rightMiddleMotor.spin(directionType::fwd, speed, velocityUnits::pct);
  rightFrontMotor.spin(directionType::rev, speed, velocityUnits::pct);
}

void moveBack(double dist, double s, double sleepTime){
  x = 0;
  z = 1;
  speed = s;
  resetDriveSensors = true;
  desiredValue = -dist;
  vex::task::sleep(sleepTime);    // the drivePID task will run for only 2ooo ms.
  leftBackMotor.spin(directionType:: rev, speed, velocityUnits::pct);
  leftMiddleMotor.spin(directionType:: fwd, speed, velocityUnits::pct);
  leftFrontMotor.spin(directionType:: fwd, speed, velocityUnits::pct);
  rightBackMotor.spin(directionType::fwd, speed, velocityUnits::pct);
  rightMiddleMotor.spin(directionType::fwd, speed, velocityUnits::pct);
  rightFrontMotor.spin(directionType::rev, speed, velocityUnits::pct);

}

void turnLeft(double ts, double turnval, double sleepTime){
  x = 1;            //X contols turning; use 1 when turning, and 0 when going straight
  z = 1;            //Z controls direction; use 0 when using positive integers, and 1 when using negative ones.
  turnspeed = -ts;
  resetDriveSensors = true;   // reset encoders to 0
  //intakeSpin(100);
  desiredTurnValue = -turnval;     // desired turn distance = 370 degrees. Make negative to turn other direction
  vex::task::sleep(sleepTime);     // the drivePID task will run for only 2ooo ms.
  leftBackMotor.spin(directionType:: rev, speed, velocityUnits::pct);
  leftMiddleMotor.spin(directionType:: fwd, speed, velocityUnits::pct);
  leftFrontMotor.spin(directionType:: fwd, speed, velocityUnits::pct);
  rightBackMotor.spin(directionType::fwd, speed, velocityUnits::pct);
  rightMiddleMotor.spin(directionType::fwd, speed, velocityUnits::pct);
  rightFrontMotor.spin(directionType::rev, speed, velocityUnits::pct);
}

void turnRight(double ts, double turnval, double sleepTime){
  x = 1;            //X contols turning; use 1 when turning, and 0 when going straight
  z = 0;            //Z controls direction; use 0 when using positive integers, and 1 when using negative ones.
  turnspeed = ts;
  resetDriveSensors = true;   // reset encoders to 0
  //intakeSpin(100);
  desiredTurnValue = turnval;     // desired turn distance = 370 degrees. Make negative to turn other direction
  vex::task::sleep(sleepTime);     // the drivePID task will run for only 2ooo ms.
  leftBackMotor.spin(directionType:: rev, speed, velocityUnits::pct);
  leftMiddleMotor.spin(directionType:: fwd, speed, velocityUnits::pct);
  leftFrontMotor.spin(directionType:: fwd, speed, velocityUnits::pct);
  rightBackMotor.spin(directionType::fwd, speed, velocityUnits::pct);
  rightMiddleMotor.spin(directionType::fwd, speed, velocityUnits::pct);
  rightFrontMotor.spin(directionType::rev, speed, velocityUnits::pct);
}

bool frontClawClosed = true;

void frontMogoControlAuton(){
  if(frontClawClosed==true){
    frontMogo.set(true);
    frontClawClosed = false;
  }
  else{
    frontMogo.set(false);
    frontClawClosed = true;
  }
}

bool backClawClosed = false;

void backMogoControlAuton(){
  // if(Controller1.ButtonL2.pressing()){
  //   backClawClosed =! backClawClosed;
  //   wait(300, msec);
  // }
  if(backClawClosed==true){
    backMogo1.set(false);
    backMogo2.set(true);
    backClawClosed = false;
    
  }
  else{
    backMogo1.set(true);
    backMogo2.set(false);
    backClawClosed = true;
  }
}

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................






  vex::task polarExpress(drivePID); //Call the task. Give it any name you want. Put the name of the task inside the parenthesi
  // frontMogo.set(false);
  // moveForward(1000, 100, 1000);
  // liftMotor.startSpinTo(350, degrees);
  // moveBack(-1040, -100, 1500);
  // ringIntake.startSpinTo(-140, degrees);
  // turnLeft(-50, -450, 1500);
  // liftMotor.startSpinTo(1500, degrees);
  // moveBack(-190, -200, 1500);
  // backMogo1.set(true);
  // backMogo2.set(true);
  // vex::task::sleep(500);
  // backMogo1.set(false);
  // backMogo2.set(false);
  // conveyorIntake(100);
  // vex::task::sleep(1500);
  // moveForward(300, 50, 1000);


//DRIVE FRONT TO GRAB NEUTRAL MOGO

  //backMogo.set(true);
  backMogo1.set(true);
  backMogo2.set(true);
  
  x = 0;            //X contols turning; use 1 when turning, and 0 when going straight
  z = 0;            //Z controls direction; use 0 when using positive integers, and 1 when using negative ones.
  speed = 100;
  resetDriveSensors = true;   // reset encoders to 0
  //intakeSpin(100);
  desiredValue = 1000;     // desired turn distance = 370 degrees. Make negative to turn other direction
  vex::task::sleep(1000);    // the drivePID task will run for only 2ooo ms.
  leftBackMotor.spin(directionType:: rev, speed, velocityUnits::pct);
  leftMiddleMotor.spin(directionType:: fwd, speed, velocityUnits::pct);
  leftFrontMotor.spin(directionType:: fwd, speed, velocityUnits::pct);
  rightBackMotor.spin(directionType::fwd, speed, velocityUnits::pct);
  rightMiddleMotor.spin(directionType::fwd, speed, velocityUnits::pct);
  rightFrontMotor.spin(directionType::rev, speed, velocityUnits::pct);
  //intakeRight.spin(fwd, speed,pct);
  //intakeLeft.spin(fwd, speed,pct);

  //vex::task::sleep(500);

  frontMogoControlAuton();

  vex::task::sleep(250);

  liftMotor.startSpinTo(350, degrees);

  //frontMogo.set(true);

//DRIVE BACK TO HOME ZONE

  x = 0;            //X contols turning; use 1 when turning, and 0 when going straight
  z = 1;            //Z controls direction; use 0 when using positive integers, and 1 when using negative ones.
  speed = -100;
  resetDriveSensors = true;   // reset encoders to 0
  //intakeSpin(100);
  desiredValue = -1040;     // desired turn distance = 370 degrees. Make negative to turn other direction
  vex::task::sleep(1500);    // the drivePID task will run for only 2ooo ms.
  leftBackMotor.spin(directionType:: rev, speed, velocityUnits::pct);
  leftMiddleMotor.spin(directionType:: fwd, speed, velocityUnits::pct);
  leftFrontMotor.spin(directionType:: fwd, speed, velocityUnits::pct);
  rightBackMotor.spin(directionType::fwd, speed, velocityUnits::pct);
  rightMiddleMotor.spin(directionType::fwd, speed, velocityUnits::pct);
  rightFrontMotor.spin(directionType::rev, speed, velocityUnits::pct);

  ringIntake.startSpinTo(-140, degrees);


//turn left TOWARDS PLATFORM

  x = 1;            //X contols turning; use 1 when turning, and 0 when going straight
  z = 1;            //Z controls direction; use 0 when using positive integers, and 1 when using negative ones.
  turnspeed = -50;
  resetDriveSensors = true;   // reset encoders to 0
  //intakeSpin(100);
  desiredTurnValue = -450;     // desired turn distance = 370 degrees. Make negative to turn other direction
  vex::task::sleep(1500);    // the drivePID task will run for only 2ooo ms.
  leftBackMotor.spin(directionType:: rev, speed, velocityUnits::pct);
  leftMiddleMotor.spin(directionType:: fwd, speed, velocityUnits::pct);
  leftFrontMotor.spin(directionType:: fwd, speed, velocityUnits::pct);
  rightBackMotor.spin(directionType::fwd, speed, velocityUnits::pct);
  rightMiddleMotor.spin(directionType::fwd, speed, velocityUnits::pct);
  rightFrontMotor.spin(directionType::rev, speed, velocityUnits::pct);


  liftMotor.startSpinTo(1500, degrees);


//drive back TO GRAB HOME GOAL

  x = 0;            //X contols turning; use 1 when turning, and 0 when going straight
  z = 1;            //Z controls direction; use 0 when using positive integers, and 1 when using negative ones.
  speed = -200;
  resetDriveSensors = true;   // reset encoders to 0
  //intakeSpin(100);
  desiredValue = -200;     // desired turn distance = 370 degrees. Make negative to turn other direction
  vex::task::sleep(1500);    // the drivePID task will run for only 2ooo ms.
  leftBackMotor.spin(directionType:: rev, speed, velocityUnits::pct);
  leftMiddleMotor.spin(directionType:: fwd, speed, velocityUnits::pct);
  leftFrontMotor.spin(directionType:: fwd, speed, velocityUnits::pct);
  rightBackMotor.spin(directionType::fwd, speed, velocityUnits::pct);
  rightMiddleMotor.spin(directionType::fwd, speed, velocityUnits::pct);
  rightFrontMotor.spin(directionType::rev, speed, velocityUnits::pct);

  
  //vex::task::sleep(500);
  backMogo1.set(false);
  backMogo2.set(false);


  conveyorIntake(100);

  //ringIntake.startSpinTo(-150, degrees);

  //vex::task::sleep(3000);    

  vex::task::sleep(1500);

  //DRIVE FRONT TO PICK UP MATCHLOADS

  x = 0;            //X contols turning; use 1 when turning, and 0 when going straight
  z = 0;            //Z controls direction; use 0 when using positive integers, and 1 when using negative ones.
  speed = 50;
  resetDriveSensors = true;   // reset encoders to 0
  //intakeSpin(100);
  desiredValue = 300;     // desired turn distance = 370 degrees. Make negative to turn other direction
  vex::task::sleep(1000);    // the drivePID task will run for only 2ooo ms.
  leftBackMotor.spin(directionType:: rev, speed, velocityUnits::pct);
  leftMiddleMotor.spin(directionType:: fwd, speed, velocityUnits::pct);
  leftFrontMotor.spin(directionType:: fwd, speed, velocityUnits::pct);
  rightBackMotor.spin(directionType::fwd, speed, velocityUnits::pct);
  rightMiddleMotor.spin(directionType::fwd, speed, velocityUnits::pct);
  rightFrontMotor.spin(directionType::rev, speed, velocityUnits::pct);
  polarExpress.stop();
  polarExpress.stop();
  // x = 0;            //X contols turning; use 1 when turning, and 0 when going straight
  // z = 0;            //Z controls direction; use 0 when using positive integers, and 1 when using negative ones.
  // speed = 100;
  // resetDriveSensors = true;   // reset encoders to 0
  // //intakeSpin(100);
  // desiredValue = 640;     // desired turn distance = 370 degrees. Make negative to turn other direction
  // vex::task::sleep(1500);    // the drivePID task will run for only 2ooo ms.
  // leftBackMotor.spin(directionType:: rev, speed, velocityUnits::pct);
  // leftMiddleMotor.spin(directionType:: fwd, speed, velocityUnits::pct);
  // leftFrontMotor.spin(directionType:: fwd, speed, velocityUnits::pct);
  // rightBackMotor.spin(directionType::fwd, speed, velocityUnits::pct);
  // rightMiddleMotor.spin(directionType::fwd, speed, velocityUnits::pct);
  // rightFrontMotor.spin(directionType::rev, speed, velocityUnits::pct);
  // //intakeRight.spin(fwd, speed,pct);
  // //intakeLeft.spin(fwd, speed,pct);

  // x = 0;            //X contols turning; use 1 when turning, and 0 when going straight
  // z = 1;            //Z controls direction; use 0 when using positive integers, and 1 when using negative ones.
  // speed = -100;
  // resetDriveSensors = true;   // reset encoders to 0
  // //intakeSpin(100);
  // desiredValue = -640;     // desired turn distance = 370 degrees. Make negative to turn other direction
  // vex::task::sleep(1500);    // the drivePID task will run for only 2ooo ms.
  // leftBackMotor.spin(directionType:: rev, speed, velocityUnits::pct);
  // leftMiddleMotor.spin(directionType:: fwd, speed, velocityUnits::pct);
  // leftFrontMotor.spin(directionType:: fwd, speed, velocityUnits::pct);
  // rightBackMotor.spin(directionType::fwd, speed, velocityUnits::pct);
  // rightMiddleMotor.spin(directionType::fwd, speed, velocityUnits::pct);
  // rightFrontMotor.spin(directionType::rev, speed, velocityUnits::pct);

  // x = 1;            //X contols turning; use 1 when turning, and 0 when going straight
  // z = 1;            //Z controls direction; use 0 when using positive integers, and 1 when using negative ones.
  // turnspeed = -100;
  // resetDriveSensors = true;   // reset encoders to 0
  // //intakeSpin(100);
  // desiredTurnValue = -470;     // desired turn distance = 370 degrees. Make negative to turn other direction
  // vex::task::sleep(1500);    // the drivePID task will run for only 2ooo ms.
  // leftBackMotor.spin(directionType:: rev, speed, velocityUnits::pct);
  // leftMiddleMotor.spin(directionType:: fwd, speed, velocityUnits::pct);
  // leftFrontMotor.spin(directionType:: fwd, speed, velocityUnits::pct);
  // rightBackMotor.spin(directionType::fwd, speed, velocityUnits::pct);
  // rightMiddleMotor.spin(directionType::fwd, speed, velocityUnits::pct);
  // rightFrontMotor.spin(directionType::rev, speed, velocityUnits::pct);

  // x = 1;            //X contols turning; use 1 when turning, and 0 when going straight
  // z = 0;            //Z controls direction; use 0 when using positive integers, and 1 when using negative ones.
  // turnspeed = 100;
  // resetDriveSensors = true;   // reset encoders to 0
  // //intakeSpin(100);
  // desiredTurnValue = 470;     // desired turn distance = 370 degrees. Make negative to turn other direction
  // vex::task::sleep(1500);    // the drivePID task will run for only 2ooo ms.
  // leftBackMotor.spin(directionType:: rev, speed, velocityUnits::pct);
  // leftMiddleMotor.spin(directionType:: fwd, speed, velocityUnits::pct);
  // leftFrontMotor.spin(directionType:: fwd, speed, velocityUnits::pct);
  // rightBackMotor.spin(directionType::fwd, speed, velocityUnits::pct);
  // rightMiddleMotor.spin(directionType::fwd, speed, velocityUnits::pct);
  // rightFrontMotor.spin(directionType::rev, speed, velocityUnits::pct);

 



  // autonLiftFunction(100, 80);
  // conveyorIntake(70);
  // wait(50, timeUnits::sec);
  // conveyorIntake(0);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

const unsigned int trueSpeed[128] = {
     0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
     0 , 21, 21, 21, 22, 22, 22, 23, 24, 24,
     25, 25, 25, 25, 26, 27, 27, 28, 28, 28,
     28, 29, 30, 30, 30, 31, 31, 32, 32, 32,
     33, 33, 34, 34, 35, 35, 35, 36, 36, 37,
     37, 37, 37, 38, 38, 39, 39, 39, 40, 40,
     41, 41, 42, 42, 43, 44, 44, 45, 45, 46,
     46, 47, 47, 48, 48, 49, 50, 50, 51, 52,
     52, 53, 54, 55, 56, 57, 57, 58, 59, 60,
     61, 62, 63, 64, 65, 66, 67, 67, 68, 70,
     71, 72, 72, 73, 74, 76, 77, 78, 79, 79,
     80, 83, 84, 86, 86, 87, 87, 88, 88, 90,
     90, 90, 95, 95, 95,100,100,100};

#define slowDriveBackBtn Controller1.ButtonA.pressing()
#define macroDriveBtn Controller1.ButtonLeft.pressing()

//---Y Definitions---//
#define Y_leftJoy Controller1.Axis3.value()
#define Y_rightJoy Controller1.Axis2.value()

//---X Definitions---//
#define X_leftJoy Controller1.Axis4.value()
#define X_rightJoy Controller1.Axis1.value()

bool useTrueSpeed = true;

int leftSpeed = 0;
int rightSpeed = 0;

void rightBaseSpin(int speed = 0){
  rightSpeed = speed;
  if(rightSpeed != 0){
    rightFrontMotor.spin(vex::directionType::fwd, rightSpeed, vex::velocityUnits::pct);
    rightMiddleMotor.spin(vex::directionType::fwd, rightSpeed, vex::velocityUnits::pct);
    rightBackMotor.spin(vex::directionType::fwd, rightSpeed, vex::velocityUnits::pct);
  }else{
    rightFrontMotor.stop(vex::brakeType::coast); 
    rightMiddleMotor.stop(vex::brakeType::coast);
    rightBackMotor.stop(vex::brakeType::coast);
  }
}

void leftBaseSpin(int speed = 0){
  leftSpeed = speed;
  if(leftSpeed != 0){
    leftFrontMotor.spin(vex::directionType::fwd, leftSpeed, vex::velocityUnits::pct);
    leftMiddleMotor.spin(vex::directionType::fwd, leftSpeed, vex::velocityUnits::pct);
    leftBackMotor.spin(vex::directionType::fwd, leftSpeed, vex::velocityUnits::pct);
  }else{
    leftFrontMotor.stop(vex::brakeType::coast);  
    leftMiddleMotor.stop(vex::brakeType::coast);  
    leftBackMotor.stop(vex::brakeType::coast);  
  }
}

void Spin(int leftSpeed, int rightSpeed){
  rightSpeed = rightSpeed;
  leftSpeed = leftSpeed;
  leftBaseSpin(leftSpeed);
  rightBaseSpin(rightSpeed);
}

void userControl(int bufferSize = 5, bool Stop = false){
 if(macroDriveBtn){
    Spin(45, 45);   
 }else if(slowDriveBackBtn){
   Spin(-30, -30);   
 }
 else{
   if(useTrueSpeed){
     if(abs(Y_rightJoy)>bufferSize){
       if(Y_rightJoy>0){
         rightBaseSpin(trueSpeed[abs(Y_rightJoy)]);
       }else{
         rightBaseSpin(-trueSpeed[abs(Y_rightJoy)]);
       }
     }else{
        rightBaseSpin(0);
     } 
     if(abs(Y_leftJoy)>bufferSize){
       if(Y_leftJoy>0){
         leftBaseSpin(trueSpeed[abs(Y_leftJoy)]);
       }else{
         leftBaseSpin(-trueSpeed[abs(Y_leftJoy)]);
       }
     }else{
       leftBaseSpin(0);
     } 
   }else{
     if(abs(Y_rightJoy)>bufferSize){
       rightBaseSpin(Y_rightJoy);
     }else{
       rightBaseSpin(0);
     } 
     if(abs(Y_leftJoy)>bufferSize){
       leftBaseSpin(Y_leftJoy);
     }else{
       leftBaseSpin(0);
     } 
   }
  
 }  
}

void liftControl(){
  if(Controller1.ButtonR1.pressing()){
    liftMotor.spin(forward,100,pct);
  }
  else if(Controller1.ButtonR2.pressing()){
    liftMotor.spin(reverse,100,pct);
  }
  else{
    liftMotor.stop(hold);
  }
}

bool ringIntakeOn = false;

void intakeControl(){
  if(Controller1.ButtonL1.pressing()){
    // if(ringIntakeOn == true){
    //   ringIntakeOn = false;
    // } else {
    //   ringIntakeOn = true;
    // }
    ringIntakeOn =! ringIntakeOn;
    wait(300, msec);
  }
  if(ringIntakeOn==true){
    ringIntake.spin(forward,100,pct);
  }
  else if(Controller1.ButtonDown.pressing()){
    ringIntake.spin(reverse,100,pct);
  }
  else{
    ringIntake.stop(hold);
  }
}

// bool frontClawClosed = false;

void frontMogoControl(){
  if(Controller1.ButtonA.pressing()){
    frontClawClosed =! frontClawClosed;
    wait(300, msec);
  }
  else if(frontClawClosed==true){
    frontMogo.set(true);
  }
  else{
    frontMogo.set(false);
  }
}

// bool backClawClosed = false;

void backMogoControl(){
  if(Controller1.ButtonL2.pressing()){
    backClawClosed =! backClawClosed;
    wait(300, msec);
  }
  else if(backClawClosed==true){
    backMogo1.set(false);
    backMogo2.set(false);
    
  }
  else{
    backMogo1.set(true);
    backMogo2.set(true);
  }
}


void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    liftControl();
    backMogoControl();
    frontMogoControl();
    intakeControl();
    userControl();
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
