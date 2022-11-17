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
// Flywheel             motor         9               
// Rotation             rotation      1               
// Controller1          controller                    
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
double kP = 0.5;
double kI = 0.0;
double kD = 0.01;

int flySpeed;

int error;
int prevError;
int derivative;
int integral;
int diff;
bool enableVisionPID = false;


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

int FlyWheelPID() {
  //Brain.Screen.print("test");
  //Brain.Screen.print(enableVisionPID);

  while (enableVisionPID) {
    diff = 0;
    error = 3600 - Rotation.velocity(rpm);
    // integral = integral + error;
    derivative = error - prevError;
    diff = (error * kP) + (derivative * kD);
    prevError = error;
    }
    // //Brain.Screen.clearLine();
    // //Brain.Screen.print(diff);
    vex::task::sleep(20);
  

  return 1;
}
// 2^flywheelspeed
// motor = 120 rpm
void FlyWheel(){

  while (Rotation.velocity(rpm) < 3400) {
    int flyrotation = Rotation.velocity(rpm);
    if (flyrotation < 1800) {
      Flywheel.spin(fwd, Flywheel.velocity(rpm) + (Flywheel.velocity(rpm) *2 +1), rpm);
      //Brain.Screen.clearLine();
    //Brain.Screen.print(diff);
    } else {
      Flywheel.spin(fwd, Flywheel.velocity(rpm) + ((600-Flywheel.velocity(rpm))/2), rpm);
      //Brain.Screen.clearLine();
      //Brain.Screen.print(diff);
    }
  }
}

float convertYToX(float yvalue){
  int xvalue = ((log((600/yvalue) - 1))/-0.006)+480;
  if (yvalue ==0){
    xvalue = 0;
  }
  return(xvalue);
}

void Test(int startSpeed, int finalSpeed){
//Flywheel.spin(forward,50, rpm);
//wait(5, sec);
  float counter = convertYToX(startSpeed/6);
  Brain.Screen.print(counter);
  while (Flywheel.velocity(rpm)*6 < finalSpeed) {
    int flyrotation = Rotation.velocity(rpm);
    Flywheel.setVelocity(600/(1+ pow(2.71828, (-0.006)*(counter - 480))), rpm);
    Flywheel.spin(forward);
    //Flywheel.spin(forward, 600/(1+ pow(2.71828, (-0.006)*(counter - 480))) , rpm);
    counter += 20;
    wait(0.7, sec);
    Brain.Screen.clearLine();
    Brain.Screen.print(Flywheel.velocity(rpm)*6);
  }
  Flywheel.spin(forward);
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

void autonomous(void) {

  
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
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

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {

  if (Controller1.ButtonR2.pressing()){
      Test(0, 1000);
    } else {
      //Flywheel.stop(coast);
    }
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

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
