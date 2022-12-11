/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\anshu                                            */
/*    Created:      Sat Dec 10 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// l1                   motor         18              
// l2                   motor         19              
// l3                   motor         20              
// r1                   motor         11              
// r2                   motor         16              
// r3                   motor         17              
// Inertial             inertial      3               
// DigitalOutG          digital_out   G               
// DigitalOutH          digital_out   H               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

float xPos;
float yPos;
float heading;

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Inertial.calibrate();
  wait(3,sec);
  //heading = Inertial.heading();

  while(true){
    // x is left/right, y is forward/backward
    Brain.Screen.printAt(15, 25, "x accel: %f", Inertial.acceleration(xaxis));
    Brain.Screen.printAt(15, 40, "y accel: %f", Inertial.acceleration(yaxis));


    wait(20, msec);
  }
}
