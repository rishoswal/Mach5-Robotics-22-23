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
// lEncoder             encoder       A, B            
// rEncoder             encoder       A, B            
// mEncoder             encoder       C, D            
// Expander8            triport       8               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

float xPos;
float yPos;
float heading;

float previousFB;
float currentFB;
float changeFB;

float previousLR;
float currentLR;
float changeLR;

const float degreesToRadians = 2 * 3.141593 / 360.0;
const float degreesToInches = 2.75 * 3.141593 / 360.0; //with a 2.75 in diameter wheel

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Inertial.calibrate();
  wait(3,sec);
  //heading = Inertial.heading();

  while(true){
    previousFB = currentFB;
    currentFB = (lEncoder.position(degrees) + rEncoder.position(degrees)) / 2;
    changeFB = currentFB - previousFB;
    previousLR = currentLR;
    currentLR = mEncoder.position(degrees);
    changeLR = currentLR - previousLR;
    heading = Inertial.heading() * degreesToRadians;

    xPos += ((changeFB * sin(heading)) + (changeLR * cos(heading))) * degreesToInches;
    yPos += ((changeFB * cos(heading)) + (changeLR * sin(heading))) * degreesToInches;


    Brain.Screen.printAt(15, 25, "    x position: %f", xPos);
    Brain.Screen.printAt(15, 40, "    y position: %f", yPos);
    Brain.Screen.printAt(15, 55, "       heading: %f", heading);
    Brain.Screen.printAt(15, 70, "  Left Encoder: %f", lEncoder.position(degrees));
    Brain.Screen.printAt(15, 85, " Right Encoder: %f", rEncoder.position(degrees));
    Brain.Screen.printAt(15, 100, "Middle Encoder: %f", mEncoder.position(degrees));
    Brain.Screen.printAt(15, 115, "cock: %f", sin(3.1415/2));


    wait(20, msec);
  }
}
