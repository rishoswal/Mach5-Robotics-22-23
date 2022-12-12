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

const float degreesToRadians = 2 * 3.141593 / 360.0;
const float degreesToInches = 2.75 * 3.141593 / 360.0; //with a 2.75 in diameter wheel

void odometryInertial(){
  float xPos = 0;
  float yPos = 0;
  float heading;

  float previousFB;
  float currentFB = 0;
  float changeFB;

  float previousLR;
  float currentLR = 0;
  float changeLR;

  Inertial.calibrate();
  wait(3,sec);

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

    wait(20, msec);
  }
}

void odometry(){
  float xPos = 0;
  float yPos = 0;
  float heading = 0;

  float previousL;
  float currentL = 0;
  float changeL;

  float previousR;
  float currentR = 0;
  float changeR;

  float previousM;
  float currentM = 0;
  float changeM;

  float changeFB;

  const float robotDiameter = 6.875;

  wait(1, sec);

  while(true){
    previousL = currentL;
    currentL = lEncoder.position(degrees);
    changeL = currentL - previousL;
    
    previousR = currentR;
    currentR = rEncoder.position(degrees);
    changeR = currentR - previousR;

    previousM = currentM;
    currentM = mEncoder.position(degrees);
    changeM = (currentM - previousM) * degreesToInches;

    heading = (currentL - currentR) * degreesToInches / robotDiameter;
    changeFB = (changeL + changeR) * degreesToInches / 2;

    xPos += ((changeFB * sin(heading)) + (changeM * cos(heading)));
    yPos += ((changeFB * cos(heading)) - (changeM * sin(heading)));

    Brain.Screen.printAt(15, 25, "      x position: %f", xPos);
    Brain.Screen.printAt(15, 40, "      y position: %f", yPos);
    Brain.Screen.printAt(15, 55, "         heading: %f", heading);
    Brain.Screen.printAt(15, 70, "    Left Encoder: %f", lEncoder.position(degrees));
    Brain.Screen.printAt(15, 85, "   Right Encoder: %f", rEncoder.position(degrees));
    Brain.Screen.printAt(15, 100, "  Middle Encoder: %f", mEncoder.position(degrees));
    Brain.Screen.printAt(15, 115, "Forward/backward: %f", changeFB);

    wait(20, msec);
  }
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  odometry();  
}
