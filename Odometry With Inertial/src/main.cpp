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
// shoota               digital_out   G               
// DigitalOutH          digital_out   H               
// lEncoder             encoder       A, B            
// rEncoder             encoder       A, B            
// mEncoder             encoder       C, D            
// Expander8            triport       8               
// flywheel             motor         9               
// Controller1          controller                    
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "math.h"

using namespace vex;

const float degreesToRadians = 2 * 3.141593 / 360.0;
const float degreesToInches = 2.75 * 3.141593 / 360.0; //with a 2.75 in diameter wheel

motor_group lDrive(l1, l2, l3);
motor_group rDrive(r1, r2, r3);

float xPos = 0;
float yPos = 0;
float heading;
float headingD;

void turn(float angle){ //function for turning. Spins with a speed cap of 36 percent, uses proportional correction
  float error = angle-(headingD*1);
  while(fabs(error)>2){ //exits loop if error <2 and rotational speed <1
    error = angle-(headingD);//calculates error value
    if(fabs(error)>54){ //if error is greater than 50, use proportional correction. if not, turn at 36 percent speed
      lDrive.spin(forward,36*(fabs(error)/error),percent);
      rDrive.spin(reverse,36*(fabs(error)/error),percent);
    }else{
      lDrive.spin(forward,error*0.73,percent);
      rDrive.spin(reverse,error*0.73,percent);
    }

    wait(20, msec);
  }
  lDrive.stop();
  rDrive.stop();
}

void shoot(float goalOffset){
  float goalAngle = 90 - (atan2(-54 - yPos, 54 - xPos) / degreesToRadians) + goalOffset;
  Controller1.Screen.print(goalAngle);
  turn(goalAngle);
  shoota.set(true);
  wait(0.3, sec);
  shoota.set(false);
  lDrive.spin(forward);
  rDrive.spin(forward);
}

void odometryInertial(){

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
    headingD = Inertial.heading(degrees);
    heading = headingD * degreesToRadians;

    xPos += ((changeFB * sin(heading)) + (changeLR * cos(heading))) * degreesToInches;
    yPos += ((changeFB * cos(heading)) + (changeLR * sin(heading))) * degreesToInches;

    Brain.Screen.printAt(15, 25, "    x position: %f", xPos);
    Brain.Screen.printAt(15, 40, "    y position: %f", yPos);
    Brain.Screen.printAt(15, 55, "       heading: %f", headingD);
    Brain.Screen.printAt(15, 70, "  Left Encoder: %f", lEncoder.position(degrees));
    Brain.Screen.printAt(15, 85, " Right Encoder: %f", rEncoder.position(degrees));
    Brain.Screen.printAt(15, 100, "Middle Encoder: %f", mEncoder.position(degrees));

    wait(20, msec);
  }
}

void odometry(){

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

  const float robotDiameter = 6.6;
  const float leftOffset = 1;

  wait(1, sec);

  lDrive.spin(forward);
  rDrive.spin(forward);

  while(true){
    previousL = currentL;
    currentL = lEncoder.position(degrees) * leftOffset;
    changeL = currentL - previousL;
    
    previousR = currentR;
    currentR = rEncoder.position(degrees);
    changeR = currentR - previousR;

    previousM = currentM;
    currentM = mEncoder.position(degrees);
    changeM = (currentM - previousM) * degreesToInches;

    heading = (currentL - currentR) * degreesToInches / robotDiameter;
    changeFB = (changeL + changeR) * degreesToInches / 2;
    headingD = heading / degreesToRadians;

    xPos += ((changeFB * sin(heading)) + (changeM * cos(heading)));
    yPos += ((changeFB * cos(heading)) - (changeM * sin(heading)));

    Brain.Screen.printAt(15, 25, "      x position: %f", xPos);
    Brain.Screen.printAt(15, 40, "      y position: %f", yPos);
    Brain.Screen.printAt(15, 55, "   heading (rad): %f", heading);
    Brain.Screen.printAt(15, 70, "   heading (deg): %f", headingD);
    Brain.Screen.printAt(15, 85, "    Left Encoder: %f", currentL);
    Brain.Screen.printAt(15, 100, "   Right Encoder: %f", currentR);
    Brain.Screen.printAt(15, 115, "  Middle Encoder: %f", currentM);
    Brain.Screen.printAt(15, 130, "  L/R Difference: %f", currentL - currentR);

    this_thread::sleep_for(20);
  }
}

void autoPower(){
  float goalDistance;

  while(true){
    goalDistance = sqrt(pow(xPos - 54, 2) + pow(yPos + 54, 2));

    flywheel.spin(forward, goalDistance * 0.04 + 5, volt);

    wait(50,msec);
  }
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  thread runOdom(odometryInertial);

  while(true){
    lDrive.setVelocity((Controller1.Axis2.position() + Controller1.Axis4.position()) / 2, percent);
    rDrive.setVelocity((Controller1.Axis2.position() - Controller1.Axis4.position()) / 2, percent);

    if(Controller1.ButtonA.pressing()){
      shoot(-10);
      waitUntil(!Controller1.ButtonA.pressing());
    }

    if(Controller1.ButtonB.pressing()){
      thread startFlywheel(autoPower);
      waitUntil(!Controller1.ButtonB.pressing());
    }

    wait(20, msec);
  }
}
