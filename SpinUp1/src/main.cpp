/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\catsc                                            */
/*    Created:      Wed Jul 13 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// flywheel             motor         4               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();


  //flywheel test thing
  const double pHeat = 10;
  // ^^^ power we would like to lose to heat
  double motorTorque;
  double angularSpeed;
  const double degreesToRadians = 1/57.3;
  const double flywheelGearRatio = 1;
  double motorCurrent;
  double motorVoltage;
  while(true){
    motorTorque = flywheel.torque();
    angularSpeed = flywheel.velocity(dps) * degreesToRadians * flywheelGearRatio;
    motorCurrent = flywheel.current();
    
    motorVoltage = (motorTorque * angularSpeed + pHeat) / motorCurrent;
    flywheel.spin(forward, motorVoltage, volt);

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(0, 0);
    Brain.Screen.print(motorVoltage);

    wait(10, msec);
  }
}
