/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\anshu                                            */
/*    Created:      Wed Nov 16 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Flywheel             motor         9               
// Controller1          controller                    
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// 1:1 motor spins 18x faster than a green catridge

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  int volts = 0;
  double heat;
  while(true){
    if(Controller1.ButtonRight.pressing()&&volts<5){
      volts++;
    }
    if(Controller1.ButtonLeft.pressing()&&volts>-5){
      volts--;
    }
    if(Controller1.ButtonDown.pressing()){
      volts=0;
    }
    Flywheel.spin(forward, volts, volt);

    heat = (Flywheel.voltage()*Flywheel.current()) - (Flywheel.torque()*Flywheel.velocity(dps)*0.3142);

    Brain.Screen.clearScreen();
    Brain.Screen.printAt(25, 25, "%d", volts);
    Brain.Screen.printAt(25, 40, "%f", Flywheel.voltage());
    Brain.Screen.printAt(25, 55, "%f", Flywheel.power());
    Brain.Screen.printAt(25, 70, "%f", Flywheel.torque());
    Brain.Screen.printAt(25, 85, "%f", Flywheel.velocity(dps)*0.3142);
    Brain.Screen.printAt(25, 100, "%f", Flywheel.efficiency());
    Brain.Screen.printAt(25, 115, "%f", heat);
    // Brain.Screen.printAt(25, 120, "%d", Flywheel.torque());

    wait(100, msec);
  }
  
}
