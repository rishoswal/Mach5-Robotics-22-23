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

  float volts = 0;
  double heat;

  while(true){
    if(Controller1.ButtonRight.pressing()&&volts<11){
      volts+=0.5;
    }
    if(Controller1.ButtonLeft.pressing()&&volts>0){
      volts-=0.5;
    }
    if(Controller1.ButtonDown.pressing()){
      volts=0;
    }

    // Generally, 7 to 10 volts is a good range for all distances on th field
    Flywheel.spin(forward, volts, volt);

    heat = (Flywheel.voltage()*Flywheel.current()) - (Flywheel.torque()*Flywheel.velocity(dps)*0.3142);

    Brain.Screen.clearScreen();
    Brain.Screen.printAt(15, 25, "Volts input: %f", volts);
    Brain.Screen.printAt(15, 40, "    Voltage: %f", Flywheel.voltage());
    Brain.Screen.printAt(15, 55, "      Power: %f", Flywheel.power());
    Brain.Screen.printAt(15, 70, "     Torque: %f", Flywheel.torque());
    Brain.Screen.printAt(15, 85, "   AngularV: %f", Flywheel.velocity(dps)*0.3142);
    Brain.Screen.printAt(15, 100, " Efficiency; %f", Flywheel.efficiency());
    Brain.Screen.printAt(15, 115, "  Heat Loss: %f", heat);
    Brain.Screen.printAt(15, 130, " Resistance: %f", Flywheel.voltage()/Flywheel.current());

    wait(100, msec);
  }
  
}
