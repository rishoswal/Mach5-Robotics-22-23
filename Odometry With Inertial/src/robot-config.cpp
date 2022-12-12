#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
triport Expander8 = triport(PORT8);
motor l1 = motor(PORT18, ratio18_1, false);
motor l2 = motor(PORT19, ratio18_1, false);
motor l3 = motor(PORT20, ratio18_1, false);
motor r1 = motor(PORT11, ratio18_1, true);
motor r2 = motor(PORT16, ratio18_1, true);
motor r3 = motor(PORT17, ratio18_1, true);
inertial Inertial = inertial(PORT3);
digital_out DigitalOutG = digital_out(Brain.ThreeWirePort.G);
digital_out DigitalOutH = digital_out(Brain.ThreeWirePort.H);
encoder lEncoder = encoder(Expander8.A);
encoder rEncoder = encoder(Brain.ThreeWirePort.A);
encoder mEncoder = encoder(Expander8.C);

// VEXcode generated functions



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}