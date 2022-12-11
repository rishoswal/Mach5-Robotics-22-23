#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor Flywheel = motor(PORT9, ratio6_1, false);
motor left1 = motor(PORT19, ratio18_1, false);
motor left2 = motor(PORT20, ratio18_1, false);
motor left3 = motor(PORT18, ratio18_1, false);
motor right1 = motor(PORT16, ratio18_1, false);
motor right2 = motor(PORT17, ratio18_1, false);
motor right3 = motor(PORT12, ratio18_1, false);
digital_out DigitalOutH = digital_out(Brain.ThreeWirePort.H);
digital_out DigitalOutG = digital_out(Brain.ThreeWirePort.G);
motor Intake = motor(PORT14, ratio18_1, false);
inertial Inertial = inertial(PORT1);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}