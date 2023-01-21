#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
triport expander = triport(PORT5);
controller Controller1 = controller(primary);
motor Flywheel = motor(PORT9, ratio6_1, false);
motor left1 = motor(PORT19, ratio18_1, false);
motor left2 = motor(PORT20, ratio18_1, false);
motor left3 = motor(PORT18, ratio18_1, false);
motor right1 = motor(PORT16, ratio18_1, true);
motor right2 = motor(PORT17, ratio18_1, true);
motor right3 = motor(PORT12, ratio18_1, true);
digital_out Shooter = digital_out(Brain.ThreeWirePort.G);
motor Intake = motor(PORT10, ratio18_1, false);
inertial Inertial = inertial(PORT3);
potV2 autonswitch = potV2(expander.H);
digital_out LeftExpansion = digital_out(expander.E);
digital_out RightMidExpansion = digital_out(expander.G);
encoder lEncoder = encoder(expander.A);
encoder rEncoder = encoder(Brain.ThreeWirePort.E);
encoder mEncoder = encoder(expander.C);
digital_out Flap = digital_out(Brain.ThreeWirePort.C);
/*vex-vision-config:begin*/
vision Vision = vision (PORT4, 50);
/*vex-vision-config:end*/
digital_out Expansion = digital_out(Brain.ThreeWirePort.B);

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