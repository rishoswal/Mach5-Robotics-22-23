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
motor Intake = motor(PORT10, ratio18_1, false);
inertial Inertial = inertial(PORT3);
digital_out LeftExpansion = digital_out(expander.E);
digital_out RightMidExpansion = digital_out(expander.G);
encoder lEncoder = encoder(expander.A);
encoder rEncoder = encoder(Brain.ThreeWirePort.E);
encoder mEncoder = encoder(expander.C);
digital_out Flap = digital_out(Brain.ThreeWirePort.C);
/*vex-vision-config:begin*/
signature goalCam__REDGOAL = signature (1, 11423, 12867, 12145, -1953, -1311, -1632, 10.7, 0);
signature goalCam__BLUEGOAL = signature (2, -4229, -2213, -3222, 12375, 18245, 15310, 3, 0);
signature goalCam__SIG_3 = signature (3, 0, 0, 0, 0, 0, 0, 3, 0);
signature goalCam__SIG_4 = signature (4, 0, 0, 0, 0, 0, 0, 3, 0);
signature goalCam__SIG_5 = signature (5, 0, 0, 0, 0, 0, 0, 3, 0);
signature goalCam__SIG_6 = signature (6, 0, 0, 0, 0, 0, 0, 3, 0);
signature goalCam__SIG_7 = signature (7, 0, 0, 0, 0, 0, 0, 3, 0);
vision goalCam = vision (PORT4, 150, goalCam__REDGOAL, goalCam__BLUEGOAL, goalCam__SIG_3, goalCam__SIG_4, goalCam__SIG_5, goalCam__SIG_6, goalCam__SIG_7);
/*vex-vision-config:end*/
digital_out Expansion = digital_out(Brain.ThreeWirePort.B);
optical rollerColor = optical(PORT8);
potV2 colorSwitch = potV2(Brain.ThreeWirePort.G);

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