using namespace vex;

extern brain Brain;

using signature = vision::signature;

// VEXcode devices
extern controller Controller1;
extern motor Flywheel;
extern motor left1;
extern motor left2;
extern motor left3;
extern motor right1;
extern motor right2;
extern motor right3;
extern motor Intake;
extern inertial Inertial;
extern triport expander;
extern digital_out LeftExpansion;
extern digital_out RightMidExpansion;
extern encoder lEncoder;
extern encoder rEncoder;
extern encoder mEncoder;
extern digital_out Flap;
extern signature goalCam__REDGOAL;
extern signature goalCam__BLUEGOAL;
extern signature goalCam__SIG_3;
extern signature goalCam__SIG_4;
extern signature goalCam__SIG_5;
extern signature goalCam__SIG_6;
extern signature goalCam__SIG_7;
extern vision goalCam;
extern digital_out Expansion;
extern optical rollerColor;
extern potV2 codeSwitch;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );