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
extern digital_out Shooter;
extern motor Intake;
extern inertial Inertial;
extern triport expander;
extern potV2 autonswitch;
extern digital_out LeftExpansion;
extern digital_out RightMidExpansion;
extern encoder lEncoder;
extern encoder rEncoder;
extern encoder mEncoder;
extern digital_out Flap;
extern signature Vision__SIG_1;
extern signature Vision__SIG_2;
extern signature Vision__SIG_3;
extern signature Vision__SIG_4;
extern signature Vision__SIG_5;
extern signature Vision__SIG_6;
extern signature Vision__SIG_7;
extern vision Vision;
extern digital_out Expansion;
extern optical rollerColor;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );