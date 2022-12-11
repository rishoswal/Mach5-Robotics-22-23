using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor Flywheel;
extern motor left1;
extern motor left2;
extern motor left3;
extern motor right1;
extern motor right2;
extern motor right3;
extern digital_out DigitalOutH;
extern digital_out DigitalOutG;
extern motor Intake;
extern inertial Inertial;
extern triport expander;
extern potV2 autonswitch;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );