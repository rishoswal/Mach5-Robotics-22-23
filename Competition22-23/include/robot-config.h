using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor Flywheel;
extern motor Motor19;
extern motor Motor20;
extern motor Motor18;
extern motor Motor16;
extern motor Motor17;
extern motor Motor12;
extern digital_out DigitalOutH;
extern digital_out DigitalOutG;
extern motor Intake;
extern inertial Inertial;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );