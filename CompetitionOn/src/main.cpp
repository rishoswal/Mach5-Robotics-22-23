/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// Flywheel             motor         9               
// left1                motor         19              
// left2                motor         20              
// left3                motor         18              
// right1               motor         16              
// right2               motor         17              
// right3               motor         12              
// DigitalOutH          digital_out   H               
// DigitalOutG          digital_out   G               
// Intake               motor         14              
// Inertial             inertial      1               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
double kP = 0.03;
int turnRight;
double kI = 0.0;
double kD = 0.003;

int flySpeed;

int counter = 0;
int rotateSpeed = 2300;
int error;
int prevError;
int derivative;
int integral;
int diff;
int currSpeed;
bool enableLogistic = true;
bool enableVisionPID = false;
bool enableFlyPID = false;
float volts = 7.5;
double heat;

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/
motor_group leftDrive(left3, left2, left1);
motor_group rightDrive(right1, right2, right3);

void motorStopAll(brakeType mode) {

  rightDrive.stop(mode);
  leftDrive.stop(mode);
}
// Spin all the motors base on direction (fwd or reverse), left motor speed, and
// right motor speed
void motorSpin(directionType dir, double speedLeft, double speedRight) {
  leftDrive.spin(dir, speedLeft, pct);
  //frontLeftMotor.spin(dir, speedLeft, pct);
  rightDrive.spin(dir, speedRight, pct);
 // backRightMotor.spin(dir, speedRight, pct);
}
// Rotate all the motors in degrees for left and right motors
void motorRotate(double degreeLeft, double degreeRight) {
  right1.startRotateFor(degreeRight, degrees);
  //backRightMotor.startRotateFor(degreeRight, degrees);
  right2.startRotateFor(degreeRight, degrees);
  right3.startRotateFor(degreeRight, degrees);
  left1.startRotateFor(degreeLeft, degrees);
  left3.startRotateFor(degreeLeft, degrees);
  left2.rotateFor(degreeLeft, degrees);
  // wait(0.5, sec);
}

timer drivetimer;
void cosdrive(double inches, double speed){ //uses the changing slope of a cosine wave to accelerate/decelerate the robot for precise movement.
// better explanation and visualization here: https://www.desmos.com/calculator/begor0sggm
	double velocity;
  double seconds=fabs((3.5*inches)/speed); //calculates the time the robot will take to complete a cycle based on the wheel
  //circumference, gear ratio, target distance, and motor speed.
  drivetimer.reset();
  while(drivetimer.time(sec)<seconds){
    velocity=(1-cos((6.283*drivetimer.time(sec))/seconds))*speed/2 * (fabs(inches)/inches); //uses equation for the cosine wave to calculate velocity.
    leftDrive.spin(forward,velocity,percent);
		rightDrive.spin(forward,velocity,percent);
	}                                                                                                                                                              
	leftDrive.stop();
	rightDrive.stop();
}

int endAngle=0; //driving forward will drift the back encoder unintentionally,
//so we save the value of where we turned last and use it when turning again to ignore drift.
void turn(float angle){ //function for turning. Spins with a speed cap of 36 percent, uses proportional correction
  //Inertial.setRotation(endAngle, degrees);
  float error = angle-(Inertial.rotation(degrees)*1.0143);
  while(fabs(error)>2||fabs(Inertial.gyroRate(zaxis, rpm))>1){ //exits loop if error <2 and rotational speed <1
    error = angle-(Inertial.rotation(degrees)*1.0143);//calculates error value
    if(fabs(error)>54){ //if error is greater than 50, use proportional correction. if not, turn at 36 percent speed
      leftDrive.spin(forward,36*(fabs(error)/error),percent);
      rightDrive.spin(reverse,36*(fabs(error)/error),percent);
    }else{
      leftDrive.spin(forward,error*0.73,percent);
      rightDrive.spin(reverse,error*0.73,percent);
    }
  }
  leftDrive.stop();
  rightDrive.stop();
  Brain.Screen.printAt(1,20,"%f",Inertial.heading());
  //endAngle=Inertial.heading();
}

void inertialRotate(int heading){
  int newTurn = heading;
  if ((Inertial.heading(degrees) < newTurn &&
         newTurn - Inertial.heading(degrees) < 180) ||
        (Inertial.heading(degrees) > newTurn &&
         newTurn - Inertial.heading(degrees) > 180)) {
      turnRight = 1;
    } else {
      turnRight = -1;
    }
  while (abs(Inertial.heading(degrees) - newTurn) > 2) {
    int spinFactor = abs(Inertial.heading(degrees) - newTurn);
    if (turnRight == -1) {
      motorSpin(fwd, -1 * spinFactor - 20, spinFactor + 20);
    } else {
      motorSpin(fwd, spinFactor + 20, -1 * spinFactor - 20);
    }
  }
motorStopAll(hold);
}

void openP(){
  while(Controller1.ButtonA.pressing()){
    DigitalOutH.set(true);
  }
}
void closeP(){
  DigitalOutH.set(false);
}

void shoot(){
  DigitalOutG.set(true);
  wait(0.2, sec);
  DigitalOutG.set(false);
  //return(1);
}

void expansion(){
  DigitalOutH.set(true);
  wait(0.3, sec);
  DigitalOutH.set(false);
}

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

int FlyWheelPID() {
  int i = 0;
  while (enableFlyPID) {
    diff = 0;
    error = Flywheel.voltage() - (volts-(volts*0.1));
    derivative = error - prevError;
    diff = (error * kP) + (derivative * kD);
    prevError = error;
    currSpeed = Flywheel.voltage() - diff;
    if (Flywheel.voltage() < volts - 0.35){
      Flywheel.spin(forward, volts, volt);
    }
    
    // else {
    //  Flywheel.spin(forward, Flywheel.velocity(rpm) - diff, rpm
    
    //Brain.Screen.clearLine();
    //Brain.Screen.print(Flywheel.voltage());
    /*Flywheel.setVelocity(rotateSpeed/6, rpm);
    Flywheel.spin(forward);
    Brain.Screen.clearLine();
    Brain.Screen.print(Flywheel.velocity(rpm)*6);*/
    vex::task::sleep(10);
    //i++;
  }
  return(1);
}

int FlyWheelPIDRPM() {
  int i = 0;
  while (enableFlyPID) {
    diff = 0;
    error = Flywheel.velocity(rpm)*6 - (rotateSpeed-(rotateSpeed*0.1));
    derivative = error - prevError;
    diff = (error * kP) + (derivative * kD);
    prevError = error;
    currSpeed = Flywheel.velocity(rpm) - diff;
    if (Flywheel.velocity(rpm)*6 < rotateSpeed - 100){
      Flywheel.spin(forward, currSpeed, rpm);
    }
    
    // else {
    //  Flywheel.spin(forward, Flywheel.velocity(rpm) - diff, rpm
    
    //Brain.Screen.clearLine();
    //Brain.Screen.print(Flywheel.voltage());
    /*Flywheel.setVelocity(rotateSpeed/6, rpm);
    Flywheel.spin(forward);
    Brain.Screen.clearLine();
    Brain.Screen.print(Flywheel.velocity(rpm)*6);*/
    vex::task::sleep(10);
    //i++;
  }
  return(1);
}

float convertYToX(float yvalue){
  int xvalue = ((log((600/yvalue) - 1))/-0.006)+480;
  if (yvalue ==0){
    xvalue = 0;
  }
  return(xvalue);
}

int Test(int startSpeed, int finalSpeed){
//Flywheel.spin(forward,50, rpm);
//wait(5, sec);
  float counter = convertYToX(startSpeed/6);
  Brain.Screen.print(counter);
  while (Flywheel.velocity(rpm)*6 < finalSpeed) {
    //int flyrotation = Rotation.velocity(rpm);
    Flywheel.setVelocity(((finalSpeed/6)+50)/(1+ pow(2.71828, (-0.006)*(counter - 480))), rpm);
    Flywheel.spin(forward);
    //Flywheel.spin(forward, 600/(1+ pow(2.71828, (-0.006)*(counter - 480))) , rpm);
    if (startSpeed < finalSpeed){
      counter += 50;
    } else{
      counter -= 50;
    }
    wait(0.7, sec);
    Brain.Screen.clearLine();
    Brain.Screen.print(Flywheel.velocity(rpm)*6);
  }
  Flywheel.spin(forward);
  return 1;
}

int Startup(){
//Flywheel.spin(forward,50, rpm);
//wait(5, sec);
  float counter = 0;
  Brain.Screen.print(counter);
  while (Flywheel.velocity(rpm)*6 < 1780 && enableLogistic == true) {
    //int flyrotation = Rotation.velocity(rpm);
    Flywheel.setVelocity(((1900/6)+50)/(1+ pow(2.71828, (-0.006)*(counter - 480))), rpm);
    Flywheel.spin(forward);
    //Flywheel.spin(forward, 600/(1+ pow(2.71828, (-0.006)*(counter - 480))) , rpm);
    counter += 65;
    wait(0.7, sec);
    Brain.Screen.clearLine();
    Brain.Screen.print(Flywheel.velocity(rpm)*6);
  }
  Flywheel.spin(forward, 7, volt);
  enableFlyPID = true;
  return 1;
}

void OnRoller(){
  vex::task runPId(Startup);
  //Intake.spinFor(forward, 220, degrees);
  motorRotate(100, 100);
  motorRotate(-50, 50);
  wait(3, seconds);
  
  enableFlyPID = true;
  rotateSpeed = 2650;
  enableLogistic = false;
  vex::task runPID(FlyWheelPIDRPM);
    
  wait(7, sec);
  shoot();
  rotateSpeed = 2650;
  wait(3.5, seconds);
  shoot();
}

void OffRoller(){
  vex::task runPId(Startup);
  //Intake.spinFor(forward, 220, degrees);
  motorRotate(150, 150);
  motorRotate(-75, 75);
  wait(3, seconds);
  
  enableFlyPID = true;
  volts = 9.5;
  enableLogistic = false;
  vex::task runPID(FlyWheelPID);
  
  wait(7, sec);
  shoot();
  wait(2, seconds);
  shoot();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  //OnRoller();
  cosdrive(24, 50);
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
      int leftVal = 0;
      int rightVal = 0;
      Brain.Screen.clearLine();
      Brain.Screen.print(left3.velocity(rpm));
      // if the axis 3 and axis 1's value is 0 the right and left wheel motors
      // should stop

    if (counter == 0){
      vex::task runPID(Startup);
      counter ++;
    }
    //while(true){
    if(Controller1.ButtonR1.pressing()&&volts<11){
      volts+=0.5;
      wait(0.2, sec);
    }

    if(Controller1.ButtonUp.pressing()){
      volts = 9.5;
      wait(0.2, sec);
    }
  
    if(Controller1.ButtonDown.pressing()){
      volts=7.5;
      enableFlyPID=false;
      Flywheel.spin(forward, volts, volt);
      wait(0.2, sec);
      enableFlyPID = true;
      //wait(0.2, sec);

    }

    if(Controller1.ButtonL1.pressing()&&volts>0){
      volts-=0.5;
      enableFlyPID=false;
      Flywheel.spin(forward, volts, volt);
      wait(0.2, sec);
      enableFlyPID = true;
      //wait(0.2, sec);

    }
    if(Controller1.ButtonDown.pressing()){
      volts=0;
    }

    // Generally, 7 to 10 volts is a good range for all distances on th field
    //Flywheel.spin(forward, volts, volt);
    //vex::task FlyWheelPID();

    vex::task runPID(FlyWheelPID); 
    heat = (Flywheel.voltage()*Flywheel.current()) - (Flywheel.torque()*Flywheel.velocity(dps)*0.3142);

      // when the axis 3 value is greater than zero the motor moves forward
    Brain.Screen.clearLine();
    if (Controller1.Axis4.value() != 0) {
      leftVal += Controller1.Axis4.value()/2;
      rightVal -= Controller1.Axis4.value()/2;
    }
    if (Controller1.Axis3.value() != 0) {
      leftVal += Controller1.Axis3.value();
      rightVal += Controller1.Axis3.value();
    }
    leftDrive.spin(fwd, leftVal, pct);
    rightDrive.spin(fwd, rightVal, pct);
  //    Brain.Screen.print(Controller1.Axis3.value());
    //  Brain.Screen.print(" ");
      //Brain.Screen.print(Controller1.Axis4.value());
    if (Controller1.Axis3.value() == 0 && Controller1.Axis4.value() == 0) {
        leftDrive.stop(coast);
        rightDrive.stop(coast);
          //Brain.Screen.print(1);
          //Brain.Screen.clearLine();
    }
    if (Controller1.Axis2.value() >= 10){
      Intake.spin(forward, 100, percent);
    } else if (Controller1.Axis2.value() <= -10){
      Intake.spin(reverse, 100, percent);
    } else{
      Intake.stop(coast);
    }
    //Controller1.ButtonA.pressed(openP);
    //Controller1.ButtonB.pressed(closeP);
    Controller1.ButtonR2.pressed(shoot);
    Controller1.ButtonB.pressed(expansion);

    //Brain.Screen.clearScreen();
    Brain.Screen.printAt(15, 25, "Volts input: %f", volts);
    Brain.Screen.printAt(15, 40, "    Voltage: %f", Flywheel.velocity(rpm)*6);
    Brain.Screen.printAt(15, 55, "      Power: %f", Flywheel.voltage());
    Brain.Screen.printAt(15, 70, "     Torque: %f", Flywheel.torque());
    Brain.Screen.printAt(15, 85, "   AngularV: %f", Flywheel.velocity(dps)*0.3142);
    Brain.Screen.printAt(15, 100, " Efficiency; %f", Flywheel.efficiency());
    Brain.Screen.printAt(15, 115, "  Heat Loss: %f", heat);
    Brain.Screen.printAt(15, 130, " Resistance: %f", Flywheel.voltage()/Flywheel.current());
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print("%.2f",volts);
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
