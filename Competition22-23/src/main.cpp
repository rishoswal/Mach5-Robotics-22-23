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
// Blocker              digital_out   H               
// Shooter              digital_out   G               
// Intake               motor         10              
// Inertial             inertial      3               
// expander             triport       8               
// autonswitch          potV2         H               
// LeftExpansion        digital_out   E               
// RightMidExpansion    digital_out   G               
// lEncoder             encoder       A, B            
// rEncoder             encoder       A, B            
// mEncoder             encoder       C, D            
// Flap                 digital_out   C               
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
int finalSpeed = 1700;
int rotateSpeed = 2200;
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

float xPos = 0;
float yPos = 0;
float heading;
float headingD;
float goalX = 16;
float goalY = -109;

const float degreesToRadians = 2 * 3.141593 / 360.0;
const float degreesToInches = 2.75 * 3.141593 / 360.0; //with a 2.75 in diameter wheel

bool toggleAutoSpeed = false;

timer expandTimer;

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
motor_group fullDrive(left1, left2, left3, right1, right2, right3);

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
  float error = angle-(Inertial.rotation());
  while(fabs(error)>2){ //exits loop if error <2 and rotational speed <1
    error = angle-(Inertial.rotation());//calculates error value
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
}

void shoot(){
  Blocker.set(false);
  wait(0.3, sec);
  Shooter.set(true);
  wait(0.3, sec);
  Shooter.set(false);
}

void autoshoot(float goalOffset){
  float goalAngle = 90 - (atan2(goalY - yPos, goalX - xPos) / degreesToRadians) + goalOffset;
  Controller1.Screen.print(goalAngle);
  turn(goalAngle);
  // wait(0.2, sec);
  // Shooter.set(true);
  // wait(0.3, sec);
  // Shooter.set(false);
  leftDrive.spin(forward);
  rightDrive.spin(forward);
}

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  Inertial.calibrate();

  left1.setBrake(coast);
  left2.setBrake(coast);
  left3.setBrake(coast);
  right1.setBrake(coast);
  right2.setBrake(coast);
  right3.setBrake(coast);

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

int FlyWheelPIDRPM() {
  while (enableFlyPID) {
    if(toggleAutoSpeed == false){
      diff = 0;
      error = Flywheel.velocity(rpm)*6 - (rotateSpeed-(rotateSpeed*0.1));
      derivative = error - prevError;
      // Brain.Screen.print(Flywheel.velocity(rpm)*6);
      // Brain.Screen.clearLine();
      diff = (error * kP) + (derivative * kD);
      prevError = error;
      currSpeed = Flywheel.velocity(rpm) - diff;
      if (Flywheel.velocity(rpm)*6 < rotateSpeed - 100){
        Flywheel.spin(forward, currSpeed, rpm);
      }
      
      // else {
      //  Flywheel.spin(forward, Flywheel.velocity(rpm) - diff, rpm
    }
    vex::task::sleep(10);
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
    // Brain.Screen.clearLine();
    // Brain.Screen.print(Flywheel.velocity(rpm)*6);
  }
  Flywheel.spin(forward);
  return 1;
}

int startup(){
//Flywheel.spin(forward,50, rpm);
//wait(5, sec);
  float counter = 0;
  while (Flywheel.velocity(rpm)*6 < finalSpeed && enableLogistic == true) {
    if(toggleAutoSpeed == false){
      //int flyrotation = Rotation.velocity(rpm);
      Flywheel.setVelocity(((finalSpeed/6)+50)/(1+ pow(2.71828, (-0.006)*(counter - 480))), rpm);
      Flywheel.spin(forward);
      //Flywheel.spin(forward, 600/(1+ pow(2.71828, (-0.006)*(counter - 480))) , rpm);
      counter += 95;
    }
    wait(0.7, sec);
    // Brain.Screen.clearLine();
    // Brain.Screen.print(Flywheel.velocity(rpm)*6);
  }
  //Flywheel.spin(forward, 7, volt);
  enableFlyPID = true;
  return 1;
}

void odometryInertial(){

  float previousFB;
  float currentFB = 0;
  float changeFB;

  float previousLR;
  float currentLR = 0;
  float changeLR;

  while(true){
    previousFB = currentFB;
    currentFB = (lEncoder.position(degrees) + rEncoder.position(degrees)) / 2;
    changeFB = currentFB - previousFB;
    previousLR = currentLR;
    currentLR = mEncoder.position(degrees);
    changeLR = currentLR - previousLR;
    headingD = Inertial.heading(degrees);
    heading = headingD * degreesToRadians;

    xPos += ((changeFB * sin(heading)) + (changeLR * cos(heading))) * degreesToInches;
    yPos += ((changeFB * cos(heading)) + (changeLR * sin(heading))) * degreesToInches;

    Brain.Screen.printAt(15, 25, "    x position: %f", xPos);
    Brain.Screen.printAt(15, 40, "    y position: %f", yPos);
    Brain.Screen.printAt(15, 55, "       heading: %f", headingD);
    Brain.Screen.printAt(15, 70, "  Left Encoder: %f", lEncoder.position(degrees));
    Brain.Screen.printAt(15, 85, " Right Encoder: %f", rEncoder.position(degrees));
    Brain.Screen.printAt(15, 100, "Middle Encoder: %f", mEncoder.position(degrees));

    wait(20, msec);
  }
}

void autoPower(){
  float goalDistance;

  while(true){
    if(toggleAutoSpeed){
      goalDistance = sqrt(pow(xPos - goalX, 2) + pow(yPos - goalY, 2));

      Flywheel.spin(forward, goalDistance * 0.04 + 5, volt);
    }

    wait(0.2, sec);
  }
}


void tripleshot(){
  rotateSpeed = 2500;
  Intake.spin(reverse, 50, percent);
  wait(2, seconds);
  Intake.stop(coast);
  rotateSpeed = 2300;
}

void printHeading(){
  while(1){
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print(Inertial.heading());
    wait(50, msec);
  }
}

void Skills(){
  thread display(printHeading);
  vex::task runPId(startup);
  //Inertial.setHeading(, degrees);
  Intake.spin(forward, 100, percent);
  fullDrive.spinFor(reverse, 0.4, sec, 100, rpm);
  wait(1, sec);
  rightDrive.spinFor(150, degrees, 60, rpm);
  turn(-45);
  cosdrive(19, 35);
  turn(90);
  fullDrive.spinFor(reverse, 1.1, sec, 40, rpm);
  wait(1, sec);
  Intake.stop();
  
  enableFlyPID = true;
  //volts = 9.5;
  rotateSpeed = 2300;
  enableLogistic = false;
  vex::task runPID(FlyWheelPIDRPM);
  
  cosdrive(10, 25);
  turn(0);
  cosdrive(50, 68);
  Flap.set(true);
  turn(-13);
  Intake.spinFor(reverse, 0.4, sec, 70, rpm);
  rotateSpeed = 2600;
  Intake.spinFor(reverse, 2.5, sec, 70, rpm);
  wait(0.2, sec);
  rotateSpeed = 2300;
  enableFlyPID = false;
  
  //turn(0);
  cosdrive(-5.5, 10);
  turn(88.5);
  Intake.spin(forward, 100, percent);
  cosdrive(99, 65);
  turn(0);
  cosdrive(56, 65);
  wait(1, sec);
  cosdrive(-14, 30);
  turn(-90);

  vex::task runpId(startup);
  cosdrive(-10, 20);
  fullDrive.spinFor(reverse, 0.8, sec, 25, rpm);
  wait(0.05, sec);
  //Intake.stop();

  cosdrive(19, 30);
  turn(-180);
  cosdrive(-18, 30);
  fullDrive.spinFor(reverse, 0.8, sec, 25, rpm);
  wait(0.7, sec);
  Intake.stop();

  enableFlyPID = true;
  rotateSpeed = 2300;
  vex::task runpID(FlyWheelPIDRPM);
  cosdrive(69, 68);
  rotateSpeed = 2300;
  Flap.set(true);
  turn(-196);
  // Intake.spinFor(reverse, 0.9, sec, 70, rpm);
  // rotateSpeed = 2300;
  // wait(0.5, sec);
  // Intake.spinFor(reverse, 0.9, sec, 70, rpm);
  // rotateSpeed = 2300;
  // wait(0.5, sec);
  // Intake.spinFor(reverse, 1.2, sec, 70, rpm);
  Intake.spinFor(reverse, 0.4, sec, 70, rpm);
  rotateSpeed = 2600;
  Intake.spinFor(reverse, 2.5, sec, 70, rpm);
  wait(0.2, sec);
  rotateSpeed = 2300;

  
  turn(-180);
  cosdrive(-51, 80);
  turn(-135);
  cosdrive(-15, 20);


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
  //thread startOdom(odometryInertial);
  //FullWin();
  //OffRoller();
  Skills();
  // if(autonswitch.value(percent)<25){
  //   OffRoller();
  // }else if(autonswitch.value(percent)<50){
  //   OnRoller();
  // }else if(autonswitch.value(percent)<75){
  //   FullWin();
  // }else{
  //   Skills();
  // }
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
  //Blocker.set(true);
  thread startOdom(odometryInertial);
  expandTimer.reset();
  while (1) {
      int leftVal = 0;
      int rightVal = 0;

     if (counter == 0){
       vex::task runPID(startup);
       counter ++;
     }

    if(Controller1.ButtonA.pressing()){
      tripleshot();
    }

    if(Controller1.ButtonUp.pressing()){
      //volts = 9.5;
      rotateSpeed += 75;
      wait(0.2, sec);
    }
  
    if(Controller1.ButtonDown.pressing()){
      rotateSpeed -= 75;
    }

    if(Controller1.ButtonRight.pressing()){
      rotateSpeed = 2300;
    }

    //vex::task FlyWheelPID();

     task runPID(FlyWheelPIDRPM); 
    // thread startFlywheel(autoPower);
    heat = (Flywheel.voltage()*Flywheel.current()) - (Flywheel.torque()*Flywheel.velocity(dps)*0.3142);

      // when the axis 3 value is greater than zero the motor moves forward
    // Brain.Screen.clearLine();
    if (Controller1.Axis1.value() != 0) {
      leftVal += Controller1.Axis1.value()/2;
      rightVal -= Controller1.Axis1.value()/2;
    }
    if (Controller1.Axis3.value() != 0) {
      leftVal += Controller1.Axis3.value();
      rightVal += Controller1.Axis3.value();
      if(abs(Controller1.Axis3.value()) > 10){
        Blocker.set(true);
      }
    }
    leftDrive.spin(fwd, leftVal, pct);
    rightDrive.spin(fwd, rightVal, pct);
  
    if (Controller1.Axis3.value() == 0 && Controller1.Axis1.value() == 0) {
        leftDrive.stop(coast);
        rightDrive.stop(coast);
    }
    if (Controller1.ButtonR2.pressing()){
      Intake.spin(forward, 100, percent);
    } else if (Controller1.ButtonR1.pressing()){ 
      Intake.spin(reverse, 50, percent);
    } else{
      Intake.stop(coast);
    }
    //Controller1.ButtonR1.pressed(shoot);
    if(Controller1.ButtonB.pressing()){
      Blocker.set(false);
      autoshoot(-10);
      waitUntil(!Controller1.ButtonR2.pressing());
    }
    // if(Controller1.ButtonX.pressing()){
    //   if(toggleAutoSpeed){
    //     toggleAutoSpeed = false;
    //   }else{
    //     toggleAutoSpeed = true;
    //   }
    //   waitUntil(!Controller1.ButtonX.pressing());
    // }
    if(Controller1.ButtonY.pressing() && expandTimer.time(sec) > 95){
      RightMidExpansion.set(true);
      wait(1, sec);
      LeftExpansion.set(true);
    }
    if(Controller1.ButtonL2.pressing()){
      Flap.set(false);
    }
    if(Controller1.ButtonL1.pressing()){
      Flap.set(true);
    }
    //Brain.Screen.clearScreen();
    // Brain.Screen.printAt(15, 25, "Volts input: %f", volts);
    // Brain.Screen.printAt(15, 40, "    Voltage: %f", Flywheel.velocity(rpm)*6);
    // Brain.Screen.printAt(15, 55, "      Power: %f", Flywheel.voltage());
    // Brain.Screen.printAt(15, 70, "     Torque: %f", Flywheel.torque());
    // Brain.Screen.printAt(15, 85, "        RPM: %f", Flywheel.velocity(rpm)*6);
    // Brain.Screen.printAt(15, 100, "Efficiency; %f", Flywheel.efficiency());
    // Brain.Screen.printAt(15, 115, " Heat Loss: %f", heat);
    // Brain.Screen.printAt(15, 130, "Resistance: %f", Flywheel.voltage()/Flywheel.current());
    // Controller1.Screen.setCursor(1,1);
    // Controller1.Screen.print("%.2f",volts);
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
