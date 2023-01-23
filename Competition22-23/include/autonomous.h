#include "functions.h"

using namespace vex;

void Skills(){
  thread display(printHeading);
  vex::task runPId(startup);
  //Inertial.setHeading(, degrees);
  //Intake.spin(forward, 100, percent);
  fullDrive.spinFor(reverse, 0.2, sec, 50, rpm);
  fullDrive.stop(hold);
  rollNextColor();
  //wait(1, sec);
  rightDrive.spinFor(150, degrees, 60, rpm);
  Intake.spin(forward, 100,percent);
  turn(-38);
  cosdrive(26, 35);
  turn(90);
  Intake.stop();
  fullDrive.spinFor(reverse, 0.6, sec, 40, rpm);
  fullDrive.stop(hold);
  rollNextColor();
  //wait(1, sec);
  //Intake.stop();
  
  enableFlyPID = true;
  //volts = 9.5;
  rotateSpeed = 2375;
  enableLogistic = false;
  vex::task runPID(FlyWheelPIDRPM);
  
  cosdrive(16, 25);
  Intake.spin(forward, 100, percent);
  turn(0);
  cosdrive(48, 68);
  Flap.set(true);
  turn(-11);
  Intake.stop();
  tripleshot();
  enableFlyPID = false;
  
  //turn(0);
  cosdrive(-3.5, 10);
  turn(87);
  Intake.spin(forward, 100, percent);
  cosdrive(103, 65);
  turn(0);
  cosdrive(56, 65);
  wait(1, sec);
  cosdrive(-14, 30);
  turn(-90);

  vex::task runpId(startup);
  cosdrive(-10, 20);
  Intake.stop();
  fullDrive.spinFor(reverse, 1, sec, 25, rpm);
  //wait(0.05, sec);
  rollNextColor();

  //Intake.stop();

  cosdrive(21, 30);
  Intake.spin(forward, 100, percent);
  turn(-180);
  cosdrive(-18, 30);
  Intake.stop();
  fullDrive.spinFor(reverse, 0.8, sec, 25, rpm);
  //wait(0.7, sec);
  //Intake.stop();
  rollNextColor();

  enableFlyPID = true;
  rotateSpeed = 2375;
  vex::task runpID(FlyWheelPIDRPM);
  cosdrive(71, 68);
  rotateSpeed = 2375;
  Flap.set(true);
  turn(-195);
  // Intake.spinFor(reverse, 0.9, sec, 70, rpm);
  // rotateSpeed = 2300;
  // wait(0.5, sec);
  // Intake.spinFor(reverse, 0.9, sec, 70, rpm);
  // rotateSpeed = 2300;
  // wait(0.5, sec);
  // Intake.spinFor(reverse, 1.2, sec, 70, rpm);
  tripleshot();

  
  turn(-180);
  cosdrive(-51, 80);
  turn(-135);
  cosdrive(-15, 20);

  Expansion.set(true);


}