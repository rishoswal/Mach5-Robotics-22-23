#include "functions.h"

using namespace vex;

void Skills(){
  thread display(printHeading);
  
  vex::task runPId(startup);
  
  fullDrive.spinFor(reverse, 0.25, sec, 50, rpm);
  fullDrive.stop(hold);
  wait(0.15, sec);
  rollRed();
  
  rightDrive.spinFor(150, degrees, 60, rpm);
  Intake.spin(forward, 100,percent);
  turn(-36);
  cosdrive(26, 35);
  turn(90);
  Intake.stop();
  fullDrive.spinFor(reverse, 0.675, sec, 40, rpm);
  fullDrive.stop();
  wait(0.15, sec);
  rollRed();
  
  enableFlyPID = true;
  rotateSpeed = 2300;
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
  
  cosdrive(-3.5, 10);
  turn(87);
  Intake.spin(forward, 100, percent);
  cosdrive(103, 65);
  turn(0);
  cosdrive(56, 65);
  wait(1, sec);
  cosdrive(-13, 30);
  turn(-90);

  vex::task runpId(startup);
  cosdrive(-10, 20);
  Intake.stop();
  fullDrive.spinFor(reverse, 0.95, sec, 25, rpm);
  fullDrive.stop(hold);
  rollRed();
  wait(0.2, seconds);
  
  cosdrive(21, 30);
  //Intake.spin(forward, 100, percent);
  turn(-180);
  cosdrive(-17, 30);
  //Intake.stop();
  fullDrive.spinFor(reverse, 1, sec, 25, rpm);
  fullDrive.stop();
  rollRed();
  

  enableFlyPID = true;
  rotateSpeed = 2300;
  vex::task runpID(FlyWheelPIDRPM);
  cosdrive(71, 68);
  rotateSpeed = 2300;
  Flap.set(true);
  turn(-197.5);
  tripleshot();

  
  turn(-180);
  cosdrive(-57, 80);
  turn(-135);
  cosdrive(-12, 20);

  Expansion.set(true);


}