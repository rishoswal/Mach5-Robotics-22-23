#include "functions.h"

using namespace vex;

void Skills(){
  thread display(printHeading);
  vex::task runPId(startup);
  //Inertial.setHeading(, degrees);
  //Intake.spin(forward, 100, percent);
  fullDrive.spinFor(reverse, 0.2, sec, 50, rpm);
  thread roll(rollNextColor);
  //Brain.Screen.print("Hello");
  wait(1, sec);
  rightDrive.spinFor(150, degrees, 60, rpm);
  turn(-45);
  cosdrive(23, 35);
  turn(90);
  fullDrive.spinFor(reverse, 1.1, sec, 40, rpm);
  wait(1, sec);
  Intake.stop();
  
  enableFlyPID = true;
  //volts = 9.5;
  rotateSpeed = 2250;
  enableLogistic = false;
  vex::task runPID(FlyWheelPIDRPM);
  
  cosdrive(12, 25);
  turn(0);
  cosdrive(50, 68);
  Flap.set(true);
  turn(-13);
  tripleshot();
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
  rotateSpeed = 2250;
  vex::task runpID(FlyWheelPIDRPM);
  cosdrive(69, 68);
  rotateSpeed = 2250;
  Flap.set(true);
  turn(-196);
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