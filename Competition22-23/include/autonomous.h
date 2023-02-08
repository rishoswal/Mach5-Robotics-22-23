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
  turn(-30);
  cosdrive(25, 35);
  turn(88.5);
  wait(0.2, sec);
  cosdrive(-10, 33);
  Intake.stop();
  fullDrive.spinFor(reverse, 0.4, sec, 20, rpm);
  fullDrive.stop(hold);
  wait(0.15, sec);
  rollRed();
  
  enableFlyPID = true;
  rotateSpeed = 2375;
  enableLogistic = false;
  
  vex::task runPID(FlyWheelPIDRPM);
  
  cosdrive(16, 25);
  Intake.spin(forward, 100, percent);
  turn(0);
  cosdrive(48, 68);
  Flap.set(true);
  turn(-10);
  Intake.stop();
  tripleshot();
  //wait(0.2, sec);

  enableFlyPID = false;
  Flywheel.stop();
  
  cosdrive(-5.5, 10);
  turn(85.5);
  Intake.spin(forward, 100, percent);
  cosdrive(103, 57);
  turn(0);
  wait(0.1, sec);
  turn(0);
  cosdrive(57, 65);
  wait(0.3, sec);
  cosdrive(-16, 30);
  turn(-90);

  enableLogistic = true;
  vex::task runpId(startup);
  cosdrive(-17, 50);
  Intake.stop();
  fullDrive.spinFor(reverse, 0.2, sec, 25, rpm);
  fullDrive.stop(hold);
  rollRed();

  wait(0.2, seconds);
  
  cosdrive(25, 50);
  turn(-180);
  cosdrive(-22, 50);
  fullDrive.spinFor(reverse, 0.4, sec, 25, rpm);
  fullDrive.stop(hold);
  rollRed();
  

  enableFlyPID = true;
  rotateSpeed = 2375;
  vex::task runpID(FlyWheelPIDRPM);
  cosdrive(71, 87);
  rotateSpeed = 2375;
  Flap.set(true);
  turn(-197.5);
  tripleshot();

  enableFlyPID = false;
  Flywheel.stop();  
  turn(-180);
  cosdrive(-57, 100);
  turn(-135);
  cosdrive(-12, 50);
  
  Expansion.set(true);


}

//------------------- 15 Seconds -----------------------------------------------------------


void OnRoller(){
  vex::task runPId(startup);
  
  fullDrive.spinFor(reverse, 0.25, sec, 50, rpm);
  fullDrive.stop(hold);
  wait(0.15, sec);
  Intake.spinFor(0.3, seconds);
  Intake.stop();
  rollToColor();
  cosdrive(8, 20);
  turn(-10);
  rotateSpeed = 2755;
  vex::task RunPid(FlyWheelPIDRPM);
  wait(4, seconds);
  //tripleshot();
  Intake.spinFor(reverse, 0.9, seconds, 50, rpm);
  wait(2, seconds);
  Intake.spinFor(reverse, 0.9, seconds, 50, rpm);
  Flywheel.spin(forward, 2000, rpm);
  rotateSpeed = 2000;
}

void OffRoller(){
  vex::task runPID(startup);
  Intake.spin(forward, 100, percent);
  cosdrive(33, 70);
  //turn(20);
  rotateSpeed = 2800;
  vex::task RUNPID(FlyWheelPIDRPM);
  //wait(4, seconds);
  //tripleshot();
  turn(-45);
  cosdrive(-43, 60);
  turn(0);
  Intake.stop();

  fullDrive.spinFor(reverse, 0.355, sec, 50, rpm);
  fullDrive.stop(hold);
  wait(0.15, sec);
  rollToColor();
  cosdrive(4, 20);
  turn(6);

  Intake.spinFor(reverse, 0.9, seconds, 50, rpm);
  wait(1.5, seconds);
  Intake.spinFor(reverse, 0.9, seconds, 50, rpm);
  wait(1.5, seconds);
  Intake.spinFor(reverse, 0.9, seconds, 50, rpm);
  Flywheel.spin(forward, 2000, rpm);
  rotateSpeed = 2000;
  
}

timer time15;
void Win(){
  time15.reset();
  task runPID(startup);
  fullDrive.spinFor(reverse, 0.25, sec, 50, rpm);
  fullDrive.stop(hold);
  rollToColor();
  fullDrive.spinFor(forward, 0.20, sec, 80, rpm);
  turn(43, true);
  cosdrive(142, 100);

  rotateSpeed = 2730;
  task RUNPID(FlyWheelPIDRPM);

  rightDrive.spinFor(forward, 0.36, sec, 100, rpm);
  //fullDrive.spinFor(forward, 0.20, sec, 50, rpm);
  turn(-90, true);
  cosdrive(-13, 40);
  fullDrive.spinFor(reverse, 0.25, sec, 50, rpm);
  fullDrive.stop();
  rollToColor();

  cosdrive(10, 30);
  turn(-83, true);
  Intake.spinFor(reverse, 0.9, seconds, 50, rpm);
  waitUntil(time15.time(sec) > 14);
  Intake.spinFor(reverse, 0.9, seconds, 50, rpm);
  Flywheel.spin(forward, 2000, rpm);
  rotateSpeed = 2000;
}