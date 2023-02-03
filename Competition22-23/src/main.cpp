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
// right2               motor         17              a
// right3               motor         12              
// Intake               motor         10              
// Inertial             inertial      3               
// expander             triport       5               
// LeftExpansion        digital_out   E               
// RightMidExpansion    digital_out   G               
// lEncoder             encoder       A, B            
// rEncoder             encoder       E, F            
// mEncoder             encoder       C, D            
// Flap                 digital_out   C               
// goalCam              vision        4               
// Expansion            digital_out   B               
// rollerColor          optical       8               
// colorSwitch          potV2         G               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "autonomous.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here


float greatestDrivePower;

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

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  Inertial.calibrate();
  rollerColor.setLightPower(100);

  left1.setBrake(coast);
  left2.setBrake(coast);
  left3.setBrake(coast);
  right1.setBrake(coast);
  right2.setBrake(coast);
  right3.setBrake(coast);

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
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
  //Win();
  //OffRoller();
  Skills();
  //OnRoller();
  // OffRoller();
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
  //thread startOdom(odometryInertial);
  thread viewVision(visionAim);

   if(Controller1.Axis4.position()<50){
     if (counter == 0){
       task runPID(startup);
       counter ++;
     }
     task runPID(FlyWheelPIDRPM); 
   }
  
  expandTimer.reset();
  while (true) {


    if(Controller1.ButtonA.pressing()){
      tripleshot();
    }

    if(Controller1.ButtonRight.pressing()){
      //volts = 9.5;
      powerLevel++;
      waitUntil(!Controller1.ButtonRight.pressing());
    }
  
    if(Controller1.ButtonLeft.pressing()){
      powerLevel--;
      Flywheel.spin(forward, 2030 + (75*powerLevel), rpm);
      waitUntil(!Controller1.ButtonLeft.pressing());
    }

    if(Controller1.ButtonB.pressing()){
      powerLevel = 3;
    }

    rotateSpeed = 2000 + (75*powerLevel);
    Controller1.Screen.print(powerLevel);

    // thread startFlywheel(autoPower);
    //heat = (Flywheel.voltage()*Flywheel.current()) - (Flywheel.torque()*Flywheel.velocity(dps)*0.3142);

    greatestDrivePower = abs(Controller1.Axis3.position()) + abs(Controller1.Axis1.position());
    if(greatestDrivePower > 100){
      greatestDrivePower = 100/greatestDrivePower;
    }else{
      greatestDrivePower = 1;
    }
    leftDrive.spin(forward, (Controller1.Axis3.position() + Controller1.Axis1.position()) * greatestDrivePower, percent);
    rightDrive.spin(forward, (Controller1.Axis3.position() - Controller1.Axis1.position()) * greatestDrivePower, percent);
  
    if (Controller1.Axis3.value() == 0 && Controller1.Axis1.value() == 0) {
        leftDrive.stop(coast);
        rightDrive.stop(coast);
    }
    
    if (Controller1.ButtonR2.pressing()){
      Intake.spin(forward, 100, percent);
    } else if (Controller1.ButtonR1.pressing()){ 
      Intake.spin(reverse, 50, percent);
    } else if(!autospinning){
      Intake.stop(coast);
    }

    /*if(Controller1.ButtonL2.pressing()){
      thread roll(rollToColor);
      waitUntil(!Controller1.ButtonL2.pressing());
    }
    */

    //Controller1.ButtonR1.pressed(shoot);
    // if(Controller1.ButtonB.pressing()){
    //   autoshoot(-10);
    //   waitUntil(!Controller1.ButtonR2.pressing());
    // }
    // if(Controller1.ButtonX.pressing()){
    //   if(toggleAutoSpeed){
    //     toggleAutoSpeed = false;
    //   }else{
    //     toggleAutoSpeed = true;
    //   }
    //   waitUntil(!Controller1.ButtonX.pressing());
    // }
    if(Controller1.ButtonY.pressing() && expandTimer.time(sec) > 95){
      Expansion.set(true);
    }

    /*if(Controller1.ButtonY.pressing()){
      Expansion.set(true);
    }

    if(Controller1.ButtonX.pressing()){
      const int targetX = 162;
      turn(Inertial.yaw() - (0.25 * (targetX-centerX)));
    }*/

    if(Controller1.ButtonL1.pressing()){
      if(Flap.value()){
        Flap.set(false);
        waitUntil(!Controller1.ButtonL1.pressing());
      } else {
        Flap.set(true);
        waitUntil(!Controller1.ButtonL1.pressing());
      }
    }

   /* if(Controller1.ButtonUp.pressing()){
      powerLevel = 3;
      Flap.set(true);
      waitUntil(!Controller1.ButtonDown.pressing());
    }

    if(Controller1.ButtonDown.pressing()){
      Flap.set(false);
      powerLevel = 6.3;
      waitUntil(!Controller1.ButtonDown.pressing());
    }*/

    Controller1.Screen.setCursor(1, 1);
    Brain.Screen.clearScreen();
     Brain.Screen.printAt(15, 20, "Hue: %f", rollerColor.hue());
    // Brain.Screen.printAt(15, 40, "    Voltage: %f", Flywheel.velocity(rpm)*6);
    // Brain.Screen.printAt(15, 55, "      Power: %f", Flywheel.voltage());
    // Brain.Screen.printAt(15, 70, "     Torque: %f", Flywheel.torque());
    // Brain.Screen.printAt(15, 85, "        RPM: %f", Flywheel.velocity(rpm)*6);
    // Brain.Screen.printAt(15, 100, "Efficiency; %f", Flywheel.efficiency());
    // Brain.Screen.printAt(15, 115, " Heat Loss: %f", heat);
    // Brain.Screen.printAt(15, 130, "Resistance: %f", Flywheel.voltage()/Flywheel.current());
    // Brain.Screen.printAt(15, 145, "     Color: %f", rollerColor.hue());
    // Controller1.Screen.setCursor(1,1);
    // Controller1.Screen.print("%.2f",volts);

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
