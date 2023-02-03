#include "vex.h"

using namespace vex;

motor_group leftDrive(left3, left2, left1);
motor_group rightDrive(right1, right2, right3);
motor_group fullDrive(left1, left2, left3, right1, right2, right3);


//---------------- AUTONOMOUS DRIVING ------------------------------------------------------------------------------
 

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
void turn(float angle, bool fast = false){ //function for turning. Spins with a speed cap of 36 percent, uses proportional correction
  float trueRotation = Inertial.rotation() * 1.0227;
  float error = angle-(trueRotation);
  while(fabs(error)>2){ //exits loop if error <2 and rotational speed <1
    trueRotation = Inertial.rotation() * 1.0227;
    error = angle-(trueRotation);//calculates error value
    if(!fast){
      if(fabs(error)>54){ //if error is greater than 50, use proportional correction. if not, turn at 36 percent speed
        leftDrive.spin(forward,36*(fabs(error)/error),percent);
        rightDrive.spin(reverse,36*(fabs(error)/error),percent);
      }else{
        leftDrive.spin(forward,error*0.73,percent);
        rightDrive.spin(reverse,error*0.73,percent);
      }
    }else{
      if(fabs(error)>54){ //if error is greater than 50, use proportional correction. if not, turn at 36 percent speed
        leftDrive.spin(forward,50*(fabs(error)/error),percent);
        rightDrive.spin(reverse,50*(fabs(error)/error),percent);
      }else{
        leftDrive.spin(forward,error*0.79,percent);
        rightDrive.spin(reverse,error*0.79,percent);
      }
    }
  }
  leftDrive.stop();
  rightDrive.stop();
}

void printHeading(){
  while(1){
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print(Inertial.rotation() * 1.0227);
    wait(50, msec);
  }
}


//--------------------- FLYWHEEL ---------------------------------------------------------------

int counter = 0;
int finalSpeed = 1800;
int rotateSpeed = 2375;
float powerLevel = 3;
int error;
int prevError;
int derivative;
int integral;
int diff;
int currSpeed;
bool enableLogistic = true;
bool enableFlyPID = false;

double kP = 0.03;
double kI = 0.0;
double kD = 0.0035;

bool toggleAutoSpeed = false;

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

int FlyWheelPIDRPM() {
  waitUntil(enableFlyPID);
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


//-------------------- ODOMETRY ----------------------------------------------------------


float xPos = 0;
float yPos = 0;
float heading;
float headingD;
float goalX = 16;
float goalY = -109;

const float degreesToRadians = 2 * 3.141593 / 360.0;
const float degreesToInches = 2.75 * 3.141593 / 360.0; //with a 2.75 in diameter wheel

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


//------------------- DRIVER -----------------------------------------------------------


bool autospinning = false;
timer rollTimer;

void rollNextColor(){
  if(rollerColor.isNearObject()){
    autospinning = true;
    if(rollerColor.hue() < 300 and rollerColor.hue() > 190){
      while(rollerColor.hue() < 300 and rollerColor.hue() > 190){
        Intake.spin(forward, 45, percent);
      }
    }else{
      while(rollerColor.hue() > 300 or rollerColor.hue() < 100){
        Intake.spin(forward, 45, percent);
      }
    }
    autospinning = false;
  }
}

void rollRed(){
  rollTimer.reset();
  //rollTimer.start();
  if(rollerColor.isNearObject()){
    autospinning = true;
    if(rollerColor.hue() > 300 or rollerColor.hue() < 200){
      while(rollerColor.hue() > 300 or rollerColor.hue() < 100){
        if(rollTimer.value() > 2.5){
          Brain.Screen.print("3 sec");
          break;

        }
        Intake.spin(forward, 100, percent);
      }
      Intake.stop();
    }
    autospinning = false;
  }
}

void rollToColor(){
  if(rollerColor.isNearObject()){
    autospinning = true;
    rollTimer.reset();
    if(colorSwitch.value(percent) < 50){
      while(rollerColor.hue() > 300 || rollerColor.hue() < 100){
        Intake.spin(forward, 90, percent);
        if(rollTimer.time(sec) > 3){
          break;
        }
      }
    }else{
      while(rollerColor.hue() < 300 && rollerColor.hue() > 100){
        Intake.spin(forward, 90, percent);
        if(rollTimer.time(sec) > 3){
          break;
        }
      }
    }
    Intake.stop();
    autospinning = false;
  }
}

void tripleshot(){
  Intake.spinFor(reverse, 0.5, sec, 70, rpm);
  rotateSpeed = 3500;
  wait(0.12, sec);
  Intake.spinFor(reverse, 1.5, sec, 70, rpm);
  //wait(0.2, sec);
  Flywheel.spin(forward, 2000, rpm);
  wait(0.2, sec);
  rotateSpeed = 2225;
}


int centerX;
void visionAim(){
  int higherObject;
  float goalDistance;

  while(true){
    Brain.Screen.clearScreen();
    Brain.Screen.setOrigin(1, 1);
    Brain.Screen.drawRectangle(0, 0, 316, 212);

    if(colorSwitch.value(percent)<50){
      goalCam.takeSnapshot(goalCam__REDGOAL);
    }else{
      goalCam.takeSnapshot(goalCam__BLUEGOAL);
    }
    if(goalCam.objects[1].exists){
      if(goalCam.objects[0].centerY < goalCam.objects[1].centerY){
        higherObject = 0;
      }else{
        higherObject = 1;
      }
      // Brain.Screen.drawRectangle(goalCam.objects[higherObject].originX, goalCam.objects[higherObject].originY, goalCam.objects[higherObject].width, goalCam.objects[higherObject].height, color::red);
      // Brain.Screen.drawRectangle(goalCam.objects[!higherObject].originX, goalCam.objects[!higherObject].originY, goalCam.objects[!higherObject].width, goalCam.objects[!higherObject].height, color::purple);
    }else{
      higherObject = 0;
      // Brain.Screen.drawRectangle(goalCam.objects[0].originX, goalCam.objects[0].originY, goalCam.objects[0].width, goalCam.objects[0].height, color::red);
    }

    // https://www.desmos.com/calculator/imonsqlj9j calculate distance
    goalDistance = 5.3 * pow(1.0125, (goalCam.objects[higherObject].originY + goalCam.objects[higherObject].height)) + 4;
    centerX = goalCam.objects[higherObject].centerX;

    // Brain.Screen.printAt(320, 20, "bottom y: %d", goalCam.objects[higherObject].originY + goalCam.objects[higherObject].height);
    // Brain.Screen.printAt(320, 35, "center y: %d", goalCam.objects[higherObject].centerY);
    // Brain.Screen.printAt(320, 50, "distance: %f", goalDistance);
    // Brain.Screen.printAt(320, 65, "center X: %d", centerX);
    // Brain.Screen.printAt(320, 80, "heading: %f", Inertial.yaw());

    wait(0.2, sec);
  }
}
