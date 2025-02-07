/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       rishs                                                     */
/*    Created:      9/14/2024, 1:59:02 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <cmath>
#include <iostream>
using namespace vex;

// A global instance of competition
competition Competition;

// this section defines global variables (variables that are used across different functions) and also the motors and vex parts
vex::brain     Brain;
motor leftF = motor(PORT16, true);
motor leftB = motor(PORT17, true);
motor leftT = motor(PORT18, false); // port 3 is broken
motor intakeMotor = motor(PORT1, false);

motor rightF = motor(PORT4, false);
motor rightB = motor(PORT10, false);
motor rightT = motor(PORT7, true);

inertial inert = inertial(PORT11);
controller c = controller();
digital_out doink = digital_out(Brain.ThreeWirePort.A);
digital_out clamp = digital_out(Brain.ThreeWirePort.B);

double gAngle = inert.rotation(degrees);
const int tile = 345;
int current_auton_selection = 0;
bool auto_started = false;



/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*---------------------------------------------------------------------------*/

/* this does things before the actual match; basically you have to get to the actual match AT LEAST 3 seconds
   before the actual match starts, because you need to turn on and run the bot so that it has 3 seconds to calibrate
   an easier way to explain it is that this code runs before the referees turn on the auton part of the comp switch */
void pre_auton(void) {
  inert.calibrate(1);
  wait(3,sec);

  while(!auto_started){
    Brain.Screen.setFillColor(213);
    Brain.Screen.drawRectangle(270, 10, 80, 80);
    Brain.Screen.drawRectangle(355, 10, 80, 80);
    Brain.Screen.setFillColor(0);
    Brain.Screen.drawRectangle(270, 95, 80, 80);
    Brain.Screen.drawRectangle(355, 95, 80, 80);
    Brain.Screen.printAt(270+40, 10+40, "-");
    Brain.Screen.printAt(355+40, 10+40, "+");
    Brain.Screen.printAt(270+40, 95+40, "-");
    Brain.Screen.printAt(355+40, 95+40, "+");

    Brain.Screen.printAt(5, 170, "SELECTED AUTON:");
    switch(current_auton_selection){
      case 0:
        Brain.Screen.printAt(5, 150, "Auton 1");
        break;
      case 1:
        Brain.Screen.printAt(5, 150, "Auton 2");
        break;
      case 2:
        Brain.Screen.printAt(5, 150, "Auton 3");
        break;
      case 3:
        Brain.Screen.printAt(5, 150, "Auton 4");
        break;
      case 4:
        Brain.Screen.printAt(5, 150, "Auton 5");
        break;
      case 5:
        Brain.Screen.printAt(5, 150, "Auton 6");
        break;
      case 6:
        Brain.Screen.printAt(5, 150, "Auton 7");
        break;
      case 7:
        Brain.Screen.printAt(5, 150, "Auton 8");
        break;
    }
    if(Brain.Screen.pressing()){
      while(Brain.Screen.pressing()) {}
      current_auton_selection ++;
    } else if (current_auton_selection == 8){
      current_auton_selection = 0;
    }
    task::sleep(10);
  }
}




/*---------------------------------------------------------------------------*/
/*                              Autonomous Task                              */
/*---------------------------------------------------------------------------*/


/* the next few functions are the basic auton components as shown by their names
   they are all coded in PID except for regLeft(), which is a test i was trying for auton
   the code was getting stuck in the PID turnLeft function, as i wrote below, so I was 
   going to try using regular left turning but I didn't get a chance to. You can use the 2 hours 
   either trying to get turning to work with auton or practicing driving strategy */

void drive(double target) {
  leftF.resetPosition();
  rightF.resetPosition();
  target *= tile;
  double derivative;
  double error = target - (leftF.position(degrees) + rightF.position(degrees))/2;
  double kp = 1.2;
  double kd = 0.5;

  while (fabs(error) > 3) {
    double previousError = error;
    error = target - (leftF.position(degrees) + rightF.position(degrees))/2;
    derivative = error - previousError;

    double speed = error*kp - derivative*kd;
    if (speed > 40) speed = 40;
    if (speed < -40) speed = -40;

    leftT.spin(forward, speed, pct);
    leftF.spin(forward, speed, pct);
    leftB.spin(forward, speed, pct);
    rightF.spin(forward, speed, pct);
    rightT.spin(forward, speed, pct);
    rightB.spin(forward, speed, pct);
    wait(10, msec);
  }

  leftF.stop(brake);
  leftT.stop(brake);
  leftB.stop(brake);
  rightF.stop(brake);
  rightT.stop(brake);
  rightB.stop(brake);
}

// // this is a very intelligent piece of code that makes the robot turn left and makes the team very happy because it allows our robot to turn left to grab the points and make us very hpapy
void turnLeft(double angle) {
  inert.resetRotation();
  double kp = 0.53;
  // double ki = 0.1;
  double error = angle + inert.rotation(degrees);
  double kd  = 0.08;
  double derivative = 0.0;
  // double integral = 0.0;
  
  while (fabs(error) > 2) {
    double previousError = error;
    error = angle + inert.rotation(degrees);
    derivative = error - previousError;
    // integral += error;

    double speed = error*kp - derivative*kd;

    leftF.spin(reverse, speed*0.5, pct);
    leftB.spin(reverse, speed*0.5, pct);
    leftT.spin(reverse, speed*0.5, pct);
    rightF.spin(forward, speed*0.5, pct);
    rightT.spin(forward, speed*0.5, pct);
    rightB.spin(forward, speed*0.5, pct);
  }

    leftF.stop(brake);
    leftT.stop(brake);
    leftB.stop(brake);
    rightF.stop(brake);
    rightT.stop(brake);
    rightB.stop(brake);
}

void turnRight(double angle) {
  inert.resetRotation();
  double kp = 0.53;
  // double ki = 0.1;
  double error = angle - inert.rotation(degrees);
  double kd  = 0.08;
  double derivative = 0.0;
  // double integral = 0.0;
  
  while (fabs(error) > 2) {
    double previousError = error;
    error = angle - inert.rotation(degrees);
    derivative = error - previousError;
    // integral+=error;

    double speed = error*kp - derivative*kd;

    leftF.spin(forward, speed*0.5, pct);
    leftB.spin(forward, speed*0.5, pct);
    leftT.spin(forward, speed*0.5, pct);
    rightF.spin(reverse, speed*0.5, pct);
    rightT.spin(reverse, speed*0.5, pct);
    rightB.spin(reverse, speed*0.5, pct);
  }

    leftF.stop(brake);
    leftT.stop(brake);
    leftB.stop(brake);
    rightF.stop(brake);
    rightT.stop(brake);
    rightB.stop(brake);
}

void brainDisplay() {
  while (true) {
    Brain.Screen.printAt(10, 30, "Global Left %f", leftB.position(degrees));
    Brain.Screen.printAt(10, 50, "GLobal Right %f", rightB.position(degrees));
    Brain.Screen.printAt(10, 70, "Left %f", leftF.position(degrees));
    Brain.Screen.printAt(10, 90, "Right %f", rightF.position(degrees));
    Brain.Screen.printAt(10, 110, "Inertial %f", inert.rotation(degrees));
    wait(20, msec);
  }
}



// // ============================================== AUTON FUNCTIONS ===================================================

/* the next 4 functions are for the auton for each respective corner. 
   the issue that wasn't working was that the code kept getting stuck in the turning functions
   the best way i think you can solve this is by using normal turn instead of PID turning, and 
   i made a new turning function called regLeft() instead of turnLeft(), so try using that 
   instead */

// void bluePos() {
//   // STARTING: https://www.youtube.com/watch?v=mfGBy_0xaxo
//   // Roughly a 60 degree angle from the wall we are at (facing the closest mogo) and right next to the red ring to the left of us 
//   driveBackward(hypot(0.5, 1));
//   clamp.set(true);
//   intakeMotor.spin(forward, 70, pct);
//   turnRight(60);
//   driveForward(0.75);
//   turnRight(120);
//   driveBackward(hypot(3, 1.5));
//   clamp.set(false);
//   driveForward(hypot(3, 1.5));
//   turnLeft(5);
//   driveForward(hypot(0.5, 1)); // ladder
// }

// void blueNeg() {
//   // STARTING: facing the nearest mogo
//   // NOTE: when possible, start by putting preload on alliance stake
//   driveBackward(1);
//   clamp.set(true);
//   intakeMotor.spin(forward, 70, pct);
//   driveBackward(0.2);
//   turnLeft(90);
//   driveForward(1*tile);
//   turnLeft(90);
//   driveForward(1*tile);
//   driveBackward(0.5*tile);
//   turnRight(45);
//   driveForward(hypot(0.5, 0.5)*tile);
//   driveBackward(hypot(1, 1)*tile);
//   turnLeft(135);
//   driveForward(1*tile); // ladder
// }

// void redPos() {
//   // STARTING: https://www.youtube.com/watch?v=mfGBy_0xaxo
//   // Roughly a 60 degree angle from the wall we are at (facing the closest mogo) and right next to the red ring to the left of us 
//   driveBackward(1.5*tile);
//   clamp.set(true);
//   intakeMotor.spin(forward, 70, pct);
//   turnLeft(90);
//   wait(0.5, sec);
//   driveForward(0.75*tile);
//   // wait(0.5, sec);
//   // turnLeft(120);
//   // driveBackward(hypot(3, 1.5)*tile);
//   // clamp.set(false);
//   // driveForward(hypot(3, 1.5)*tile);
//   // turnRight(5);
//   // driveForward(hypot(0.5, 1)*tile); // ladder
// }

// void redNeg() {
//   // STARTING: facing the nearest mogo
//   // NOTE: when possible, start by putting preload on alliance stake
//   driveBackward(1*tile);
//   clamp.set(true);
//   intakeMotor.spin(forward, 70, pct);
//   driveBackward(0.2*tile);
//   turnRight(90);
//   driveForward(1*tile);
//   turnRight(90);
//   driveForward(1*tile);
//   driveBackward(0.5*tile);
//   turnLeft(45);
//   driveForward(hypot(0.5, 0.5)*tile);
//   driveBackward(hypot(1, 1)*tile);
//   turnRight(135);
//   driveForward(1*tile); // ladder
// }


void test() { // auton testing
  drive(-1);
  turnLeft(90);
  clamp.set(true);
}

void autonSkills() { // unfinished skills auton
  drive(-0.2);
  clamp.set(true);
  intakeMotor.spin(forward, 100, pct);
  turnRight(90);
  drive(1);
  wait(0.3, sec);
  drive(0.3);
  wait(1, sec);
  turnRight(110);
  drive(1.4);
  turnRight(10);
  wait(1.5, sec);
  drive(-2);
  clamp.set(false);
  drive(0.15);
  turnLeft(120);
  drive(-3.85);
  clamp.set(true);
  // drive(0.25);
  // turnRight(90);
  // drive(1);
  // turnRight(90);
  // drive(1);
  // turnRight(90);
  // drive(1.5);
  // turnLeft(110);
  // drive(-0.5);
  // clamp.set(false);
}



void preloadREDPOS() {
  drive(-1.3);
  clamp.set(true);
  intakeMotor.spin(forward, 100, pct);
  turnLeft(90);
  drive(1);
  turnRight(175);
  drive(1.5);

}
void preloadREDNEG() {
  drive(-1.3);
  clamp.set(true);
  intakeMotor.spin(forward, 100, pct);
  turnLeft(90);
  drive(1);
}
void preloadBLUEPOS() {
  drive(-1.3); 
  clamp.set(true);
  intakeMotor.spin(forward, 70, pct);
  turnRight(90);
  drive(1);
  turnLeft(175);
  drive(1.5);
}
void preloadBLUENEG() {
  drive(-1.3);
  clamp.set(true);
  intakeMotor.spin(forward, 70, pct);
  turnLeft(90);
  drive(1);
}


// /*---------------------------------------------------------------------------*/
// /*                                                                           */
// /*                               TASK FUNCTIONS                              */
// /*                                                                           */
// /*---------------------------------------------------------------------------*/


// // driving control function: call me to explain the code if you need to adjust drive speed or anything else
void usercontrol(void) {
  // User control code here, inside the loop
  clamp.set(false);
  doink.set(false);
  double k = 1;
  while (1) {
    double logDiv = 185;

    int logFD = (c.Axis3.position()*c.Axis3.position()) / logDiv;
    if (c.Axis3.position() < 0) {
      logFD *= -1;
    }
    int logLR = (c.Axis1.position()*c.Axis1.position()) / logDiv;
    if (c.Axis1.position() < 0) {
      logLR *= -1;
    }

    // constant multiplied to speed, currently not in use just for testing purpsoses

    if (c.ButtonR1.PRESSED) {intakeMotor.spin(forward, 100, pct);}
    if (c.ButtonR2.PRESSED) {intakeMotor.spin(reverse, 100, pct);}
    if (c.ButtonR2.RELEASED) {intakeMotor.stop(brake);}
    if (c.ButtonA.PRESSED) {intakeMotor.stop(brake);}
    if (c.ButtonL1.PRESSED) {clamp.set(true);}
    if (c.ButtonL2.PRESSED) {clamp.set(false);}
    if (c.ButtonX.PRESSED) {doink.set(true);}
    if (c.ButtonB.PRESSED) {doink.set(false);}
    if (c.ButtonUp.PRESSED) {k = 10;}
    if (c.ButtonDown.PRESSED) {k = 1;}
    

    leftF.spin(forward, (logFD + logLR*0.5)*k, pct);
    leftT.spin(forward, (logFD + logLR*0.5)*k, pct);
    leftB.spin(forward, (logFD + logLR*0.5)*k, pct);
    rightF.spin(forward, (logFD - logLR*0.5)*k, pct);
    rightT.spin(forward, (logFD - logLR*0.5)*k, pct);
    rightB.spin(forward, (logFD - logLR*0.5)*k, pct);

    wait(20, msec);
  }
}


/* this function is the thing that lets you have multiple slots for auton (clicking the brain switches slots and switches auton)
   if you get auton to work, this is where you should put in the code for the different corners and which slot you want each auton to go in.
   it is currently coded to have the defaultAuton(), driving forward 5 inches, in the first slot, so you will not have to click the brain to 
   select the first auton */



// NEEV AND AADITYA (or whoever is in drivebox): when it says something like "case 4", that means the Brain will read AUTON 5. 
// this is because code counts starting from 0, not 1, but whether or not you understand it, keep this in mind if I am not at a competition.
void autonomous(void) {
  auto_started = true;
  switch(current_auton_selection){ 
    case 0:
      autonSkills();
      // preloadREDPOS();
      break;
    case 1:
      preloadREDNEG();
      break;
    case 2:
      preloadBLUEPOS();
      break;
    case 3:
      preloadBLUENEG();
      break;
    case 4:
      test();
      break;
    case 5:
      break;
    case 6:
      autonSkills();
      break;
    case 7:
      usercontrol();
  }
  // clamp.set(false);
  doink.set(false);
} 

int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  thread t(brainDisplay);
  clamp.set(false);
  doink.set(false);
  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
