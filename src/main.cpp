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
motor leftF = motor(PORT15, true);
motor leftB = motor(PORT16, true);
motor leftT = motor(PORT17, false); // port 3 is broken
motor intakeMotor = motor(PORT4, false);

motor rightF = motor(PORT6, false);
motor rightB = motor(PORT7, false);
motor rightT = motor(PORT20, true);

inertial inert = inertial(PORT11);

// const int tile = 340;
// int current_auton_selection = 0;
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
}




/*---------------------------------------------------------------------------*/
/*                              Autonomous Task                              */
/*---------------------------------------------------------------------------*/


void driveBackward(double targ) {
  leftF.resetPosition();
  leftT.resetPosition();
  leftB.resetPosition();
  rightF.resetPosition();
  rightT.resetPosition();
  rightB.resetPosition();
  double dist = targ*340;
  double avgPosition = (leftF.position(degrees) + leftT.position(degrees) + leftB.position(degrees) + rightF.position(degrees) + rightT.position(degrees) + rightB.position(degrees))/6;

  double derivative;
  double previousError;
  double error = dist + avgPosition;
  double kp = 0.15;
  double kd  = 0;

  while (fabs(error) > 3) {
    previousError = error;
    avgPosition = (leftF.position(degrees) + leftT.position(degrees) + leftB.position(degrees) + rightF.position(degrees) + rightT.position(degrees) + rightB.position(degrees))/6;
    error = dist + avgPosition;
    derivative = error - previousError;

    double speed = error*kp - derivative*kd;
    if (speed > 75) {
      speed = 75;
    }

    leftF.spin(reverse, speed, pct);
    leftT.spin(reverse, speed, pct);
    leftB.spin(reverse, speed, pct);
    rightF.spin(reverse, speed, pct);
    rightT.spin(reverse, speed, pct);
    rightB.spin(reverse, speed, pct);
  }

  leftF.stop();
  leftT.stop();
  leftB.stop();
  rightF.stop();
  rightT.stop();
  rightB.stop();
}

// void test() {
//   driveBackward(1);
// }

/* this function is the thing that lets you have multiple slots for auton (clicking the brain switches slots and switches auton)
   if you get auton to work, this is where you should put in the code for the different corners and which slot you want each auton to go in.
   it is currently coded to have the defaultAuton(), driving forward 5 inches, in the first slot, so you will not have to click the brain to 
   select the first auton */

void autonomous(void) {
  auto_started = true;
  driveBackward(1);
} 

int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  pre_auton();

  while (true) {
    wait(100, msec);
  }
}
