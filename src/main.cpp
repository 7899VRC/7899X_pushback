/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       student                                                   */
/*    Created:      8/13/2025, 5:38:21 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
brain Brain;
controller Controller1;
motor LF = motor(PORT1, ratio6_1);
motor LB = motor(PORT10, ratio6_1);
motor LM = motor(PORT2, ratio6_1);
motor RF = motor(PORT18, ratio6_1);
motor RB = motor(PORT20, ratio6_1);
motor RM = motor(PORT19, ratio6_1);
motor topIntake = motor(PORT6, ratio18_1);
motor backIntake = motor(PORT12, ratio18_1, true);
motor middleIntake = motor(PORT7, ratio18_1);
motor frontIntake = motor(PORT5, ratio18_1);
digital_out topPiston =  digital_out(Brain.ThreeWirePort.A);
//digital_out matchLoadPiston = digital_out()

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
    // button to control the piston that controls if top intake spins into storage or into top goal
    if (Controller1.ButtonX.pressing()) {
      topPiston.set(true);
    }
    else {
      topPiston.set(false);
    }
    // midddle goal
    if (Controller1.ButtonR2.pressing()) {
      frontIntake.spin(forward, 100, pct);
      middleIntake.spin(reverse, 100, pct);
      topIntake.spin(forward, 100, pct);
    }
    // top goal
    else if (Controller1.ButtonR1.pressing()) {
      frontIntake.spin(forward, 100, pct);
      middleIntake.spin(reverse, 100, pct);
      topIntake.spin(reverse, 100, pct);
    }
    else {
      frontIntake.stop();
      middleIntake.stop();
      topIntake.stop();
    }
    // intake into storage
    if (Controller1.ButtonL1.pressing()) {
      backIntake.spin(forward, 100, pct);
      middleIntake.spin(forward, 100, pct);
    }
    // remove from storage
    else if (Controller1.ButtonL2.pressing()) {
      backIntake.spin(reverse, 100, pct);
      middleIntake.spin(reverse, 100, pct);
    }
    else {
      backIntake.stop();
      middleIntake.stop();
    }
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
