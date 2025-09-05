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
motor LF = motor(PORT1, ratio6_1, true);
motor LM = motor(PORT2, ratio6_1, true);
motor LB = motor(PORT10, ratio6_1,  false);
motor RF = motor(PORT18, ratio6_1, false);
motor RM = motor(PORT19, ratio6_1, false);
motor RB = motor(PORT20, ratio6_1, true);
motor topIntake = motor(PORT6, ratio18_1);
motor backIntake = motor(PORT12, ratio18_1);
motor middleIntake = motor(PORT7, ratio18_1);
motor frontIntake = motor(PORT5, ratio18_1);
digital_out topPiston =  digital_out(Brain.ThreeWirePort.A);
digital_out sidePiston = digital_out(Brain.ThreeWirePort.H);
motor_group leftSide = motor_group(LF, LB, LM);
motor_group rightSide = motor_group(RF, RB, RM);
bool topPistonExtended = false;
bool sidePistonExtended = false;
// pid constants
double pi = 3.1415926;
double diameter = 3.25;
double g = 36.0/48.0; // gear ratio 

void drive(int lspeed, int rspeed, int wt) {
  leftSide.spin(forward, lspeed, pct);
  rightSide.spin(forward, rspeed, pct);
  wait(wt, msec);
}
// controls top pistons
void pistonControlTop() {
  topPistonExtended = !topPistonExtended;
  topPiston.set(topPistonExtended);
}

// controls side piston (goal intake)
void pistonControlSide() {
  sidePistonExtended = !sidePistonExtended;
  sidePiston.set(sidePistonExtended);
}

void inchDrive(float target) {
  while (1) {
    double leftEncoders = leftSide.position(degrees);
    double rightEncoders = rightSide.position(degrees);
    double currentPosition = (leftEncoders + rightEncoders) / 2;
    float kp = 2.0;
    float kd = 0.0;
    float error = 5.0;
    leftSide.resetPosition();
    rightSide.resetPosition();
    float x = 0.0;
    float accuracy = 0.5;
    float prevError = 0.0;
    float derivative;
    float output;
    int time = 10;
    while (error > accuracy) {
      x = (currentPosition * pi * diameter) / 360;
      error = (target - x);
      derivative = (error - prevError) / time;
      output = kp * error + (kd * derivative);
      leftSide.spin(forward, error, pct);
      rightSide.spin(forward, error, pct);
      prevError = error; 
      wait(time, msec);
    }
    leftSide.stop();
    rightSide.stop();
  }
  
}

void pre_auton(void) {

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

void autonomous(void) {
  leftSide.setStopping(brake);
  rightSide.setStopping(brake);
  inchDrive(10);
}

void usercontrol(void) {
  // User control code here, inside the loop
  Controller1.ButtonX.pressed(pistonControlTop);
  Controller1.ButtonA.pressed(pistonControlSide);
  while (1) {
    // --------------------- DRIVE CONTROL -----------------------
    double sensitivity = 0.5;
    int leftSpeed = (Controller1.Axis3.position(pct) + Controller1.Axis1.position(pct)) * sensitivity;
    int rightSpeed = (Controller1.Axis3.position(pct) - Controller1.Axis1.position(pct)) * sensitivity;
    drive (leftSpeed, rightSpeed, 10);
    // button to control the piston that controls if top intake spins into storage or into top goal
    // ---------------------- INTAKE CONTROL --------------------
    // midddle goal
    if (Controller1.ButtonR2.pressing()) {
      frontIntake.spin(forward, 100, pct);
      //middleIntake.spin(reverse, 100, pct);
      topIntake.spin(forward, 100, pct);
    }
    // top goal
    else if (Controller1.ButtonR1.pressing()) {
      frontIntake.spin(forward, 100, pct);
      // middleIntake.spin(reverse, 100, pct);
      topIntake.spin(reverse, 100, pct);
    }
    else {
      frontIntake.stop();
      // middleIntake.stop();
      topIntake.stop();
    }
    // intake into storage
    if (Controller1.ButtonL1.pressing()) {
      backIntake.spin(forward, 20, pct);
      middleIntake.spin(reverse, 100, pct);
    }
    // remove from storage
    else if (Controller1.ButtonL2.pressing()) {
      backIntake.spin(reverse, 100, pct);
      middleIntake.spin(forward, 100, pct);
    }
    else {
      backIntake.stop();
      middleIntake.stop();
    }
    // go from back to front intake
    if (Controller1.ButtonL1.pressing() && Controller1.ButtonL2.pressing()) {
      backIntake.spin(forward, 100, pct);
      middleIntake.spin(reverse, 100, pct);
      frontIntake.spin(reverse, 100, pct);
    }
    // outtake from front intake 
    else if (Controller1.ButtonR2.pressing() && Controller1.ButtonR1.pressing()) {
      frontIntake.spin(reverse, 100, pct);
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