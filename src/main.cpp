#include "main.h"
#include "pros/misc.h"
#include "globals.hpp"


/////
// For instalattion, upgrading, documentations and tutorials, check out website!
// https://ez-robotics.github.io/EZ-Template/
/////



//FLYWHEEL CONSTANTS-------------------------------------
int currentSpeed;
int targetSpeed = 127;
int error;
float Kp = 0.5;
float Ki = 0.1;
float Kd = 0.1;
double targetSpeed;
//FLYWHEEL CONSTANTS--------------------------------------






// Chassis constructor
Drive chassis (
  
  {-3, -5, -17} ,{16, 8, 14}, 21, 4.125, 600, 2.333

);

void initialize() {
  // Print our branding over your terminal :D
  ez::print_ez_template();
  
  pros::delay(500); // Stop the user from doing anything while legacy ports configure.

  expansion1.set_value(false);

  // Configure your chassis controls
  chassis.toggle_modify_curve_with_controller(true); // Enables modifying the controller curve with buttons on the joysticks
  chassis.set_active_brake(0.1); // Sets the active brake kP. We recommend 0.1.
  chassis.set_curve_default(0.1, 0); // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)  
  default_constants(); // Set the drive to your own constants from autons.cpp!
  exit_condition_defaults(); // Set the exit conditions to your own constants from autons.cpp!

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  chassis.set_left_curve_buttons (pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT); // If using tank, only the left side is used. 
  // chassis.set_right_curve_buttons(pros::E_CONTROLLER_DIGITAL_Y,    pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.add_autons({
    Auton ("", autonright),
    Auton("", autonleft),
    Auton("", autonskills),
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  sylib::initialize();
}




void disabled() {
  expansion1.set_value(true);
}



void competition_initialize() {
  expansion1.set_value(false);
}




void autonomous() {
  expansion1.set_value(false);

  chassis.reset_pid_targets(); // Resets PID targets to 0
  chassis.reset_gyro(); // Reset gyro position to 0
  chassis.reset_drive_sensor(); // Reset drive sensors to 0
  chassis.set_drive_brake(MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps autonomous consistency.
  chassis.set_angle(0);
  chassis.set_max_speed(115);

  ez::as::auton_selector.call_selected_auton(); // Calls selected auton from autonomous selector.
}


/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void opcontrol() {
  // This is preference to what you like to drive on.
  expansion1.set_value(false);
  chassis.set_drive_brake(MOTOR_BRAKE_COAST);
  flywheel.set_velocity(3600);
  currentSpeed = flywheel.get_velocity();
  targetSpeed = currentSpeed/2;

  
  int intake_mode = 0; // Sets up intake control for buttons
  int flywheel_mode = 0; //Sets up flywheel control for buttons
  
  while (true) {
    
    

    chassis.tank(); // Tank control
    // chassis.arcade_standard(ez::SPLIT); // Standard split arcade
    // chassis.arcade_standard(ez::SINGLE); // Standard single arcade
    // chassis.arcade_flipped(ez::SPLIT); // Flipped split arcade
    // chassis.arcade_flipped(ez::SINGLE); // Flipped single arcade

    // . . .
    // Put more user control code here!
    // . . .
    currentSpeed = flywheel.get_velocity();
    error = targetSpeed- currentSpeed;
    float P = error * Kp;
    float I = error * Ki;
    float D = error * Kd;
    int output = P+I+D;
    flywheel.set_velocity(output+currentSpeed);
    pros::delay(20);

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) { // When R1 pressed,
      if (intake_mode == 0){ // If intake not running,
        intake.move_velocity(170); // Run Intake
        intake_mode = 1;
      } else { // If intake already running,
        intake.move_velocity(0); // Turn off intake motor
        intake_mode = 0;
      }
    }
    // Outtake
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) { // When R2 pressed,
      if (intake_mode == 0){ // If outtake not running,
        intake.move_velocity(-170); // Run Outtake
        intake_mode = -1;
      } else { // If outtake already running,
        intake.move_velocity(0); // Turn off intake motor
        intake_mode = 0;
      }
    }

    // Flywheel
    if ( (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) || (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) ) { // When L1 or L2 pressed,
      if (flywheel_mode != 4) { // If flywheel not running at speed 4,
        targetSpeed = 3600; // Run Flywheel Sp4
        flywheel_mode = 4;
      } else { // If flywheel already running, 
        targetSpeed = 0; // Turn off flywheel motor
        flywheel_mode = 0;
      }
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){ // When A pressed,
      if (flywheel_mode != 1) { // If flywheel not running at speed 1,
        targetSpeed = 3000; // Run Flywheel Sp1
        flywheel_mode = 1;
      } else { // If flywheel already running, 
        targetSpeed = 0; // Turn off flywheel motor
        flywheel_mode = 0;
      }
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){ // When B pressed,
      if (flywheel_mode != 2) { // If flywheel not running at speed 2,
        targetSpeed = 2500; // Run Flywheel Sp2
        flywheel_mode = 2;
      } else { // If flywheel already running, 
        targetSpeed = 0; // Turn off flywheel motor
        flywheel_mode = 0;
      }
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){ // When X pressed,
      if (flywheel_mode != 3) { // If flywheel not running at speed 3,
        targetSpeed = 2000; // Run Flywheel Sp3
        flywheel_mode = 3;
      } else { // If flywheel already running, 
        targetSpeed = 0; // Turn off flywheel motor
        flywheel_mode = 0;
      }
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){ // When Y pressed, 
      expansion1.set_value(true);
    }

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}

