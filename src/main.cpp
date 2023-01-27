#include "main.h"
#include "pros/misc.h"
#include "globals.hpp"
#include "autons.hpp"


/////
// For instalattion, upgrading, documentations and tutorials, check out website!
// https://ez-robotics.github.io/EZ-Template/
/////


// Chassis constructor
Drive chassis (
  // Left Chassis Ports (negative port will reverse it!)
  //`   the first port is the sensored port (when trackers are not used!)
  {-3, -5, -17} //top

  // Right Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  ,{16, 8, 14}

  // IMU Port
  ,21

  // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
  //    (or tracking wheel diameter)
  ,4.125

  // Cartridge RPM
  //   (or tick per rotation if using tracking wheels)
  ,600

  // External Gear Ratio (MUST BE DECIMAL)
  //    (or gear ratio of tracking wheel)
  // eg. if your drive is 84:36 where the 36t is powered, your RATIO would be 2.333.
  // eg. if your drive is 36:60 where the 60t is powered, your RATIO would be 0.6.
  ,2.333

  // Uncomment if using tracking wheels
  /*
  // Left Tracking Wheel Ports (negative port will reverse it!)
  // ,{1, 2} // 3 wire encoder
  // ,8 // Rotation sensor

  // Right Tracking Wheel Ports (negative port will reverse it!)
  // ,{-3, -4} // 3 wire encoder
  // ,-9 // Rotation sensor
  */

  // Uncomment if tracking wheels are plugged into a 3 wire expander
  // 3 Wire Port Expander Smart Port
  // ,1
);





/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
  ez::print_ez_template();
  
  pros::delay(500); // Stop the user from doing anything while legacy ports configure.

  expansion1.set_value(false);

  // Configure your chassis controls
  chassis.toggle_modify_curve_with_controller(false); // Enables modifying the controller curve with buttons on the joysticks
  chassis.set_active_brake(0); // Sets the active brake kP. We recommend 0.1.
  chassis.set_curve_default(0, 0); // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)  
  default_constants(); // Set the drive to your own constants from autons.cpp!
  exit_condition_defaults(); // Set the exit conditions to your own constants from autons.cpp!

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  chassis.set_left_curve_buttons (pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT); // If using tank, only the left side is used. 
  // chassis.set_right_curve_buttons(pros::E_CONTROLLER_DIGITAL_Y,    pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.add_autons({
    Auton("", autonright)
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  sylib::initialize();
}




/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  expansion1.set_value(true);
}


/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  expansion1.set_value(false);
}



/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
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
    if ( (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1))) { // When L1 pressed,
      if (flywheel_mode != 4) { // If flywheel not running at speed 4,
        flywheel.set_velocity_custom_controller(110) ; // Run Flywheel Sp4
        flywheel_mode = 4;
      } else { // If flywheel already running, 
        flywheel.set_velocity_custom_controller(110) ; // Turn off flywheel motor
        flywheel_mode = 0;
      }
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){ // When A pressed,
      if (flywheel_mode != 1) { // If flywheel not running at speed 1,
        flywheel.set_velocity_custom_controller(110) ; // Run Flywheel Sp1
        flywheel_mode = 1;
      } else { // If flywheel already running, 
        flywheel.set_velocity_custom_controller(110) ; // Turn off flywheel motor
        flywheel_mode = 0;
      }
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){ // When B pressed,
      if (flywheel_mode != 2) { // If flywheel not running at speed 2,
        flywheel.set_velocity_custom_controller(110) ; // Run Flywheel Sp2
        flywheel_mode = 2;
      } else { // If flywheel already running, 
        flywheel.set_velocity_custom_controller(110) ; // Turn off flywheel motor
        flywheel_mode = 0;
      }
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){ // When X pressed,
      if (flywheel_mode != 3) { // If flywheel not running at speed 3,
        flywheel.set_velocity_custom_controller(110) ; // Run Flywheel Sp3
        flywheel_mode = 3;
      } else { // If flywheel already running, 
        flywheel.set_velocity_custom_controller(0) ; // Turn off flywheel motor
        flywheel_mode = 0;
      }
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){ // When Y pressed, 
      expansion1.set_value(true);
    }

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}

