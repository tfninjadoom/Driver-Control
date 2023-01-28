#include "main.h"
#include "pros/misc.h"
#include "globals.hpp"


/////
// For instalattion, upgrading, documentations and tutorials, check out website!
// https://ez-robotics.github.io/EZ-Template/
/////

///
// Constants
///
int tune_turn;
// It's best practice to tune constants when the robot is empty and with heavier game objects, or with lifts up vs down.
// If the objects are light or the cog doesn't change much, then there isn't a concern here.
int currentSpeed;
int targetSpeed = 127;
int error;
float Kp = 0.5;
float Ki = 0.1;
float Kd = 0.1;
void default_constants() {
  chassis.set_slew_min_power(80, 80);
  chassis.set_slew_distance(7, 7);
  chassis.set_pid_constants(&chassis.headingPID, 11, 0, 20, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.turnPID, 5, 0.003, 35, 15);
  chassis.set_pid_constants(&chassis.swingPID, 7, 0, 45, 0);
}


void exit_condition_defaults() {
  chassis.set_exit_condition(chassis.turn_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.swing_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.drive_exit, 80, 50, 300, 150, 500, 500);
}

void modified_exit_condition() {
  chassis.set_exit_condition(chassis.turn_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.swing_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.drive_exit, 80, 50, 300, 150, 500, 500);
}


///
// Drive Example
///

// . . .
// Make your own autonomous functions here!
// . . .


void autonflywheel(int rpm) {
  //rpm = rpm/6;
  flywheel.set_velocity_custom_controller(rpm);
  
  /* OLD:
  flywheel.set_velocity(rpm);
  currentSpeed = flywheel.get_velocity();
  targetSpeed = currentSpeed/2;
  while(true){
    currentSpeed = flywheel.get_velocity();
    error = targetSpeed- currentSpeed;
    float P = error * Kp;
    float I = error * Ki;
    float D = error * Kd;
    int output = P+I+D;
    flywheel.set_velocity(output);
    pros::delay(20); 
  */



};

void intakeon() {
  intake.move_velocity(180);
};

void intakeoff() {
  intake.move_velocity(0);
};

void autonroller() {
  intake.move_velocity(-200);
  pros::delay(500);
  intake.move_velocity(0);
};

void autonindex() {
  bool var = true;
  while (var == true) {
    intake.move_velocity(-180);
    pros::delay(5000);
    var = false;
  }
};

void fcompressor() {
  compressor.set_value(!(compressor.get_value()));
}

const int DRIVE_SPEED = 110; // This is 110/127 (around 87% of max speed).  We don't suggest making this 127.
                             // If this is 127 and the robot tries to heading correct, it's only correcting by
                             // making one side slower.  When this is 87%, it's correcting by making one side
                             // faster and one side slower, giving better heading correction.
const int TURN_SPEED  = 90;
const int SWING_SPEED = 90;

int flywheel_start = 3600;
int R1 = 10;
int R2 = 20;

int L1 = 10;
int L2 = 10;

int AWP1 = 20;

int SK1 = 20;
int SK2 = 20;

void autonright() {
  
  autonflywheel(flywheel_start);
  chassis.set_drive_pid(-23.4, DRIVE_SPEED, true);  // Move 23.4 inches backward
  chassis.wait_drive(); 
  chassis.set_turn_pid(90, TURN_SPEED);               // Turn 90 degrees clockwise
  chassis.wait_drive();
  chassis.set_drive_pid(-6, DRIVE_SPEED, true);     // Move 5 inches backward
  chassis.wait_drive();
  autonroller();
  chassis.wait_drive(); /*
  chassis.set_turn_pid(-(10), TURN_SPEED);             // Turn R1 degrees clockwise
  chassis.wait_drive();
  autonindex(2);
  chassis.set_turn_pid(-(45+R1), TURN_SPEED);       // Turn 45+R1 degrees counterclockwise
  chassis.wait_drive();
  chassis.set_drive_pid(17, DRIVE_SPEED, true);     // Move 17 inches forward
  chassis.wait_drive();
  intakeon();
  chassis.set_drive_pid(33.1, DRIVE_SPEED, true);   // Move 33.1 inches forward
  chassis.wait_drive();
  intakeoff();
  chassis.set_turn_pid((90-R2), TURN_SPEED);        // Turn 90-R2 degrees clockwise
  chassis.wait_drive();
  autonindex(2);
  */
}

void autonleft() {
  autonflywheel(flywheel_start);
  chassis.set_drive_pid(-3, DRIVE_SPEED, true);
  autonroller();
  chassis.set_drive_pid(5, DRIVE_SPEED, true);      // Move forward 5 inches
  chassis.wait_drive(); /*
  chassis.set_turn_pid(-L1, TURN_SPEED);            // Turn R1 degrees counterclockwise
  chassis.wait_drive();
  autonindex(2);
  chassis.set_turn_pid((45+L1), TURN_SPEED);        // Turn 45+R1 degrees clockwise
  chassis.wait_drive();
  chassis.set_drive_pid(17, DRIVE_SPEED, true);     // Move forward 17 inches
  chassis.wait_drive();
  intakeon();
  chassis.set_drive_pid(33.1, DRIVE_SPEED, true);   // Move forward 33.1 inches
  chassis.wait_drive();
  intakeoff();
  chassis.set_turn_pid(-(90-L2), TURN_SPEED);       // Turn 90-R2 degrees counterclockwise
  chassis.wait_drive();
  // Index 3 discs pls
  chassis.set_turn_pid((90-L2), TURN_SPEED);        // Turn 90-R2 degrees clockwise
  chassis.wait_drive();
  intakeon();
  chassis.set_turn_pid(18, DRIVE_SPEED);            // Move forward 18 inches
  intakeoff(); */
}

/*
void autonselfawpleft() {
  autonflywheel(flywheel_start);
  autonroller();
  chassis.set_drive_pid(5, DRIVE_SPEED, true);      // Move forward 5 inches
  chassis.wait_drive();
  chassis.set_turn_pid(-L1, TURN_SPEED);            // Turn R1 degrees counterclockwise
  chassis.wait_drive();
  autonindex(2);
  chassis.set_tu  `rn_pid((45+L1), TURN_SPEED);        // Turn 45+R1 degrees clockwise
  chassis.wait_drive();
  chassis.set_drive_pid(132.4, DRIVE_SPEED, true);  // Move forward 132.4 inches
  chassis.wait_drive();
  chassis.set_turn_pid(-135, TURN_SPEED);           // Turn 135 degrees counterclockwise
  chassis.wait_drive();
  chassis.set_drive_pid(-AWP1, DRIVE_SPEED, true);  // Move backward AWP1 inches
  chassis.wait_drive();
  autonroller();
}
*/

/*void autonskills1() {
  autonflywheel();
  autonroller();
  intakeon();
  // swing thing
  intakeoff();
  chassis.set_turn_pid(180, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(-5, DRIVE_SPEED, true);
  chassis.wait_drive();
  autonroller();
  chassis.set_drive_pid(5, DRIVE_SPEED, true);
  chassis.wait_drive();
  chassis.set_turn_pid(-90, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(59, DRIVE_SPEED, true);
  chassis.wait_drive();
  chassis.set_turn_pid(SK1, TURN_SPEED);
  chassis.wait_drive();
  // Index 3 discs pls
  chassis.set_turn_pid((90-SK1), TURN_SPEED);
  chassis.wait_drive();
  intakeon();
  chassis.set_drive_pid(117, DRIVE_SPEED, true);
  chassis.wait_drive();
  intakeoff();
  chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(23.4, DRIVE_SPEED, true);
  chassis.wait_drive();
  chassis.set_turn_pid(SK2, TURN_SPEED);
  chassis.wait_drive();
  // Index 3 discs pls
  chassis.set_turn_pid((-SK2), TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(47, DRIVE_SPEED, true);
  chassis.wait_drive();
  chassis.set_turn_pid(-90, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(-5, DRIVE_SPEED, true);
  chassis.wait_drive();
  autonroller();

}
*/

void skillsauton(){
  autonflywheel(flywheel_start);
  chassis.set_drive_pid(-5, DRIVE_SPEED, true);
  autonroller();
  chassis.set_drive_pid(23.4, DRIVE_SPEED, true);
  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.set_drive_pid(46.8, DRIVE_SPEED, true);
  chassis.set_turn_pid(-90, TURN_SPEED);
  autonindex();
  intakeon();
  chassis.set_drive_pid(-5, DRIVE_SPEED, true);
  chassis.set_turn_pid(90, TURN_SPEED);
  chassis.set_drive_pid(46.8, 90, true);
  chassis.set_turn_pid(-90, TURN_SPEED);
  chassis.set_drive_pid(5, DRIVE_SPEED, true);
  autonindex();
  chassis.set_turn_pid(-45, TURN_SPEED);
  chassis.set_drive_pid(-27.4, DRIVE_SPEED, true);
  autonroller();




}