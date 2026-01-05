#include "main.h"
#include "SubSystems/Chassis.hpp"
#include "SubSystems/Intake.hpp"

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor LEFT_MIDDLE(-3, pros::v5::MotorGears::blue);
pros::Motor LEFT_FRONT(-4, pros::v5::MotorGears::blue);
pros::Motor LEFT_BACK(2, pros::v5::MotorGears::blue);
pros::Motor RIGHT_MIDDLE(6, pros::v5::MotorGears::blue);
pros::Motor RIGHT_FRONT(7, pros::v5::MotorGears::blue);
pros::Motor RIGHT_BACK(-9, pros::v5::MotorGears::blue);

pros::Motor left(1, pros::v5::MotorGears::blue);
pros::Motor right(-10, pros::v5::MotorGears::blue);

pros::Imu inertial(0);
pros::Distance distance(0);
pros::Rotation tracking(0);

pros::adi::DigitalOut bot('A');
pros::adi::DigitalOut top('B');
pros::adi::DigitalOut doinker('C');
pros::adi::DigitalOut tongue('D');
pros::adi::DigitalOut backDoinker('E');
pros::adi::DigitalOut doublePark('F');

Chassis chassis(LEFT_FRONT, RIGHT_FRONT, LEFT_MIDDLE, RIGHT_MIDDLE, LEFT_BACK, RIGHT_BACK, inertial, tracking);
Intake intake(left, right, bot, top, doublePark, distance);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

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
	intake.brake();
	chassis.brake();
	inertial.reset();

	bool doinkerToggle = false;
	bool tongueToggle = false;
	bool backdoinkerToggle = false;
	bool doubleParkToggle = false;

	while(true){

		//Function in chassis class to move according to joystick inputs
		chassis.inlineTelOp(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), 
		master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

		// function in intake class to spin intake based on controller button inputs
		intake.telOP(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1), master.get_digital(pros::E_CONTROLLER_DIGITAL_L1), 
		master.get_digital(pros::E_CONTROLLER_DIGITAL_R2), master.get_digital(pros::E_CONTROLLER_DIGITAL_X));

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            tongueToggle = !tongueToggle;
            tongue.set_value(tongueToggle);
        }
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
            doinkerToggle = !doinkerToggle;
            doinker.set_value(doinkerToggle);
        }
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            backdoinkerToggle = !backdoinkerToggle;
            backDoinker.set_value(backdoinkerToggle);
        }
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            doubleParkToggle = !doubleParkToggle;
            doublePark.set_value(doubleParkToggle);
        }
	}
}