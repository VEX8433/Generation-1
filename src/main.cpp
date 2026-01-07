#include "main.h"
#include "pros/rtos.hpp"
#include "SubSystems/Intake.hpp"
#include "Subsystems/auton_routes.hpp"
#include "lemlib/api.hpp"
#include "pros/abstract_motor.hpp"
#include <future>



pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::Motor left(1, pros::v5::MotorGears::blue);
pros::Motor right(-10, pros::v5::MotorGears::blue);
pros::Imu inertial(20);
pros::Distance distance(8);
pros::Rotation tracking(5);

pros::adi::DigitalOut bot('A');
pros::adi::DigitalOut top('B');
pros::adi::DigitalOut doinker('C');
pros::adi::DigitalOut tongue('D');
pros::adi::DigitalOut backDoinker('E');
pros::adi::DigitalOut doublePark('F');


pros::MotorGroup left_motors({-3, -4, 2}, pros::MotorGearset::blue); // left motors use 600 RPM cartridges
pros::MotorGroup right_motors({6, 7, -9}, pros::MotorGearset::blue); // right motors use 200 RPM cartridges

lemlib::Drivetrain drivetrain(
	&left_motors, // left motor group
	&right_motors, // right motor group
	10.5, // 10.5 inch track width
	lemlib::Omniwheel::NEW_4, // using new 4" omnis
	300, // drivetrain rpm is 300
	1 // horizontal drift is 0 (for now)
);

lemlib::TrackingWheel vertical_tracking_wheel(&tracking, lemlib::Omniwheel::NEW_2, -0.787402);

lemlib::OdomSensors sensors(
	&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
	nullptr, 
	nullptr, 
	nullptr,
	&inertial // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
											0, // integral gain (kI)
											24, // derivative gain (kD)
											3, // anti windup
											1, // small error range, in inches
											100, // small error range timeout, in milliseconds
											2, // large error range, in inches
											500, // large error range timeout, in milliseconds
											30 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(4, // proportional gain (kP)
											0, // integral gain (kI)
											25, // derivative gain (kD)
											3, // anti windup
											1, // small error range, in degrees
											100, // small error range timeout, in milliseconds
											3, // large error range, in degrees
											500, // large error range timeout, in milliseconds
											0 // maximum acceleration (slew)
);

lemlib::Chassis chassis(drivetrain, // drivetrain settings
					lateral_controller, // lateral PID settings
					angular_controller, // angular PID settings
					sensors // odometry sensors
);
Intake intake(left, right, bot, top, doublePark, distance);

void rightside(){
    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    // turn to face heading 90 with a very long timeout
	intake.telOP(true, false, false, false, false, false);  // intake
	//queue balls (intake)
	chassis.moveToPose(9, 26, 24, 2000);
	chassis.turnToHeading(114, 1000);
	chassis.moveToPoint(34.73, 6, 2000);
	chassis.turnToHeading(180,  2000);
	tongue.set_value(true);
	chassis.moveToPose(34.73,-20,180,1500, {.maxSpeed = 150});
	chassis.moveToPose(36.73, 25, 180,  2000, {.forwards = false});
	pros::delay(2000);
	intake.telOP(false, true, false, false, false, false);  // intake
}

void leftside(){
	chassis.setPose(0, 0, 0);

	intake.telOP(true, false, false, false, false, false);
	chassis.moveToPose( -8.6, 37, -21, 2000, {.minSpeed = 50}, false);
	pros::delay(300);
	chassis.turnToHeading(-131, 1000); // fix
	chassis.moveToPose(7, 44, -131, 1300,{.forwards=false});
	pros::delay(1300);
	intake.telOP(false, false, true, false, false, false);
	pros::delay(400);
	intake.telOP(true, false, false, false, false, false);
	pros::delay(200);
	chassis.moveToPoint(-34.73, 8, 2000);
	chassis.turnToHeading(180, 1000);
	tongue.set_value(true);
	chassis.moveToPoint(-36, -20, 1600, {.maxSpeed = 40});
	chassis.moveToPoint(-35.5, 30, 1000, {.forwards=false,.maxSpeed = 80}, false);
	intake.telOP(false, true, false, false, false, false);
	pros::delay(2000);
	tongue.set_value(false);
	chassis.moveToPoint(-35.5, 17, 1000, {.minSpeed = 60}, false);
	chassis.moveToPoint(-35.5, 40, 1000, {.forwards=false,.minSpeed = 200}, false);
}
void skills(){
	/* CODEGEN EXPORT: LemLib */

	chassis.setPose(-60.000000, -0.000000, 270.000000);

	chassis.moveToPoint(-72.0, -0.0, 1180);
	pros::delay(50);
	chassis.turnToHeading(90.0, 1508);
	chassis.moveToPoint(-24.0, -0.0, 2146);
	pros::delay(50);
	chassis.turnToHeading(180.0, 1139);
	chassis.turnToHeading(26.565051, 1402);
	chassis.moveToPoint(-18.0, 12.0, 1235, {.minSpeed = 63, .earlyExitRange = 4.67});
	pros::delay(50);
	chassis.moveToPoint(-24.0, 24.0, 1235);
	pros::delay(50);
	chassis.turnToHeading(135.0, 1435);
	chassis.moveToPoint(-12.0, 12.0, 1362);
	pros::delay(50);
	chassis.turnToHeading(315.0, 1508);
	chassis.moveToPoint(-48.0, 48.0, 2212);
	pros::delay(50);
	chassis.turnToHeading(270.0, 863);
	chassis.moveToPoint(-66.0, 48.0, 1397);
	pros::delay(50);
	chassis.turnToHeading(248.198591, 770);
	chassis.moveToPoint(-36.0, 60.0, 1806, {.forwards = false});
	pros::delay(50);
	chassis.turnToHeading(270.0, 770);
	chassis.moveToPoint(48.0, 60.0, 2973, {.forwards = false});
	pros::delay(50);
	chassis.turnToHeading(333.434949, 987);
	chassis.moveToPoint(54.0, 48.0, 1235, {.forwards = false});
	pros::delay(50);
	chassis.turnToHeading(90.0, 1273);
	chassis.moveToPoint(24.0, 48.0, 1746, {.forwards = false});
	pros::delay(50);
	chassis.turnToHeading(270.0, 1508);
	chassis.moveToPoint(66.0, 48.0, 1996, {.forwards = false});
	pros::delay(50);
	chassis.turnToHeading(212.005383, 953);
	chassis.moveToPoint(36.0, -0.0, 2343);
	pros::delay(50);
	chassis.turnToHeading(90.0, 1264);
	chassis.moveToPoint(72.0, -0.0, 1900);
	pros::delay(50);
	chassis.turnToHeading(252.645975, 1439);
	chassis.moveToPoint(-24.0, -30.0, 3354);

	// Estimated total time: 27.93 s

}

void autonomous() {
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
	// rightside();
	leftside();
	// skills();
}







/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });
}
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

void driveTelop(int leftY, int rightX){// 127, -127
    // https://www.desmos.com/calculator/lve1dfzzku
    double t = 5;
    float forward = 4.7244 * leftY;
    // float forward = (exp(-(t/10))+exp((abs(leftY)-127)/10)*(1- exp(-(t/10))))*leftY * 4.7244;
    // float turn = 3.5 * rightX;//4.7244 * rightX
    float turn = (exp(-(t/10))+exp((abs(rightX)-127)/10)*(1- exp(-(t/10))))*rightX * 4;

    left_motors.move_velocity(forward + turn);
    right_motors.move_velocity(forward - turn);
}

void opcontrol() {
	bool doinkerToggle = false;
	bool tongueToggle = false;
	bool backdoinkerToggle = false;
	bool doubleParkToggle = false;

	
	// pros::Task thing(intake.telOP(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1), master.get_digital(pros::E_CONTROLLER_DIGITAL_L1), master.get_digital(pros::E_CONTROLLER_DIGITAL_X), master.get_digital(pros::E_CONTROLLER_DIGITAL_R2), master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN), master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)));
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

	while(true){
		//Function in chassis class to move according to joystick inputs
		driveTelop(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), 
		master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

		intake.telOP(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1), master.get_digital(pros::E_CONTROLLER_DIGITAL_L1), 
		master.get_digital(pros::E_CONTROLLER_DIGITAL_X), master.get_digital(pros::E_CONTROLLER_DIGITAL_R2), master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN), master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT));
		
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
		pros::delay(20);
	}
}