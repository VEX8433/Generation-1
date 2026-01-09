#include "main.h"
#include "pros/rtos.hpp"
#include "SubSystems/Intake.hpp"
#include "SubSystems/auton_routes.hpp"
#include "lemlib/api.hpp"
#include "pros/abstract_motor.hpp"
// #include <future>



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

// Forward declaration for calibration tests (defined in SubSystems/calib.cpp)
void run_calibration_tests();

Intake intake(left, right, bot, top, doublePark, distance);

void long_goal_score(bool active){
	if (active){
		intake.telOP(false, true, false, false, false, false);
	} else {
		intake.telOP(false, false, false, false, false, false);
	}

}

void middle_goal_score(bool active){
	if (active){
		intake.telOP(false, false, true, false, false, false);
	} else {
		intake.telOP(false, false, false, false, false, false);
	}

}

void  matchload_activate(bool active){
	if (active){
		tongue.set_value(true);
	intake.telOP(true, false, false, false, false, false);
	} else {
		tongue.set_value(false);
	intake.telOP(false, false, false, false, false, false);
	}

}

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
	intake.telOP(true, false, false, false, false, false);

		/* CODEGEN EXPORT: LemLib */

	chassis.setPose(-49.920000, 15.120000, 85.236000);

	chassis.turnToHeading(90.0, 503);
	chassis.moveToPoint(-36.24, 15.12, 1245);
	pros::delay(50);
	chassis.turnToHeading(52.815294, 805);
	chassis.moveToPoint(-22.32, 25.68, 1379);
	pros::delay(50);
	chassis.turnToHeading(312.633752, 1192);
	chassis.moveToPoint(-7.2, 11.76, 1478, {.forwards = false});
	chassis.waitUntil(18.35462);
	middle_goal_score(true);
	chassis.waitUntilDone();
	
	
	// pros::delay(1000);


	chassis.turnToHeading(318.868204, 536);
	chassis.moveToPoint(-38.64, 47.76, 2141);
	chassis.waitUntil(11.144602);
	middle_goal_score(false);
	chassis.waitUntilDone();
	pros::delay(50);
	chassis.turnToHeading(258.310631, 969);
	chassis.moveToPoint(-45.6, 46.32, 1012);
	chassis.waitUntil(0.481858);


	matchload_activate(true);
	chassis.waitUntilDone();
	pros::delay(300);
	chassis.turnToHeading(270, 646);
	chassis.moveToPoint(-74.32, 46.32, 3420, {.maxSpeed=50}); //node 6
	// pros::delay(2000);
	chassis.moveToPoint(-45.12, 46.32, 1436, {.forwards = false}); //node 7	
	chassis.waitUntil(14.629714);
	matchload_activate(false);


	intake.telOP(true, false, false, false, false, false);
	chassis.waitUntilDone();
	pros::delay(50);
	chassis.turnToHeading(24.034288, 1258);
	chassis.moveToPoint(-37.2, 64.08, 1443); //node 8
	// pros::delay(50);
	// chassis.turnToHeading(88.174458, 991);
	// chassis.moveToPoint(-19.922253, 64.630685, 1373);
	// pros::delay(50);
	// chassis.moveToPoint(-3.843288, 65.143162, 1332);
	// pros::delay(50);
	// chassis.moveToPoint(10.100794, 65.587595, 1255);
	// pros::delay(50);
	// chassis.moveToPoint(23.04, 66.0, 1217);
	// pros::delay(50);
	chassis.turnToHeading(91.487868, 465);
	chassis.moveToPoint(41.52, 70.52, 1413); //node 13
	pros::delay(50);
	chassis.turnToHeading(177.207298, 1116);
	chassis.moveToPoint(42.0, 52.5, 1573); // node 14
	pros::delay(50);
	chassis.turnToHeading(90.855097, 1120);
	
	chassis.moveToPoint(25.92, 52.5, 1332, {.forwards = false}); //node 15
	long_goal_score(true);
	chassis.waitUntilDone();
	pros::delay(2500);
	chassis.waitUntil(11.940754);


	//initial match, top left score
	
	chassis.turnToHeading(90, 461); //prev89
	chassis.moveToPoint(47.76, 52.5, 1518); //prey56 //node 15?? ///current 48
	chassis.waitUntil(16.507699);


	//top left matchload
	matchload_activate(true);
	chassis.waitUntilDone();
	pros::delay(500);
	chassis.turnToHeading(90, 461);
	chassis.moveToPoint(65.52, 54, 3389, {.maxSpeed = 50}); //prev 5 //node 17
	// pros::delay(2000);
	chassis.turnToHeading(90.0, 461);
	chassis.moveToPoint(25.92, 52.5, 4912, {.forwards = false}); //node 18 //scoring on long goal
	// chassis.waitUntil(32.81013);


	//top left 2nd matchload score
	matchload_activate(false);
	chassis.waitUntilDone();

	long_goal_score(true);
	pros::delay(2500);
	chassis.moveToPoint(48.72, 52.5, 1539);

	chassis.turnToHeading(192.528808, 1204);
	chassis.moveToPoint(30.48, -26.16, 2975);
	pros::delay(50);
	chassis.turnToHeading(144.833564, 883);
	chassis.moveToPoint(45.36, -47.28, 1634);
	chassis.waitUntil(21.38118);


	//top right matchload
	long_goal_score(false);
	matchload_activate(true);
	chassis.waitUntilDone();
	pros::delay(500);
	chassis.turnToHeading(89.472475, 941);
	chassis.moveToPoint(63.36, -45.8, 1397);
	pros::delay(3000);
	chassis.turnToHeading(89.255941, 461);
	chassis.moveToPoint(44.88, -47.04, 3727, {.forwards = false}); //node 23
	chassis.waitUntil(13.722215);

	
	//top right finish matchload
	matchload_activate(false);
	chassis.waitUntilDone();
	pros::delay(50);
	chassis.turnToHeading(222.909841, 1317);
	chassis.moveToPoint(29.04, -64.08, 1561);//node 24
	pros::delay(50);
	chassis.turnToHeading(270.739259, 884);
	chassis.moveToPoint(-45.36, -63.12, 2753);//node 35
	pros::delay(50);

	//bottom right section
	chassis.turnToHeading(354.897835, 1108);
	chassis.moveToPoint(-46.56, -49.68, 1237); //node 26
	pros::delay(50);
	chassis.turnToHeading(270.0, 1112);
	chassis.moveToPoint(-24.48, -49.68, 1525, {.forwards = false}); //node 27
	chassis.waitUntil(13.893972);

	//bottom right score
	long_goal_score(true);
	pros::delay(2500);
	chassis.waitUntilDone();
	matchload_activate(true);
	pros::delay(50);
	chassis.moveToPoint(-61.44, -49.44, 3870, {.maxSpeed=50}); //nodem28 //bottom right matchload

	
	chassis.waitUntilDone();
	pros::delay(50);
	chassis.moveToPoint(-51.84, -49.44, 1941, {.forwards = false});//node 29
	pros::delay(500);
	chassis.moveToPoint(-24.96, -49.44, 1663, {.forwards = false}); //node 30
	chassis.waitUntil(20.017067);

	//bottom right matchload score
	matchload_activate(false);
	long_goal_score(true);
	pros::delay(2500);
	chassis.waitUntilDone();
	pros::delay(50);
	chassis.turnToHeading(268.898294, 461);
	chassis.moveToPoint(-37.44, -49.68, 1199); //node 31
	pros::delay(50);
	chassis.turnToHeading(319.820766, 906);
	chassis.moveToPoint(-64.8, -17.28, 2006); //node 32
	pros::delay(50);
	chassis.turnToHeading(0.806929, 834);
	chassis.moveToPoint(-64.56, -0.24, 1364); //node 33

	// Estimated total time: 63.92 s


}

void autonomous() {
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
	// rightside();
	// leftside();
	skills();
	// run_calibration_tests();
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
