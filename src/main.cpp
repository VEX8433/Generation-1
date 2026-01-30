#include "main.h"
#include "pros/rtos.hpp"
#include "SubSystems/Intake.hpp"
#include "SubSystems/AutonSelector.hpp"
#include "lemlib/api.hpp"
#include "pros/abstract_motor.hpp"
#include "SubSystems/Localizer.hpp"
#include <cstddef>

// #include <future>



pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::Motor left(1, pros::v5::MotorGears::blue);
pros::Motor right(-10, pros::v5::MotorGears::blue);
pros::Imu inertial(20);


pros::Rotation tracking(5);

// TOF Distance Sensors
pros::Distance distFront(8);   // Front distance sensor
pros::Distance distLeft(11);   // Left distance sensor
pros::Distance distBack(19);   // Back distance sensor

// Distance sensor offsets from robot center (mm)
// Positive values = sensor is that many mm away from center in its direction
constexpr float OFFSET_FRONT = 0.0f;  // Front sensor offset (mm from center)
constexpr float OFFSET_LEFT = 0.0f;   // Left sensor offset (mm from center)
constexpr float OFFSET_BACK = 0.0f;   // Back sensor offset (mm from center)

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
											10 // maximum acceleration (slew)
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

Localizer localizer(chassis, &distFront, &distBack, &distLeft, nullptr);



Intake intake(left, right, bot, top, doublePark, distFront);

/**
 * @brief Reset robot position by moving to a target distance from wall
 * Slows down as it approaches target for accuracy.
 * 
 * @param targetDist Target distance in mm (what the sensor should read)
 * @param useFront true = use front sensor, false = use back sensor
 * @param speed Max motor velocity for adjustment (default 50)
 */
void resetToDistance(int targetDist, bool useFront, int speed = 50) {
    pros::delay(50);  // Allow sensor to stabilize
    
    pros::Distance& sensor = useFront ? distFront : distBack;
    int currentDist = sensor.get_distance();
    
    if (currentDist > 3000) {
        pros::lcd::print(4, "Sensor error! Dist=%dmm", currentDist);
        return;
    }
    
    pros::lcd::print(4, "%s Target=%dmm Curr=%dmm", useFront ? "FRONT" : "BACK", targetDist, currentDist);
    
    const int MIN_SPEED = 10;  // Minimum speed to prevent stalling
    const int SLOW_RANGE = 100;  // Start slowing down within 100mm of target
    
    if (useFront) {
        if (currentDist > targetDist) {
            while (sensor.get_distance() > targetDist) {
                int dist = sensor.get_distance();
                int error = dist - targetDist;
                int moveSpeed = (error < SLOW_RANGE) ? (MIN_SPEED + (speed - MIN_SPEED) * error / SLOW_RANGE) : speed;
                left_motors.move_velocity(moveSpeed);
                right_motors.move_velocity(moveSpeed);
                pros::delay(10);
            }
        } else if (currentDist < targetDist) {
            while (sensor.get_distance() < targetDist) {
                int dist = sensor.get_distance();
                int error = targetDist - dist;
                int moveSpeed = (error < SLOW_RANGE) ? (MIN_SPEED + (speed - MIN_SPEED) * error / SLOW_RANGE) : speed;
                left_motors.move_velocity(-moveSpeed);
                right_motors.move_velocity(-moveSpeed);
                pros::delay(10);
            }
        }
    } else {
        if (currentDist > targetDist) {
            while (sensor.get_distance() > targetDist) {
                int dist = sensor.get_distance();
                int error = dist - targetDist;
                int moveSpeed = (error < SLOW_RANGE) ? (MIN_SPEED + (speed - MIN_SPEED) * error / SLOW_RANGE) : speed;
                left_motors.move_velocity(-moveSpeed);
                right_motors.move_velocity(-moveSpeed);
                pros::delay(10);
            }
        } else if (currentDist < targetDist) {
            while (sensor.get_distance() < targetDist) {
                int dist = sensor.get_distance();
                int error = targetDist - dist;
                int moveSpeed = (error < SLOW_RANGE) ? (MIN_SPEED + (speed - MIN_SPEED) * error / SLOW_RANGE) : speed;
                left_motors.move_velocity(moveSpeed);
                right_motors.move_velocity(moveSpeed);
                pros::delay(10);
            }
        }
    }
    
    left_motors.move_velocity(0);
    right_motors.move_velocity(0);
    pros::lcd::print(5, "DONE! Final=%dmm", sensor.get_distance());
}





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
	chassis.moveToPose(12, 37, 24, 2000, {.maxSpeed = 60});
	chassis.turnToHeading(114, 1000);
	chassis.moveToPoint(35, 6, 2000);
	chassis.turnToHeading(180,  2000);
	tongue.set_value(true);
	chassis.moveToPose(35,-20,180,1500, {.maxSpeed = 150});
	chassis.moveToPose(36.73, 30, 180,  1500, {.forwards = false, .minSpeed = 60}, false);
	intake.telOP(false, true, false, false, false, false);  // intake
	pros::delay(3000);
	tongue.set_value(false);
	chassis.moveToPoint(36.73, 17, 1000, {.minSpeed = 60}, false);
	chassis.moveToPoint(36.73, 40, 1000, {.forwards=false,.minSpeed = 200}, false);
}

void leftside(){
	chassis.setPose(0, 0, 0);

	intake.telOP(true, false, false, false, false, false);
	chassis.moveToPose( -12, 37, -21, 2000, {.minSpeed = 50}, false);
	pros::delay(300);
	chassis.turnToHeading(-131, 1000); // fix
	chassis.moveToPose(7, 44, -131, 1600,{.forwards=false}, false);
	intake.telOP(false, false, true, false, false, false);
	pros::delay(400);
	intake.telOP(true, false, false, false, false, false);
	pros::delay(200);
	chassis.moveToPoint(-34, 8, 2000);
	chassis.turnToHeading(180, 1000);
	tongue.set_value(true);
	chassis.moveToPoint(-34, -20, 1700, {.maxSpeed = 40});
	chassis.moveToPoint(-34, 30, 1000, {.forwards=false,.maxSpeed = 80}, false);
	intake.telOP(false, true, false, false, false, false);
	pros::delay(2000);
	tongue.set_value(false);
	chassis.moveToPoint(-36, 17, 1000, {.minSpeed = 60}, false);
	chassis.moveToPoint(-36, 40, 1000, {.forwards=false,.minSpeed = 200}, false);
}

void soloAWP(){
	// set position to x:0, y:0, heading:0
    chassis.setPose(-1, -1.5, -90);
   
	intake.telOP(true, false, false, false, false, false);

	chassis.moveToPoint(-34, 5, 1000);
	chassis.turnToHeading(180, 1000);
	tongue.set_value(true);
	chassis.moveToPoint(-34, -20, 1300, {.maxSpeed = 50, .minSpeed = 50});
	chassis.moveToPoint(-34, 30, 1000, {.forwards=false,.minSpeed = 80});
	pros::delay(800);
	intake.telOP(false, true, false, false, false, false);
	pros::delay(1500);
	tongue.set_value(false);

	chassis.moveToPoint(-34, 17, 1000, {.minSpeed = 60}, false);

	chassis.turnToHeading(90, 700, {}, false);
	intake.telOP(true, false, false, false, false, false);
	chassis.moveToPose( -15.5, 34, -21, 2000, {.minSpeed = 70}, false);
	pros::delay(300);
	chassis.turnToHeading(-131, 700); 
	chassis.moveToPose(10, 47, -131, 1000,{.forwards=false, .minSpeed = 100}, false);
	intake.telOP(false, false, true, false, false, false);
	pros::delay(700);

	intake.telOP(true, false, false, false, false, false);
	chassis.moveToPose(52, 34, 90, 1000, {.minSpeed = 80}, false);

	chassis.moveToPoint(64, 10, 1000, {.minSpeed = 70});
	chassis.turnToHeading(180, 1000);

	chassis.moveToPoint(64, 30, 1000, {.forwards = false, .minSpeed = 80});
	pros::delay(700);
	intake.telOP(false, true, false, false, false, false);
}

void skills(){
	intake.telOP(true, false, false, false, false, false);

	chassis.setPose(-49.920000, 15.120000, 85.236000);

	
	chassis.moveToPoint(-36.24, 15.12, 1245);
	chassis.turnToHeading(0, 1000);
	
	chassis.turnToHeading(0, 1000);
	
	chassis.moveToPoint(-45.6, 47, 1312, {}, false);
	pros::delay(200);
	chassis.turnToHeading(260, 1000, {}, false);


	matchload_activate(true);
	chassis.moveToPoint(-74.32, 45.5, 1000, {.maxSpeed=50}, false); //node 6
	pros::delay(300);
	chassis.moveToPoint(-74.32, 46.5, 1000, {.maxSpeed=20}, false); //node 6

	// pros::delay(2000);
	chassis.moveToPoint(-45.12, 47, 1436, {.forwards = false}); //node 7	
	chassis.waitUntil(14.629714);
	matchload_activate(false);


	intake.telOP(true, false, false, false, false, false);
	chassis.waitUntilDone();
	pros::delay(200);
	chassis.turnToHeading(24.034288, 1258);
	chassis.moveToPoint(-37.2, 64.08, 1443); //node 8

	chassis.turnToHeading(91.487868, 465);
	chassis.moveToPoint(42, 70.52, 2000, {.maxSpeed=100}); //node 13

	

	pros::delay(2000);
	chassis.turnToHeading(173, 1116, {.maxSpeed=100});
	pros::delay(1000);
	resetToDistance(440, false, 100);



	// chassis.moveToPoint(42.0, 53, 1573); // node 14
	pros::delay(1000);
	chassis.setPose(42, 53, chassis.getPose().theta+4.77);
	chassis.setPose(42, 53, chassis.getPose().theta+4.77);





	pros::delay(50);
	chassis.turnToHeading(90, 1120);






	chassis.moveToPoint(23, 53, 1000, {.forwards = false, .maxSpeed=100}, false); //node 15
	long_goal_score(true);
	pros::delay(2500); 
	chassis.waitUntil(11.940754);


	// initial match, top left score
	
	chassis.turnToHeading(90, 461, {}, false); //prev89
	matchload_activate(true);
	chassis.moveToPoint(47.76, 53, 1518, {.maxSpeed=100}, false); //prey56 //node 16?? ///current 48
	chassis.turnToHeading(93, 500);

	//top left matchload
	chassis.moveToPoint(65.52, 52, 1000, {.maxSpeed = 50}, false); //prev 5 //node 17
	pros::delay(300);
	chassis.moveToPoint(65.52, 52, 1700, {.maxSpeed = 20}, false); //prev 5 //node 17

	chassis.turnToHeading(90.0, 461);
	chassis.moveToPoint(23, 53, 1500, {.forwards = false, .maxSpeed=100}, false); //node 18 //scoring on long goal
	chassis.waitUntil(32.81013);



	chassis.setPose(23, 53, chassis.getPose().theta);


	//top left 2nd matchload score		
	matchload_activate(false);

	long_goal_score(true);



	pros::delay(2500);
	chassis.moveToPoint(37, 53, 1539, {.maxSpeed=110}); //node 19

	chassis.turnToHeading(180, 1204);
	// LOCALIZATION RESET: Near left wall (X = -72), reset X coordinate
	// Robot is at the wall, heading 270 (facing left), so left sensor points at wall
	// localizer.resetXWithHeading(-72.0f);)
	
	// chassis.moveToPoint(30.48, -26.16, 2975); //node 20
	// pros::delay(50);
	// chassis.turnToHeading(144.833564, 883);
	chassis.moveToPoint(37, -42, 20000, {.maxSpeed = 100}, false); // ode 21
	chassis.moveToPoint(37, -42, 20000, {.maxSpeed = 100}, false); // ode 21

	pros::delay(200);





	
	// Position reset using front distance sensor
	resetToDistance(540, true);  // true = front sensor
	chassis.setPose(40, -45, 180);
	// chassis.turnToHeading(175, 1000);
	// Update odometry Y based on known wall position (using front sensor offset)
	// resetPoseFromSensor(distFront, -72.0f, OFFSET_FRONT, false, false);

	// localizer.resetXWithHeading(72)
	// chassis.waitUntil(21.38118);


	//top right matchload
	// chassis.waitUntilDone();
	// pros::delay(500);
	
	chassis.turnToHeading(90, 941, {}, false);
	

	long_goal_score(false);
	matchload_activate(true);
	pros::delay(400);

	chassis.moveToPoint(65, -45, 1300, {.maxSpeed=50}, false); //node 22 // THIS IS THE MATCHLOAD POSITION
	pros::delay(300);
	chassis.moveToPoint(65, -45, 1700, {.maxSpeed=20}, false); 

	// chassis.waitUntilDone();
	// chassis.turnToHeading(90, 461);
	// chassis.moveToPoint(44.88, -53, 3727, {.forwards = false}); //node 23
	chassis.moveToPoint(40, -45, 2000, {.forwards = false, .maxSpeed = 80}); // ode 21 NEW NODE 23
	// chassis.waitUntil(13.722215);


	
	//top right finished matchload
	matchload_activate(false);
	chassis.waitUntilDone();
	pros::delay(50);
	chassis.turnToHeading(180, 1317);
	chassis.moveToPoint(33, -60, 1561);//node 24
	pros::delay(50);
	chassis.turnToHeading(270, 884);
	chassis.moveToPoint(-46, -60, 3500);//node 25
	chassis.moveToPoint(-46, -60, 3500);//node 25
	chassis.turnToHeading(0, 461);
	pros::delay(1000);
	chassis.moveToPoint(-46, -44, 1237); //node 26
	
	chassis.moveToPoint(-46, -44, 1237); //node 26
	

	pros::delay(1000);
	resetToDistance(500, false, 100);
	pros::delay(1000);
	chassis.setPose(-45, -47, 0);





	chassis.turnToHeading(270.0, 1112);
	chassis.moveToPoint(-23, -47, 1500, {.forwards = false}); //node 27

	//bottom right score
	long_goal_score(true);
	pros::delay(2500);
	// chassis.setPose(chassis.getPose().x, chassis.getPose().y, 270);


	// chassis.setPose(chassis.getPose().x, chassis.getPose().y, 270);


	matchload_activate(true);
	pros::delay(50);
	chassis.moveToPoint(-68.44, -47, 2000, {.maxSpeed=50}, false); //nodem28 //bottom right matchload
	pros::delay(3000);
	pros::delay(3000);

	chassis.moveToPoint(-51.84, -47, 1941, {.forwards = false});//node 29
	pros::delay(500);
	chassis.moveToPoint(-24.96, -47, 1663, {.forwards = false}); //node 30
	chassis.waitUntil(20.017067);
	matchload_activate(false);
	long_goal_score(true);
	pros::delay(2500);
	chassis.waitUntilDone();
	pros::delay(50);
	chassis.turnToHeading(268.898294, 461);
	chassis.moveToPoint(-37.44, -49, 1199); //node 31
	pros::delay(50);
	chassis.turnToHeading(320, 906);
	chassis.turnToHeading(320, 906);
	chassis.moveToPoint(-64.8, -17.28, 2006); //node 32
	pros::delay(50);
	chassis.turnToHeading(0, 834);
	chassis.turnToHeading(0, 834);
	chassis.moveToPoint(-64.56, -0.24, 1364); //node 33
}


void autonomous() {
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
	
	// Run the selected autonomous route
	switch (AutonSelector::getSelectedRoute()) {
		case AutonRoute::SKILLS:
			skills();
			break;
		case AutonRoute::RIGHT_SIDE:
			rightside();
			break;
		case AutonRoute::LEFT_SIDE:
			leftside();
			break;
		case AutonRoute::SOLO_AWP:
			soloAWP();
			break;
		case AutonRoute::DO_NOTHING:
			// Do nothing - robot stays still
			break;
	}
}







/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    chassis.calibrate(); // calibrate sensors
    
    // Initialize autonomous route selector UI
    AutonSelector::init();
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
void competition_initialize() {
    // Autonomous selector is already displayed from initialize()
    // Selection remains visible until autonomous or opcontrol starts
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
	// Clean up selector UI when entering driver control
	AutonSelector::destroy();
	
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
