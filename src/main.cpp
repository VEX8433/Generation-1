#include "main.h"
#include "pros/rtos.hpp"
#include "SubSystems/Intake.hpp"
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

Localizer localizer(chassis, &distFront, &distBack, &distLeft, nullptr);

// Forward declaration for calibration tests (defined in SubSystems/calib.cpp)
void run_calibration_tests();

Intake intake(left, right, bot, top, doublePark, distFront);

/**
 * @brief Reset robot position by moving to a target distance from wall
 * 
 * Automatically uses front or back sensor based on parameter.
 * Front sensor: too close → move back, too far → move forward
 * Back sensor: too close → move forward, too far → move back
 * Uses smooth acceleration to avoid jolting.
 * 
 * @param targetDist Target distance in mm (what the sensor should read)
 * @param useFront true = use front sensor, false = use back sensor
 * @param speed Motor velocity for adjustment (default 50)
 */
void resetToDistance(int targetDist, bool useFront, int speed = 50) {
    pros::delay(50);  // Allow sensor to stabilize, avoid prefetch error
    
    // Select sensor based on parameter
    pros::Distance& sensor = useFront ? distFront : distBack;
    
    int currentDist = sensor.get_distance();
    
    // Check for sensor error (returns 9999 or very high value on error)
    if (currentDist > 3000) {
        pros::lcd::print(4, "Sensor error! Dist=%dmm", currentDist);
        return;
    }
    
    pros::lcd::print(4, "%s Target=%dmm Curr=%dmm", useFront ? "FRONT" : "BACK", targetDist, currentDist);
    
    // Slew rate control - how fast to accelerate (velocity units per loop)
    const int SLEW_RATE = 5;  // Increase velocity by 5 each loop (smoother ramp up)
    int currentVel = 0;
    
    if (useFront) {
        // Front sensor: too far → forward, too close → backward
        if (currentDist > targetDist) {
            while (sensor.get_distance() > targetDist) {
                // Ramp up velocity smoothly
                if (currentVel < speed) currentVel += SLEW_RATE;
                if (currentVel > speed) currentVel = speed;
                left_motors.move_velocity(currentVel);
                right_motors.move_velocity(currentVel);
                pros::lcd::print(4, "FRONT Target=%dmm Curr=%dmm", targetDist, sensor.get_distance());
                pros::delay(20);
            }
        } else if (currentDist < targetDist) {
            while (sensor.get_distance() < targetDist) {
                // Ramp up velocity smoothly
                if (currentVel < speed) currentVel += SLEW_RATE;
                if (currentVel > speed) currentVel = speed;
                left_motors.move_velocity(-currentVel);
                right_motors.move_velocity(-currentVel);
                pros::lcd::print(4, "FRONT Target=%dmm Curr=%dmm", targetDist, sensor.get_distance());
                pros::delay(20);
            }
        }
    } else {
        // Back sensor: too far → backward, too close → forward
        if (currentDist > targetDist) {
            while (sensor.get_distance() > targetDist) {
                // Ramp up velocity smoothly
                if (currentVel < speed) currentVel += SLEW_RATE;
                if (currentVel > speed) currentVel = speed;
                left_motors.move_velocity(-currentVel);
                right_motors.move_velocity(-currentVel);
                pros::lcd::print(4, "BACK Target=%dmm Curr=%dmm", targetDist, sensor.get_distance());
                pros::delay(20);
            }
        } else if (currentDist < targetDist) {
            while (sensor.get_distance() < targetDist) {
                // Ramp up velocity smoothly
                if (currentVel < speed) currentVel += SLEW_RATE;
                if (currentVel > speed) currentVel = speed;
                left_motors.move_velocity(currentVel);
                right_motors.move_velocity(currentVel);
                pros::lcd::print(4, "BACK Target=%dmm Curr=%dmm", targetDist, sensor.get_distance());
                pros::delay(20);
            }
        }
    }
    
    // Stop motors
    left_motors.move_velocity(0);
    right_motors.move_velocity(0);
    pros::lcd::print(5, "DONE! Final=%dmm", sensor.get_distance());
}

/**
 * @brief Reset robot position using multiple distance sensors
 * 
 * Takes an array of sensor-target pairs and adjusts the robot position
 * until all sensors read their target distances (or timeout).
 * Use this when you want to align using multiple walls simultaneously.
 * 
 * @param sensor1 First distance sensor
 * @param target1 Target distance for first sensor (mm)
 * @param sensor2 Second distance sensor (optional, nullptr to skip)
 * @param target2 Target distance for second sensor (mm)
 * @param tolerance Acceptable error in mm (default 10mm)
 * @param speed Motor velocity for adjustment (default 50)
 * @param timeout Maximum time in ms (default 3000ms)
 * @return true if all targets reached, false if timeout
 */
bool resetToDistanceMulti(
    pros::Distance& sensor1, int target1,
    pros::Distance* sensor2 = nullptr, int target2 = 0,
    int tolerance = 10, int speed = 50, int timeout = 3000
) {
    int startTime = pros::millis();
    
    while (true) {
        // Check timeout
        if (pros::millis() - startTime > timeout) {
            left_motors.move_velocity(0);
            right_motors.move_velocity(0);
            pros::lcd::print(5, "Multi Reset TIMEOUT!");
            return false;
        }
        
        int dist1 = sensor1.get_distance();
        int dist2 = sensor2 ? sensor2->get_distance() : target2; // Use target if no sensor
        
        int error1 = dist1 - target1;
        int error2 = sensor2 ? (dist2 - target2) : 0;
        
        pros::lcd::print(4, "S1: %dmm (t=%d) S2: %dmm (t=%d)", dist1, target1, dist2, target2);
        
        // Check if both within tolerance
        if (abs(error1) <= tolerance && abs(error2) <= tolerance) {
            left_motors.move_velocity(0);
            right_motors.move_velocity(0);
            pros::lcd::print(5, "Multi Reset DONE!");
            return true;
        }
        
        // Simple approach: prioritize primary sensor, use secondary for differential
        int forward = 0;
        int turn = 0;
        
        if (abs(error1) > tolerance) {
            forward = (error1 > 0) ? speed : -speed;
        }
        
        if (sensor2 && abs(error2) > tolerance) {
            // Add slight turn correction based on secondary sensor
            turn = (error2 > 0) ? (speed / 3) : -(speed / 3);
        }
        
        left_motors.move_velocity(forward + turn);
        right_motors.move_velocity(forward - turn);
        
        pros::delay(10);
    }
}

/**
 * @brief Quick position reset using pose update based on distance sensor
 * 
 * Instead of moving the robot, this updates the odometry pose based on
 * the known wall position and sensor reading. Use after manually aligning.
 * 
 * @param sensor Distance sensor to read
 * @param wallCoord The known coordinate of the wall the sensor faces (e.g., -72 or 72 inches)
 * @param sensorOffsetMM Distance from robot center to sensor (mm)
 * @param isXAxis true if resetting X coordinate, false for Y
 * @param wallIsPositive true if wall is at positive coordinate (+72), false for negative (-72)
 */
void resetPoseFromSensor(pros::Distance& sensor, float wallCoord, float sensorOffsetMM, bool isXAxis, bool wallIsPositive) {
    float distInches = sensor.get_distance() / 25.4f; // Convert sensor reading mm to inches
    float offsetInches = sensorOffsetMM / 25.4f;      // Convert offset mm to inches
    float newCoord;
    
    pros::lcd::print(4, "Sensor: %dmm Offset: %.0fmm", sensor.get_distance(), sensorOffsetMM);
    
    if (wallIsPositive) {
        newCoord = wallCoord - distInches - offsetInches;
    } else {
        newCoord = wallCoord + distInches + offsetInches;
    }
    
    lemlib::Pose pose = chassis.getPose();
    if (isXAxis) {
        chassis.setPose(newCoord, pose.y, pose.theta);
        pros::lcd::print(6, "X Reset: %.1f -> %.1f", pose.x, newCoord);
    } else {
        chassis.setPose(pose.x, newCoord, pose.theta);
        pros::lcd::print(6, "Y Reset: %.1f -> %.1f", pose.y, newCoord);
    }
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

/**
 * @brief Test routine for distance sensor position reset
 * 
 * Moves robot toward a wall, resets position using distance sensor,
 * and displays before/after values on brain screen.
 */
void testDistanceReset() {
    // Start at known position
    chassis.setPose(0, 0, 180);  // Facing the back wall (Y = -72)
    
    pros::lcd::print(0, "=== DISTANCE RESET TEST ===");
    pros::lcd::print(1, "Starting at (0, 0, 180)");
    pros::delay(1000);
    
    // Move toward the back wall
    pros::lcd::print(2, "Moving toward back wall...");
    chassis.moveToPoint(0, -50, 3000);  // Move toward Y = -72 wall
    chassis.waitUntilDone();
    
    // Show position before reset
    lemlib::Pose beforePose = chassis.getPose();
    pros::lcd::print(2, "Before: X=%.1f Y=%.1f", beforePose.x, beforePose.y);
    pros::delay(500);
    
    // Use distance sensor to move to exact distance from wall (e.g., 300mm)
    pros::lcd::print(3, "Resetting to 300mm from wall...");
    resetToDistance(300, true);  // Move until front sensor reads 300mm
    
    // Now update odometry based on sensor reading
    // Front sensor facing back wall (Y = -72), wall is negative
    resetPoseFromSensor(distFront, -72.0f, OFFSET_FRONT, false, false);
    
    // Show position after reset
    lemlib::Pose afterPose = chassis.getPose();
    pros::lcd::print(4, "After: X=%.1f Y=%.1f", afterPose.x, afterPose.y);
    
    // Calculate expected Y position: wall(-72) + distance + offset
    float expectedY = -72.0f + (distFront.get_distance() / 25.4f) + (OFFSET_FRONT / 25.4f);
    pros::lcd::print(5, "Expected Y: %.1f", expectedY);
    pros::lcd::print(6, "Dist: %dmm Offset: %.0fmm", distFront.get_distance(), OFFSET_FRONT);
    
    pros::delay(3000);  // Show results for 3 seconds
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
	// chassis.setPose(0, 0, 0);

	chassis.setPose(-49.920000, 15.120000, 85.236000);

	

	chassis.turnToHeading(90.0, 503);
	chassis.moveToPoint(-36.24, 15.12, 1245);
	pros::delay(50);
	chassis.turnToHeading(52.815294, 805);
	chassis.moveToPoint(-22.32, 25.68, 1379);
	pros::delay(50);
	chassis.turnToHeading(312.633752, 1192);
	chassis.moveToPoint(-7.2, 11.76, 1478, {.forwards = false});
	pros::delay(1000);
	middle_goal_score(true);
	
	
	// pros::delay(1000);


	chassis.turnToHeading(318.868204, 536);
	chassis.moveToPoint(-38.64, 47.76, 2141); // node 4
	chassis.waitUntil(11.144602);
	middle_goal_score(false);
	chassis.waitUntilDone();
	pros::delay(200);
	chassis.turnToHeading(258.310631, 969);
	chassis.moveToPoint(-45.6, 46.32, 1012);
	chassis.waitUntil(0.481858);


	matchload_activate(true);
	chassis.waitUntilDone();
	pros::delay(300);
	chassis.turnToHeading(270, 646);
	chassis.moveToPoint(-74.32, 46.32, 3420, {.maxSpeed=50}, false); //node 6

	// pros::delay(2000);
	chassis.moveToPoint(-45.12, 46.32, 1436, {.forwards = false}); //node 7	
	chassis.waitUntil(14.629714);
	matchload_activate(false);


	intake.telOP(true, false, false, false, false, false);
	chassis.waitUntilDone();
	pros::delay(200);
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

	

	pros::delay(200);
	chassis.turnToHeading(177.207298, 1116);
	
	resetToDistance(240, false);



	pros::delay(200000000);




	chassis.moveToPoint(42.0, 53, 1573); // node 14
	pros::delay(50);
	chassis.turnToHeading(90.855097, 1120);
	
	chassis.moveToPoint(23, 53, 1000, {.forwards = false}, false); //node 15
	long_goal_score(true);
	pros::delay(2500); 
	chassis.waitUntil(11.940754);


	//initial match, top left score
	
	chassis.turnToHeading(90, 461); //prev89
	chassis.moveToPoint(47.76, 53, 1518); //prey56 //node 16?? ///current 48
	chassis.waitUntil(16.507699);


	//top left matchload
	matchload_activate(true);
	chassis.waitUntilDone();
	pros::delay(500);
	chassis.turnToHeading(85, 461);
	chassis.moveToPoint(65.52, 54, 3389, {.maxSpeed = 50}); //prev 5 //node 17
	// pros::delay(2000);
	chassis.turnToHeading(90.0, 461);
	chassis.moveToPoint(23, 52.5, 1500, {.forwards = false}, false); //node 18 //scoring on long goal
	// chassis.waitUntil(32.81013);
//__________above no change is good

	//top left 2nd matchload score		
	matchload_activate(false);

	long_goal_score(true);
	pros::delay(2500);
	chassis.moveToPoint(45, 53, 1539); //node 19

	chassis.turnToHeading(180, 1204);
	// LOCALIZATION RESET: Near left wall (X = -72), reset X coordinate
	// Robot is at the wall, heading 270 (facing left), so left sensor points at wall
	// localizer.resetXWithHeading(-72.0f);)
	
	// chassis.moveToPoint(30.48, -26.16, 2975); //node 20
	// pros::delay(50);
	// chassis.turnToHeading(144.833564, 883);
	chassis.moveToPoint(45, -49, 2000000, {.maxSpeed = 80}, false); // ode 21

	pros::delay(200);





































	
	// Position reset using front distance sensor
	resetToDistance(530, true);  // true = front sensor
	chassis.turnToHeading(175, 1000);
	// Update odometry Y based on known wall position (using front sensor offset)
	resetPoseFromSensor(distFront, -72.0f, OFFSET_FRONT, false, false);

	// localizer.resetXWithHeading(72)
	// chassis.waitUntil(21.38118);


	//top right matchload
	// chassis.waitUntilDone();
	// pros::delay(500);
	
	chassis.turnToHeading(90, 941, {}, false);
	

	long_goal_score(false);
	matchload_activate(true);
	pros::delay(400);

	chassis.moveToPoint(65, -53, 1397, {}, false); //node 22

	chassis.waitUntilDone();













// 	//reset code HERE. node 6 is = node 22.
// 	chassis.setPose(resetPosX, resetPosY, -90);
	
// 	pros::delay(2000);

// 	chassis.moveToPoint(-45.12, 46.32, 1436, {.forwards = false}); //node 7	
// 	chassis.waitUntil(14.629714);
// 	matchload_activate(false);


// 	intake.telOP(true, false, false, false, false, false);
// 	chassis.waitUntilDone();
// 	pros::delay(50);
// 	chassis.turnToHeading(24.034288, 1258);
// 	chassis.moveToPoint(-37.2, 64.08, 1443); //node 8
// 	chassis.turnToHeading(91.487868, 465);
// 	chassis.moveToPoint(41.52, 70.52, 1413); //node 13
// 	pros::delay(50);
// 	chassis.turnToHeading(177.207298, 1116);
// 	chassis.moveToPoint(42.0, 53, 1573); // node 14
// 	pros::delay(50);
// 	chassis.turnToHeading(90.855097, 1120);
	
// 	chassis.moveToPoint(23, 53, 1000, {.forwards = false}, false); //node 15
// 	long_goal_score(true);
// 	pros::delay(2500); 
// 	chassis.waitUntil(11.940754);


// 	//initial match, top left score
	
// 	chassis.turnToHeading(90, 461); //prev89
// 	chassis.moveToPoint(47.76, 53, 1518); //prey56 //node 16?? ///current 48
// 	chassis.waitUntil(16.507699);


// 	//top left matchload
// 	matchload_activate(true);
// 	chassis.waitUntilDone();
// 	pros::delay(500);
// 	chassis.turnToHeading(85, 461);
// 	chassis.moveToPoint(65.52, 54, 3389, {.maxSpeed = 50}); //prev 5 //node 17
// 	// pros::delay(2000);
// 	chassis.turnToHeading(90.0, 461);
// 	chassis.moveToPoint(23, 52.5, 1500, {.forwards = false}, false); //node 18 //scoring on long goal
// 	// chassis.waitUntil(32.81013);
// //__________above no change is good

// 	//top left 2nd matchload score		
// 	matchload_activate(false);

// 	long_goal_score(true);
// 	pros::delay(2500);
// 	chassis.moveToPoint(45, 53, 1539); //node 19

























	pros::delay(2000);
	chassis.turnToHeading(90, 461);
	// chassis.moveToPoint(44.88, -53, 3727, {.forwards = false}); //node 23
	chassis.moveToPoint(45, -49, 2000000, {.maxSpeed = 80}, false); // ode 21 NEW NODE 23
	chassis.waitUntil(13.722215);

	
	//top right finished matchload
	matchload_activate(false);
	chassis.waitUntilDone();
	pros::delay(50);
	chassis.turnToHeading(222.909841, 1317);
	chassis.moveToPoint(33, -73.08, 1561);//node 24
	pros::delay(50);
	chassis.turnToHeading(270, 884);
	chassis.moveToPoint(-45.36, -73.08, 2753);//node 25
	pros::delay(50);

	//bottom right section
	chassis.turnToHeading(354.897835, 1108);
	chassis.moveToPoint(-46.56, -47, 1237); //node 26
	pros::delay(50);
	chassis.turnToHeading(270.0, 1112);
	chassis.moveToPoint(-23, -48, 1500, {.forwards = false}); //node 27

	//bottom right score
	long_goal_score(true);
	pros::delay(2500);
	matchload_activate(true);
	pros::delay(50);
	chassis.moveToPoint(-61.44, -47, 3870, {.maxSpeed=50}); //nodem28 //bottom right matchload

	chassis.waitUntilDone();
	pros::delay(50);
	chassis.moveToPoint(-51.84, -47, 1941, {.forwards = false});//node 29
	pros::delay(500);
	chassis.moveToPoint(-24.96, -47, 1663, {.forwards = false}); //node 30
	chassis.waitUntil(20.017067);

	//bottom right matchload score
	matchload_activate(false);
	long_goal_score(true);
	pros::delay(2500);
	chassis.waitUntilDone();
	pros::delay(50);
	chassis.turnToHeading(268.898294, 461);
	chassis.moveToPoint(-37.44, -49, 1199); //node 31
	pros::delay(50);
	chassis.turnToHeading(319.820766, 906);
	chassis.moveToPoint(-64.8, -17.28, 2006); //node 32
	pros::delay(50);
	chassis.turnToHeading(0.806929, 834);
	chassis.moveToPoint(-64.56, -0.24, 1364); //node 33

	// Estimated total time: 63.92 s


}

// void skills(){
// 	chassis.setPose(0, 0, 0);

// 	intake.telOP(true, false, false, false, false, false);
// 	chassis.moveToPose( -15, 34, -21, 2000, {.minSpeed = 50}, false);
// 	pros::delay(300);
// 	chassis.turnToHeading(-131, 1000); // fix
// 	chassis.moveToPose(7, 44, -131, 1600,{.forwards=false}, false);
// 	intake.telOP(false, false, true, false, false, false);
// 	pros::delay(400);
// 	intake.telOP(true, false, false, false, false, false);
// 	pros::delay(200);
// 	chassis.moveToPoint(-34, 8, 2000);
// 	chassis.turnToHeading(180, 1000);
// 	tongue.set_value(true);
// 	chassis.moveToPoint(-35, -20, 1600, {.maxSpeed = 40});
// 	chassis.moveToPoint(-35.5, 30, 1000, {.forwards=false,.maxSpeed = 80}, false);
// 	intake.telOP(false, true, false, false, false, false);
// 	pros::delay(2000);
// 	tongue.set_value(false);
// 	chassis.moveToPoint(-35.5, 15, 1000, {.minSpeed = 60}, false);

// 	chassis.turnToHeading(-50, 1000);
// 	chassis.moveToPoint(-50, 30, 3000);
// 	chassis.turnToHeading(0, 1000);
// 	chassis.moveToPoint(-50, 95, 2000);
// 	chassis.turnToHeading(90, 1000);
// 	chassis.moveToPoint(-35.5, 95, 1000, {}, false);
// 	chassis.turnToHeading(0, 0, {}, false);
// 	tongue.set_value(true);
// 	intake.telOP(true, false, false, false, false, false);
// 	chassis.moveToPoint(-35.5, 120, 4000, {.maxSpeed = 50}, false);

// 	chassis.moveToPoint(-35.5, 60, 1000, {.forwards = false}, false);
// 	intake.telOP(false, true, false, false, false, false);
// }

void autonomous() {
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
	// rightside();
	// // leftside();
	// skills();
	// // soloAWP();
	chassis.setPose(0, 0, 0);

	resetToDistance(200, false, 20);  // true = front sensor
	// run_calibration_tests();




	// chassis.setPose(-48.000000, 12.000000, 90);
	// chassis.moveToPoint(-24.0, 24.0, 1662);
	// pros::delay(50);
	// chassis.turnToHeading(323.130102, 1194);
	// chassis.moveToPoint(-42.0, 48.0, 1746);
	// pros::delay(50);
	// chassis.turnToHeading(270.0, 920);
	// chassis.moveToPoint(-54.0, 48.0, 1180);
	
	// // Reset position using right distance sensor
	// // Robot center should be 400mm from wall (sensor reads 400 + 170 = 570mm)
	// resetToDistance(distRight, 470, OFFSET_RIGHT);
	// chassis.setPose(-54.0, 48.0, 1180);
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
