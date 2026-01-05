#include "Chassis.hpp"
#include <cmath>

using namespace std;

Chassis::Chassis(pros::Motor FrontLeft, pros::Motor FrontRight, pros::Motor MiddleLeft, pros::Motor MiddleRight, pros::Motor BackLeft, pros::Motor BackRight, pros::Imu inertial, pros::Rotation tracking) : 
	frontLeft(FrontLeft), frontRight(FrontRight), middleLeft(MiddleLeft), middleRight(MiddleRight), backLeft(BackLeft), backRight(BackRight), inertial(inertial), tracking(tracking){ 
}

// sets all the motor brake modes to brake, so our robot will stop immediately when we tell it to stop
void Chassis::brake(){
    frontRight.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	frontLeft.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    middleLeft.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    middleRight.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	backLeft.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	backRight.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}

// Gets the values from the joystick to tell the motors how fast to move
void Chassis::inlineTelOp(int leftY, int rightX){// 127, -127
    // https://www.desmos.com/calculator/lve1dfzzku
    double t = 5;
    float forward = 4.7244 * leftY;
    // float forward = (exp(-(t/10))+exp((abs(leftY)-127)/10)*(1- exp(-(t/10))))*leftY * 4.7244;
    // float turn = 3.5 * rightX;//4.7244 * rightX
    float turn = (exp(-(t/10))+exp((abs(rightX)-127)/10)*(1- exp(-(t/10))))*rightX * 4;

    frontLeft.move_velocity(forward + turn);
    middleLeft.move_velocity(forward + turn);	
    backLeft.move_velocity(forward + turn);
    frontRight.move_velocity(forward - turn);
    middleRight.move_velocity(forward - turn);
    backRight.move_velocity(forward - turn);
}

