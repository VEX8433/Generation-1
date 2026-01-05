#pragma once

#include "api.h"

class Chassis{
  public:
    Chassis(pros::Motor FrontLeft, pros::Motor FrontRight, pros::Motor MiddleLeft, pros::Motor MiddleRight, pros::Motor BackLeft, pros::Motor BackRight, pros::Imu inertial, pros::Rotation tracking);
    void brake();

    void inlineTelOp(int leftY, int rightX);

  private:
    pros::Motor frontLeft;
    pros::Motor frontRight;
    pros::Motor middleLeft;
    pros::Motor middleRight;
    pros::Motor backLeft;
    pros::Motor backRight;

    pros::Imu inertial;
    pros::Rotation tracking;
};