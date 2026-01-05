#pragma once

#include "api.h"

class Intake{
  public:
    Intake(pros::Motor LEFT, pros::Motor RIGHT, pros::adi::DigitalOut BOT, pros::adi::DigitalOut TOP, pros::adi::DigitalOut DOUBLEPARK, pros::Distance DISTANCE);

    void brake();
    void telOP(bool intake, bool scoreTop, bool scoreMid, bool outtake);

  private:
    pros::Motor left;
    pros::Motor right;
    pros::Distance distance;
    pros::adi::DigitalOut bot;
    pros::adi::DigitalOut top;
    pros::adi::DigitalOut doublepark;
};