#include "Intake.hpp"

Intake::Intake(pros::Motor LEFT, pros::Motor RIGHT, pros::adi::DigitalOut BOT, pros::adi::DigitalOut TOP, pros::adi::DigitalOut DOUBLEPARK, pros::Distance DISTANCE) : left(RIGHT), right(RIGHT), bot(BOT), top(TOP), doublepark(DOUBLEPARK), distance(DISTANCE){}

void Intake::brake(){
    left.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    right.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}

void Intake::telOP(bool intake, bool scoreTop, bool scoreMid, bool outtake){
    if(outtake){
			left.move_velocity(-200);
			right.move_velocity(-200);
    }
    else if(intake){
        left.move_velocity(200);
        right.move_velocity(200);
        bot.set_value(true);
        top.set_value(false);
    }
    else if(scoreTop){
        bot.set_value(false);
        top.set_value(false);
        left.move_velocity(200);
        right.move_velocity(200);
    }
    else if(scoreMid){
        left.move_velocity(200);
        right.move_velocity(200);
        bot.set_value(true);
        top.set_value(true);
    }
    else{
        left.move_velocity(0);
        right.move_velocity(0);
    }
}