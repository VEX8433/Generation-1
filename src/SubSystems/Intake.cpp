#include "Intake.hpp"

Intake::Intake(pros::Motor LEFT, pros::Motor RIGHT, pros::adi::DigitalOut BOT, pros::adi::DigitalOut TOP, pros::adi::DigitalOut DOUBLEPARK, pros::Distance DISTANCE) : left(LEFT), right(RIGHT), bot(BOT), top(TOP), doublepark(DOUBLEPARK), distance(DISTANCE){}

void Intake::telOP(bool intake, bool scoreTop, bool scoreMid, bool outtake, bool park, bool prime){
    if(outtake){
        left.move_velocity(-600);
        right.move_velocity(-600);
    }
    else if(intake){
        left.move_velocity(600);
        right.move_velocity(600);
        bot.set_value(true);
        top.set_value(false);
    }
    else if(scoreTop){
        bot.set_value(false);
        top.set_value(false);
        left.move_velocity(600);
        right.move_velocity(600);
    }
    else if(scoreMid){
        left.move_velocity(600);
        right.move_velocity(600);
        bot.set_value(true);
        top.set_value(true);
    }
    else if(prime){
        while(distance.get_distance() > 30){
            left.move_velocity(-200);
            right.move_velocity(-200);
        }
        left.move_velocity(0);
        right.move_velocity(0);
    }
    else if(park){
        while(distance.get_distance() < 60){
            left.move_velocity(-200);
            right.move_velocity(-200);
        }
        left.move_velocity(0);
        right.move_velocity(0);
        doublepark.set_value(true);
    }
    else{
        left.move_velocity(0);
        right.move_velocity(0);
    }
}