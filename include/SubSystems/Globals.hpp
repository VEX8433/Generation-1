#pragma once
#include "lemlib/api.hpp"
#include "SubSystems/Intake.hpp"
#include "SubSystems/Localizer.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/motor_group.hpp"

// Global Robot Subsystems
extern lemlib::Chassis chassis;
extern Intake intake;
extern Localizer localizer;

// ADI Pneumatics
extern pros::adi::DigitalOut bot;
extern pros::adi::DigitalOut top;
extern pros::adi::DigitalOut doinker;
extern pros::adi::DigitalOut tongue;
extern pros::adi::DigitalOut backDoinker;
extern pros::adi::DigitalOut doublePark;

// Sensors
extern pros::Distance distFront;
extern pros::Distance distLeft;
extern pros::Distance distBack;

// Motor Groups (needed for direct access in some resets)
extern pros::MotorGroup left_motors;
extern pros::MotorGroup right_motors;

// Constants
constexpr float OFFSET_FRONT = 0.0f;
constexpr float OFFSET_LEFT = 0.0f;
constexpr float OFFSET_BACK = 0.0f;
