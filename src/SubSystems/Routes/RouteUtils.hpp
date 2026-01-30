#pragma once
#include "SubSystems/Globals.hpp"

/**
 * @brief Reset robot position by moving to a target distance from wall
 * Slows down as it approaches target for accuracy.
 * 
 * @param targetDist Target distance in mm
 * @param useFront true = use front sensor, false = use back sensor
 * @param speed Max motor velocity for adjustment (default 50)
 */
void resetToDistance(int targetDist, bool useFront, int speed = 50);

/**
 * @brief Configure intake for long goal scoring
 */
void long_goal_score(bool active);

/**
 * @brief Configure intake for middle goal scoring
 */
void middle_goal_score(bool active);

/**
 * @brief Activate match load mechanism
 */
void matchload_activate(bool active);
