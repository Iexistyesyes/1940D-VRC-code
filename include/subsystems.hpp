#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');
// inline pros::Vision vision_sensor1(20);
inline pros::Motor intake_motor(5);
inline pros::Motor Cascade(8);
inline pros::Motor TwoBar(6);
inline pros::Rotation Chain_Bar(7);
inline pros::Motor Hmotor(3);