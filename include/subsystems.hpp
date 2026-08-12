#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::Motor intake(5);
inline pros::adi::DigitalIn limit_switch('A');
// inline pros::Vision vision_sensor1(20);
inline pros::Motor Cascade(8);
inline pros::Motor ChainBar_1(4);
inline pros::Motor ChainBar_2(7);
inline pros::Motor Hmotor(3);
