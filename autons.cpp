#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 127;
const int TURN_SPEED = 90;
const int SWING_SPEED = 127;

///
// Constants
///
void default_constants() {

  // Drive PID (aggressive but stable)
  chassis.pid_drive_constants_set(12.0, 0.0, 35.0);

  // Keeps robot straight while driving
  chassis.pid_heading_constants_set(9.0, 0.0, 15.0);

  // Faster turns
  chassis.pid_turn_constants_set(2.5, 0.02, 18.0, 15.0);

  // Faster swings
  chassis.pid_swing_constants_set(4.5, 0.0, 40.0);

  // Odom motion control
  chassis.pid_odom_angular_constants_set(5.0, 0.0, 30.0);
  chassis.pid_odom_boomerang_constants_set(4.0, 0.0, 25.0);

  // Exit faster (robot stops waiting as long)
  chassis.pid_turn_exit_condition_set(70_ms, 2_deg, 200_ms, 5_deg, 400_ms, 400_ms);
  chassis.pid_swing_exit_condition_set(70_ms, 2_deg, 200_ms, 5_deg, 400_ms, 400_ms);
  chassis.pid_drive_exit_condition_set(70_ms, 0.75_in, 200_ms, 2_in, 400_ms, 400_ms);

  chassis.pid_odom_turn_exit_condition_set(70_ms, 2_deg, 200_ms, 5_deg, 400_ms, 600_ms);
  chassis.pid_odom_drive_exit_condition_set(70_ms, 0.75_in, 200_ms, 2_in, 400_ms, 600_ms);

  chassis.pid_turn_chain_constant_set(2_deg);
  chassis.pid_swing_chain_constant_set(3_deg);
  chassis.pid_drive_chain_constant_set(2_in);

  // VERY important for speed
  chassis.slew_drive_constants_set(1_in, 127);
  chassis.slew_turn_constants_set(1_deg, 127);
  chassis.slew_swing_constants_set(1_in, 127);

  // Odom path following
  chassis.odom_turn_bias_set(1.0);
  chassis.odom_look_ahead_set(16_in);
  chassis.odom_boomerang_distance_set(20_in);
  chassis.odom_boomerang_dlead_set(0.7);

  chassis.pid_angle_behavior_set(ez::shortest);
}
///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

// . . .
// Make your own autonomous functions here!
// . . .
void blueauton() {
  chassis.odom_xyt_set(161, -38, 270);
}