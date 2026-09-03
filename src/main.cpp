#include "main.h"
#include "lemlib/api.hpp"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}
// INITIALISATION DO NOT TOUCH
pros::MotorGroup left_motors({2, 3}, pros::MotorGearset::blue); // left motors on ports 2, 3
pros::MotorGroup right_motors({9, 10}, pros::MotorGearset::blue); // right motors on ports 9, 10
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, //new 3.25" omnisx
                              360, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);
pros::Imu imu(19);
pros::Rotation vertical_encoder(20);
pros::Rotation horizontal_encoder(18);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, 2.5); // horizontal tracking wheel, 2.
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, -2.5);
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);
// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);
// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::MotorGroup Cascade({1, -8}, pros::MotorGearset::green); // cascade motors on ports 1, 8
pros::Motor Claw(20);
pros::Distance ClawSense(12);
int CascadeUp = 0;
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(3, "Cascade: %d", Cascade.get_position()); // cascade position
            pros::lcd::print(4, "Claw: %d", Claw.get_position()); // claw position
            pros::lcd::print(5, "ClawSense: %d", ClawSense.get_distance()); // claw sensor
            pros::lcd::print(6, "CascadeUp: %d", CascadeUp); // cascade up
            // delay to save resources
            pros::delay(20);
        }
    });
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    // loop forever
    while (true) {
        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
        pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
        pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
        pros::lcd::print(3, "Cascade: %d", Cascade.get_position()); // cascade position
        pros::lcd::print(4, "Claw: %d", Claw.get_position()); // claw position
        pros::lcd::print(5, "ClawSense: %d", ClawSense.get_distance()); // claw sensor
        pros::lcd::print(6, "CascadeUp: %d", CascadeUp); // cascade up
        // move the robot
        // prioritize steering slightly
        chassis.arcade(leftY, leftX, false, 0.75);
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            Cascade.move_velocity(200);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            Cascade.move_velocity(-200);
        } else {
            Cascade.move_velocity(0);
        }
        if (ClawSense.get_distance() < 100) { // adjust threshold as needed
            Claw.move_absolute(270, 100);
            CascadeUp = 0;
        }
        if (ClawSense.get_distance() >= 100 and CascadeUp != 1) { // adjust threshold as needed
            Claw.move_absolute(0, 100);
            CascadeUp = 1;
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            Claw.move_velocity(200);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            Claw.move_velocity(-200);
        } else {
            Claw.move_velocity(0);
        }
        // delay to save resources
        pros::delay(25);
    }
}
