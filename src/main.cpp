    #include "main.h"
    #include "lemlib/api.hpp"
    #include "autonSelector.hpp"
    //merged auton test branch
    /**
     * A callback function for LLEMU's center button.
     *
     * When this callback is fired, it will toggle line 2 of the LCD text between
     * "I was pressed!" and nothing.
     */
    int autonSelected = 0;
    // INITIALISATION DO NOT TOUCH
    pros::MotorGroup left_motors({1,-2, -3}, pros::MotorGearset::blue); // left motors on ports 2, 3
    pros::MotorGroup right_motors({-8, 9, 10}, pros::MotorGearset::blue); // right motors on ports 9, 10
    lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                                &right_motors, // right motor group
                                12.7, // 12.7 inch track width
                                lemlib::Omniwheel::NEW_325, //new 3.25" omnisx
                                360, // drivetrain rpm is 360
                                2 // horizontal drift is 2 (for now)
    );
    pros::Imu imu(4);
    pros::Rotation vertical_encoder(5);
    pros::Rotation horizontal_encoder(6);
    // lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, 0.75); // horizontal tracking wheel, 2.
    lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, 0);
    lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                                nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                                // &horizontal_tracking_wheel, // horizontal tracking wheel 1
                                nullptr,
                                nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                                &imu // inertial sensor
    );
    // lateral PID controller
    lemlib::ControllerSettings lateral_controller(24.7, // proportional gain (kP)
                                                0, // integral gain (kI)
                                                0, // derivative gain (kD)
                                                3, // anti windup
                                                1, // small error range, in inches
                                                100, // small error range timeout, in milliseconds
                                                3, // large error range, in inches
                                                500, // large error range timeout, in milliseconds
                                                3 // maximum acceleration (slew)
    );

    // angular PID controller
    lemlib::ControllerSettings angular_controller(3, // proportional gain (kP)
                                                0, // integral gain (kI)
                                                10, // derivative gain (kD)
                                                3, // anti windup
                                                1, // small error range, in degrees
                                                100, // small error range timeout, in milliseconds
                                                3, // large error range, in degrees
                                                500, // large error range timeout, in milliseconds
                                                5 // maximum acceleration (slew)
    );
    // create the chassis
    lemlib::Chassis chassis(drivetrain, // drivetrain settings
                            lateral_controller, // lateral PID settings
                            angular_controller, // angular PID settings
                            sensors // odometry sensors
    );
    pros::Controller controller(pros::E_CONTROLLER_MASTER);
    pros::MotorGroup Cascade({-11, 12}, pros::MotorGearset::green); // cascade motors on ports 1, 8
    pros::Motor Claw({7}, pros::MotorGearset::green);
    pros::Optical OpticalSensor(8);


    int autonRunning = 0;
    int autonFinished = 0;
    int drivething = 0;
    // autons are defined further down, declared here so the selector can list them
    void Right_Quad();
    void Left_Quad();
    void Skills();
    void RightQuadNoStack();
    void pidTest();
    /**
     * Runs initialization code. This occurs as soon as the program is started.
     *
     * All other competition modes are blocked by initialize; it is recommended
     * to keep execution time for this mode under a few seconds.
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
    void on_right_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
    }
    void on_left_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
    }


void initialize() {
        left_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
        right_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

    // 1. BUILD THE AUTON SELECTOR FIRST (replaces pros::lcd, don't use both)
    // Tap a button on the brain screen to pick the auton. The highlighted one runs in autonomous().
    selector::init({
        {"RIGHT QUAD", "Right side quad. Starts at (0, 65) facing 0.", Right_Quad},
        {"LEFT QUAD", "Left side quad. Starts at (61.8, 9.1) facing -135.", Left_Quad},
        {"SKILLS", "Programming skills run. Starts at (0, 65) facing 270.", Skills},
        {"Right Quad No stack", "Same as right quad without the second pin and cup", RightQuadNoStack},
        {"PID TEST", "Drives 24 in forward with moveToPose for PID tuning.", pidTest},
        {"NONE", "Does nothing. Robot stays still.", nullptr},
    }, 0);

    // 2. START THE SCREEN TASK
    // Shows the odom pose on the brain and the selected auton on the controller.
    pros::Task screen_task([&]() {
        int lastShown = -1;
        while (true) {
            lemlib::Pose pose = chassis.getPose();
            char status[48];
            snprintf(status, sizeof(status), "X %.1f  Y %.1f  T %.1f", pose.x, pose.y, pose.theta);
            selector::setStatus(status);
            // printf("X: %f\n", pose.x); // x
            // printf("Y: %f\n", pose.y); // y
            // printf("Theta: %f\n", pose.theta); // heading

            // controller screen only updates when the pick changes (it's slow to write to)
            if (selector::selectedIndex() != lastShown) {
                lastShown = selector::selectedIndex();
                controller.print(0, 0, "Auton: %-10s", selector::selectedName().c_str());
            }

            // Standard delay
            pros::delay(50);
        }
    });

    // 3. RUN CALIBRATION AFTER THE TASK STARTS
    // The robot will calibrate in the background while your UI is fully functional.
    chassis.calibrate(); 
    Cascade.set_zero_position(0);
    Claw.set_zero_position(0);
}

    /**
     * Runs while the robot is in the disabled state of Field Management System or
     * the VEX Competition Switch, following either autonomous or opcontrol. When
     * the robot is enabled, this task will exit.
     */

    void disabled() {
        //skills
        Claw.move_absolute(1000,127);
        //normal
        // Claw.move_absolute(0,127);
    }

    void RightQuadNoStack() {

    }
    void pidTest() {
    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    // turn to face heading 90 with a very long timeout
    chassis.moveToPose(0, 24, 0 , 3000);
    chassis.waitUntilDone();
    printf("X: %f\n", chassis.getPose().x); // x
    printf("Y: %f\n", chassis.getPose().y); // y
    }
    void Right_Quad() {
    
    }
    void Left_Quad() {

    }
    void Skills() {

    }


    /**
     * Runs after initialize(), and before autonomous when connected to the Field
     * Management System or the VEX Competition Switch. This is intended for
     * competition-specific initialization routines, such as an autonomous selector
     * on the LCD.
     *
     * This task will exit when the robot is enabled and autonomous or opcontrol
     * starts.
     */
    void competition_initialize() {
        left_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
        right_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
        Cascade.set_zero_position(0);
        Claw.set_zero_position(0);
    }

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
    void autonomous() {
        int autonstart = 0;
        printf("Autonomous started: %s\n", selector::selectedName().c_str());
        // runs whatever was picked on the brain screen
            selector::runSelected();
    }


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

    int drivething2;
    void opcontrol() {
        // Claw.set_zero_position(30);
        left_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
        right_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
        Claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        Cascade.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
        int CascadeUp = 0;
        std::uint32_t liftToIntakeUntil = 0; // Y/B keep raising the cascade until this time
        int clawState1 = 0;
        // Threshold angle in degrees where the robot is considered "tipping"
        const double TILT_THRESHOLD = 7.0;
        // Power applied to counteract the tip (out of 127)
        const int CORRECTION_POWER = 80;
        // pros::Task ClawupTask([&]() {
        //     while (true) {
        //         if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        //             Pickup.move_voltage(12000);
        //             pros::delay(800);
        //             Claw.move_absolute(1100, 127);
        //         } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        //             Pickup.move_voltage(-12000);
        //             Claw.move_absolute(480, 127);
        //         } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
        //             Claw.move_absolute(680,127);
        //         }
        //         pros::delay(25);
        //     }
        // });
        while (true) {
            // get left y and right x positions
            int RightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
            int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
            chassis.arcade(leftY, RightX, false, 0.75);
            pros::Task AutoToggle([&]() {
            bool wasMatching = false;
            const int OBJECT_THRESHOLD = 100;
            const double MIN_SATURATION = 0.25;
            const double RED_LOW_MIN = 0;
            const double RED_LOW_MAX = 20;
            const double RED_HIGH_MIN = 340;
            const double RED_HIGH_MAX = 360;
            const double BLUE_MIN = 190;
            const double BLUE_MAX = 250;
            while (true) {
                double hue = opticalSensor.get_hue();
                double saturation = opticalSensor.get_saturation();
                int proximity = opticalSensor.get_proximity();

                bool objectDetected =
                    proximity > OBJECT_THRESHOLD &&
                    saturation > MIN_SATURATION &&
                    std::isfinite(hue);
                bool matchingColour = false;

                if (objectDetected) {
                    if (selector::alliance() == selector::Alliance::RED) {
                        matchingColour =
                            (hue >= RED_LOW_MIN && hue <= RED_LOW_MAX) ||
                            (hue >= RED_HIGH_MIN && hue <= RED_HIGH_MAX);
                    }

                    else if (selector::alliance() == selector::Alliance::BLUE) {
                        matchingColour =
                            hue >= BLUE_MIN && hue <= BLUE_MAX;
                    }
                }
                if (matchingColour && !wasMatching) {
                    autotogglemotor.move_relative(180, 100);
                }

                wasMatching = matchingColour;

                pros::delay(25);
            }
        });
            pros::delay(25);
        }
    }
 
