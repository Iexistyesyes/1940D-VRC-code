    #include "main.h"
    #include "lemlib/api.hpp"
    #include "autonSelector.hpp"
    #include "settings.hpp"
    #include "underglow.hpp"
    //merged auton test branch
    /**
     * A callback function for LLEMU's center button.
     *
     * When this callback is fired, it will toggle line 2 of the LCD text between
     * "I was pressed!" and nothing.
     */
    int autonSelected = 0;
    // INITIALISATION DO NOT TOUCH
    pros::MotorGroup left_motors({1,-2}, pros::MotorGearset::blue); // left motors on ports 2, 3
    pros::MotorGroup right_motors({-8, 9}, pros::MotorGearset::blue); // right motors on ports 9, 10
    lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                                &right_motors, // right motor group
                                12.7, // 12.7 inch track width
                                lemlib::Omniwheel::NEW_275, //new 3.25" omnisx
                                450, // drivetrain rpm is 360
                                2 // horizontal drift is 2 (for now)
    );
    pros::Imu imu(4);
    pros::Rotation vertical_encoder(5);
    pros::Rotation horizontal_encoder(6);
    // lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, 0.75); // horizontal tracking wheel, 2.
    lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, 0);
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
                                                3, // derivative gain (kD)
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
    pros::MotorGroup Cascade({-11, 20}, pros::MotorGearset::blue); // cascade motors on ports 1, 8
    pros::Motor Claw({7}, pros::MotorGearset::green);
    pros::Optical opticalSensor(8);
    pros::Motor Togglemotor({13}, pros::MotorGearset::green);
    pros::adi::Pneumatics ToggleSolenoid('A', false);
    int toggleBlock = 0;
    int autonRunning = 0;
    int autonFinished = 0;
    int drivething = 0;
    // autons are defined further down, declared here so the selector can list them
    void Right_Quad();
    void Left_Quad();z
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

    // 1. BUILD THE BRAIN SCREEN FIRST (replaces pros::lcd, don't use both)
    // Tabs: AUTON (pick the auton), STATS, ALLIANCE (red / blue), DISPLAY (picture shown during the match)
    // and LEDS (underglow animations).
    selector::init(
        // autons: 12 fit on a page, add more and page arrows show up
        {
            {"RIGHT QUAD", "Right side quad. Starts at (0, 65) facing 0.", Right_Quad},
            {"LEFT QUAD", "Left side quad. Starts at (61.8, 9.1) facing -135.", Left_Quad},
            {"SKILLS", "Programming skills run. Starts at (0, 65) facing 270.", Skills},
            {"Right Quad No stack", "Same as right quad without the second pin and cup", RightQuadNoStack},
            {"PID TEST", "Drives 24 in forward with moveToPose for PID tuning.", pidTest},
            {"NONE", "Does nothing. Robot stays still.", nullptr},
        },
        // tiles on the STATS tab (up to 12)
        {
            selector::motorTemp("L DRIVE", left_motors),
            selector::motorTemp("R DRIVE", right_motors),
            selector::motorTemp("CASCADE", Cascade),
            selector::motorTemp("CLAW", Claw),
            selector::motorTemp("Auto-Toggle", Togglemotor),
            // selector::motorTemp("INTAKE", Intake),
            selector::battery(),
            {"X", [] { return (double)chassis.getPose().x; }, "%.1f"},
            {"Y", [] { return (double)chassis.getPose().y; }, "%.1f"},
            {"HEADING", [] { return (double)chassis.getPose().theta; }, "%.1f°"},
            {"CASC POS", [] { return Cascade.get_position(); }},
            {"CLAW POS", [] { return Claw.get_position(); }},
        });

    // underglow on 3-wire ports A and B (LED count + brightness are in include/underglow.hpp).
    // Numbers are positions in the list in src/underglowAnimations.cpp, changed on the brain's LEDS tab:
    // before / after the match = CHRISTMAS, during the match = LOGO YELLOW
    underglow::init(/*idle*/ 1, /*match*/ 0);

    // remember the picks above between runs (saved on the SD card, see include/settings.hpp).
    // The defaults above are only used the first time, or when there's no SD card
    settings::init();

    // 2. START THE SCREEN TASK
    // Prints the odom pose to the terminal and shows the alliance + auton pick on the controller.
    pros::Task screen_task([&]() {
        int lastAuton = -1;
        int lastAlliance = -1;
        while (true) {
            lemlib::Pose pose = chassis.getPose();
            (void)pose; // only used by the printfs below
            // printf("X: %f\n", pose.x); // x
            // printf("Y: %f\n", pose.y); // y
            // printf("Theta: %f\n", pose.theta); // heading

            // controller screen only updates when a pick changes (it's slow to write to)
            int auton = selector::selectedIndex();
            int alliance = selector::isRed() ? 0 : 1;
            if (auton != lastAuton || alliance != lastAlliance) {
                lastAuton = auton;
                lastAlliance = alliance;
                controller.print(0, 0, "%s %-12s", alliance == 0 ? "RED" : "BLU", selector::selectedName().c_str());
            }

            // Standard delay
            pros::delay(50);
        }
    });

    // 3. RUN CALIBRATION AFTER THE TASK STARTS
    // The robot will calibrate in the background while your UI is fully functional.
    chassis.calibrate(); 
    Cascade.set_zero_position(0);
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

    void MotionChainTest() {
        chassis.setPose(0, 0, 0);
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
        Cascade.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
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
                    Togglemotor.move_relative(180, 100);
                }

                wasMatching = matchingColour;

                pros::delay(25);
            }
        });
        while (true) {
            // get left y and right x positions
            int RightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
            int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
            chassis.arcade(-RightX, -leftY, false, 0.75);
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                Cascade.move_voltage(12000);
            } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                Cascade.move_voltage(-12000);
            } else {
                Cascade.brake();
            }
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
                Claw.move_voltage(12000);
            } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                Claw.move_voltage(-12000);
            } else {
                Claw.brake();
            }
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
                if (toggleBlock == 0) {
                    ToggleSolenoid.set_value(true);
                    toggleBlock = 1;
                } else if (toggleBlock == 1) {
                    ToggleSolenoid.set_value(false);
                    toggleBlock = 0;
                }
            }
            if (Cascade.get_position() < 0) {
                pros::millis + 1000;
                if (pros::millis() != 0) {
                    Cascade.move_voltage(6000);
                    pros::delay(25);
                }
            }
            pros::delay(25);
        }    
    }
 
