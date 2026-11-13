#include "robot_config.hpp"

// Ports. A minus sign means that motor spins the other way.
// Same ports and directions as 839X's old code.
pros::Controller Controller1(pros::E_CONTROLLER_MASTER);

pros::MotorGroup LeftDrive({-3, 4}, pros::MotorGearset::blue, pros::MotorUnits::degrees);
pros::MotorGroup RightDrive({-6, 7}, pros::MotorGearset::blue, pros::MotorUnits::degrees);
pros::MotorGroup Lift({-8, 5}, pros::MotorGearset::green, pros::MotorUnits::degrees);
pros::Motor Intake(2, pros::MotorGearset::blue, pros::MotorUnits::degrees);
pros::Motor Grabber(11, pros::MotorGearset::green, pros::MotorUnits::degrees);
pros::Motor Wrist(-13,  // flipped 10-30 with the 1:3 gears 
                   pros::MotorGearset::green, pros::MotorUnits::degrees);
pros::Imu Inertial(20);
pros::Rotation VerticalEncoder(9);  // vertical tracking wheel. Counts down going forward? make it -9

// ---------- LemLib ----------
lemlib::Drivetrain drivetrain(&LeftDrive, &RightDrive, TRACK_WIDTH_IN, WHEEL_DIAMETER_IN,
                              DRIVE_WHEEL_RPM, 8);  // 8 = center traction wheels

// The vertical tracking wheel measures distance, the inertial sensor measures turning.
lemlib::TrackingWheel verticalWheel(&VerticalEncoder, TRACKING_WHEEL_IN, TRACKING_OFFSET_IN);
lemlib::OdomSensors sensors(&verticalWheel, nullptr, nullptr, nullptr, &Inertial);

// Driving straight. Starting values.
// If it goes past the target, raise kD. If it stops short, raise kP.
lemlib::ControllerSettings linearController(10,   // kP
                                            0,    // kI
                                            3,    // kD
                                            3,    // anti-windup range
                                            1,    // small error (in)
                                            100,  // small error timeout (ms)
                                            3,    // large error (in)
                                            150,  // large error timeout (ms): short, so no waiting between moves
                                            20);  // slew: how fast it speeds up

// Turning. Starting values.
lemlib::ControllerSettings angularController(2,    // kP
                                             0,    // kI
                                             10,   // kD
                                             3,    // anti-windup range
                                             1,    // small error (deg)
                                             100,  // small error timeout (ms)
                                             3,    // large error (deg)
                                             150,  // large error timeout (ms): short, so no waiting between moves
                                             0);   // slew

// Joystick feel: straight line, ignores tiny stick movements.
lemlib::ExpoDriveCurve throttleCurve(6, 0, 1);
lemlib::ExpoDriveCurve steerCurve(6, 0, 1);

lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors,
                        &throttleCurve, &steerCurve);
