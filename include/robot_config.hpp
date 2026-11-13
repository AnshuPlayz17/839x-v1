#pragma once

#include "api.h"
#include "lemlib/api.hpp"

// ============================================================
// !!!!  PLACEHOLDER FILE  !!!!
// 839X's real robot_config.hpp was not in the upload, so this one was
// rebuilt from the .cpp files. Numbers marked GUESS are NOT the team's
// numbers. Do not run the elevator or wrist with this file: replace it
// with the team's real robot_config.hpp first.
// Numbers marked FROM CODE were found in comments in the team's .cpp files.
// ============================================================

// ---------- Devices (ports are in src/robot_config.cpp) ----------
extern pros::Controller Controller1;
extern pros::MotorGroup LeftDrive;
extern pros::MotorGroup RightDrive;
extern pros::MotorGroup Lift;
extern pros::Motor Intake;
extern pros::Motor Grabber;  // 5.5 W motor
extern pros::Motor Wrist;    // 5.5 W motor
extern pros::Imu Inertial;
extern pros::Rotation VerticalEncoder;
extern lemlib::Chassis chassis;

// ---------- Drive ----------
constexpr double TRACK_WIDTH_IN = 12.0;     // GUESS: left wheels to right wheels, center to center
constexpr double WHEEL_DIAMETER_IN = 3.25;  // GUESS
constexpr double DRIVE_WHEEL_RPM = 450;     // GUESS: wheel speed after the gears
constexpr double TRACKING_WHEEL_IN = 2.0;   // GUESS: tracking wheel diameter
constexpr double TRACKING_OFFSET_IN = 0;    // GUESS: tracking wheel left (-) / right (+) of center
constexpr double GRABBER_OFFSET_IN = 10.0;  // GUESS: robot center to the grabber
constexpr double DRIVER_TURN_SCALE = 0.8;   // GUESS: 1 = full turn speed

// ---------- Auton start spot ----------
constexpr double START_X = 0;          // FROM CODE (the auton's start)
constexpr double START_Y = -62.7;      // FROM CODE
constexpr double START_HEADING = 180;  // FROM CODE

// ---------- Elevator (motor degrees) ----------
constexpr double LIFT_MAX_DEG = 300;          // GUESS, kept low on purpose
constexpr double LIFT_RPM = 100;              // GUESS
constexpr double LIFT_TOLERANCE_DEG = 10;     // GUESS
constexpr double LIFT_NUDGE_DEG = 5;          // GUESS
constexpr int LIFT_MANUAL_VOLTS = 60;         // GUESS: R1/R2 power, -127 to 127
constexpr int LIFT_SLOW_VOLTS = 30;           // GUESS: power near the top / bottom
constexpr double LIFT_SLOW_ZONE_DEG = 60;     // GUESS
constexpr double LIFT_REST_DEG = 15;          // GUESS: below this = at the bottom
constexpr double HOME_BOTTOM_SLACK_DEG = 40;  // GUESS
constexpr double LIFT_GAP_DEG = 40;           // GUESS: the 2 motors may differ this much
constexpr int LIFT_SETTLE_MS = 200;           // GUESS
constexpr int LIFT_TENSION_MV = 1500;         // GUESS: string tightening power
constexpr double LIFT_TENSION_MA = 600;       // GUESS
constexpr double LIFT_HOT_C = 50;             // GUESS

// ---------- Wrist (wrist degrees, after the gears) ----------
constexpr double WRIST_GEAR = 3.0;  // FROM CODE ("1:3 gears"): motor turns 3 for 1 wrist turn
constexpr double WRIST_DOWN = 0;
constexpr double WRIST_VERTICAL = 90;    // GUESS
constexpr double WRIST_SCORE_DEG = 120;  // GUESS
constexpr double WRIST_MAX_DEG = 120;    // GUESS
constexpr double WRIST_RPM = 40;         // GUESS
constexpr double WRIST_GRAB_RPM = 30;    // GUESS
constexpr double WRIST_TOLERANCE_DEG = 4;  // GUESS
constexpr double WRIST_NUDGE_DEG = 1;    // GUESS

// ---------- Intake and grabber ----------
constexpr int INTAKE_IN_RPM = 600;         // GUESS
constexpr int GRABBER_IN_RPM = 200;        // GUESS
constexpr int GRABBER_HOLD_RPM = 30;       // GUESS
constexpr int GRABBER_RELEASE_RPM = -200;  // GUESS

// ---------- Y (grab) and B (score) ----------
constexpr double GRAB_CLEAR_DEG = 60;      // GUESS: Y lifts this much first
constexpr int GRAB_GRIP_MS = 250;          // GUESS
constexpr double STACK_DROP_DIP_DEG = 30;  // FROM CODE ("B uses 30")
constexpr int SCORE_OUT_FIRST_MS = 400;    // GUESS
constexpr int SCORE_PAUSE_MS = 200;        // GUESS
constexpr double SCORE_RAISE_DEG = 60;     // GUESS
constexpr bool SCORE_SPIN_WHILE_RAISING = false;  // GUESS

// ---------- Auton stack heights ----------
constexpr double STACK_CLEAR_DEG = 250;  // GUESS: high enough to pass over 2 standing stacks
constexpr double STACK_GRAB_DEG = 50;    // FROM CODE ("50 didn't grab it" for UPPER)

// ---------- Stuck check ----------
constexpr double STALL_CURRENT_MA = 2000;  // GUESS
constexpr double STALL_RPM = 5;            // GUESS
constexpr std::uint32_t STALL_MS = 300;    // GUESS
