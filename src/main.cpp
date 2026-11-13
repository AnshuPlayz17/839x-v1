#include "main.h"

#include <cmath>

#include "autons.hpp"
#include "build.hpp"
#include "mechanisms.hpp"
#include "robot_config.hpp"

// ============================================================
// 839X Halloween
//
// CONTROLS
//   Left stick      drive (front = INTAKE)
//   Right stick     turn
//   L1 (hold)       intake + grabber in
//   L2 (hold)       intake + grabber out
//   Y               grab: elevator up a little, grip, wrist up
//   B               score: elevator down a little, grabber spins out,
//                   grabber stops and waits a moment, then elevator up
//                   a little, wrist up all the way. Nothing else.
//   L1 (press)      not holding a stack? elevator, then wrist, go back down to start
//   A               stop a Y / B move (keeps holding). Press again: let go
//   R1 / R2 (hold)  elevator up / down, full speed. Let go = it holds.
//                   At the bottom it tightens the string, then rests
//                   with the power off (stays cool).
//   Up / Down       elevator up / down by hand
//   Right / Left    wrist up / down by hand
//   Up + Down 1 s   reset elevator and wrist (both down first!)
//   X               test the auton (practice only, not on a field)
// ============================================================

namespace {

int selectedAuton = 0;

// Brain screen: tap the RIGHT half for the next auton, LEFT half to go back.
// Also shows the elevator, wrist and where the robot is.
// Measuring build screen (MEASURE_MODE in robot_config.hpp).
// Move things by hand and write these numbers down.
void measureScreenLoop() {
  while (true) {
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "839X  BUILD %d   MEASURING BUILD", BUILD_NUMBER);
    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Elevator + wrist are OFF. Move by hand.");
    pros::screen::print(pros::E_TEXT_MEDIUM, 3, "ELEVATOR E %6.0f  (motors %6.0f %6.0f)  ",
                        mech::liftDeg(), Lift.get_position(0), Lift.get_position(1));
    pros::screen::print(pros::E_TEXT_MEDIUM, 4, "WRIST W %6.1f  (motor %6.0f)   ", mech::wristDeg(),
                        Wrist.get_position());
    pros::screen::print(pros::E_TEXT_MEDIUM, 5, "TRACKING WHEEL %8.1f deg   ",
                        VerticalEncoder.get_position() / 100.0);
    pros::screen::print(pros::E_TEXT_MEDIUM, 6, "DRIVE LEFT %7.0f  RIGHT %7.0f deg   ",
                        LeftDrive.get_position(0), RightDrive.get_position(0));
    pros::screen::print(pros::E_TEXT_MEDIUM, 7, "HEADING %6.1f   BATTERY %3.0f%%   ",
                        chassis.getPose().theta, pros::battery::get_capacity());
    pros::screen::print(pros::E_TEXT_MEDIUM, 9, "A: zero tracking + drive");
    pros::screen::print(pros::E_TEXT_MEDIUM, 10, "UP + DOWN 1 s: zero elevator + wrist");
    pros::delay(100);
  }
}

void screenLoop() {
  if (MEASURE_MODE) measureScreenLoop();  // never comes back
  int lastPress = pros::screen::touch_status().press_count;
  while (true) {
    const pros::screen_touch_status_s_t touch = pros::screen::touch_status();
    if (touch.press_count != lastPress) {
      const int step = touch.x < 240 ? AUTON_COUNT - 1 : 1;  // brain screen is 480 wide
      selectedAuton = (selectedAuton + step) % AUTON_COUNT;
      lastPress = touch.press_count;
    }
    const lemlib::Pose pose = chassis.getPose();
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "AUTON %d/%d: %-24s", selectedAuton + 1, AUTON_COUNT,
                        AUTONS[selectedAuton].name);
    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "< tap left: back     tap right: next >");
    pros::screen::print(pros::E_TEXT_MEDIUM, 3, "ELEVATOR %6.0f   WRIST %6.0f   ", mech::liftDeg(),
                        mech::wristDeg());
    pros::screen::print(pros::E_TEXT_MEDIUM, 4, "TEMP %3.0fC %-11s %-8s", mech::liftTemp(), mech::liftStatus(),
                        mech::holding() ? "HOLDING" : "");
    pros::screen::print(pros::E_TEXT_MEDIUM, 5, "X %6.1f  Y %6.1f  HEADING %6.1f   ", pose.x, pose.y,
                        pose.theta);
    pros::screen::print(pros::E_TEXT_MEDIUM, 6, "BATTERY %3.0f%%   ELEVATOR %5.0f mA   ",
                        pros::battery::get_capacity(), mech::liftCurrent());
    pros::screen::print(pros::E_TEXT_MEDIUM, 7, "839X  BUILD %d", BUILD_NUMBER);
    pros::delay(100);
  }
}

// Controller screen: X, Y and heading on the field (see autons.cpp),
// then E = elevator and W = wrist. Use it to write down auton points.
// Turn the robot on at the auton start spot so the numbers are right.
// (The controller can only update one line at a time, so they take turns.)
void controllerLoop() {
  int line = 0;
  while (true) {
    const lemlib::Pose pose = chassis.getPose();
    double heading = std::fmod(pose.theta, 360.0);
    if (heading < 0) heading += 360.0;
    if (line == 0) {
      Controller1.print(0, 0, "X%6.1f Y%6.1f  ", pose.x, pose.y);
    } else if (line == 1) {
      Controller1.print(1, 0, "HEADING %5.1f   ", heading);
    } else {
      const bool hot = mech::liftTemp() >= LIFT_HOT_C;
      Controller1.print(2, 0, "E%5.0f W%4.0f %s%s ", mech::liftDeg(), mech::wristDeg(),
                        mech::holding() ? "H" : " ", hot ? " HOT" : "    ");
    }
    line = (line + 1) % 3;
    pros::delay(60);
  }
}

}  // namespace

void initialize() {
  pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Calibrating... keep the robot still");
  pros::screen::print(pros::E_TEXT_MEDIUM, 2, "839X  BUILD %d", BUILD_NUMBER);
  Controller1.print(0, 0, "839X BUILD %d   ", BUILD_NUMBER);
  chassis.calibrate();  // inertial sensor, 2-3 seconds, keep still
  // The robot starts at the auton start spot.
  chassis.setPose(START_X, START_Y, START_HEADING);
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
  mech::init();  // elevator and wrist start spot = 0 (start them down)
  pros::screen::erase();
  static pros::Task screenTask(screenLoop);
  static pros::Task controllerTask(controllerLoop);
}

// Stop any Y/B move left over from auton, so nothing moves by itself.
void disabled() { mech::stopSteps(); }

void competition_initialize() {}

void autonomous() {
  if (MEASURE_MODE) return;  // measuring build: no autons
  AUTONS[selectedAuton].run();
}

void opcontrol() {
  mech::stopSteps();
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
  std::uint32_t resetHeldSince = 0;
  int buzzCooldown = 0;
  bool wasHot = false;

  while (true) {
    // ---------- Drive ----------
    const int throttle = Controller1.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    const int turn = Controller1.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) * DRIVER_TURN_SCALE;
    chassis.arcade(throttle, turn);

    // ---------- Intake (hold) ----------
    const bool in = Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    const bool out = Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    if (Controller1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1) && !out) mech::home();
    mech::setIntake(in == out ? 0 : (in ? 1 : -1));

    // ---------- Buttons (press once) ----------
    if (Controller1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) mech::grab();
    if (Controller1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) mech::score();
    if (Controller1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      mech::cancel();
      if (MEASURE_MODE) {  // measuring build: A zeroes the tracking wheel and drive counts
        VerticalEncoder.reset_position();
        LeftDrive.tare_position_all();
        RightDrive.tare_position_all();
      }
    }
    const bool r1 = Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    const bool r2 = Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
    mech::manualLift(r1 == r2 ? 0 : (r1 ? 1 : -1));

    // ---------- Move by hand (hold) ----------
    const bool up = Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
    const bool down = Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
    if (up && !down) mech::nudgeLift(LIFT_NUDGE_DEG);
    if (down && !up) mech::nudgeLift(-LIFT_NUDGE_DEG);
    if (Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) mech::nudgeWrist(WRIST_NUDGE_DEG);
    if (Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) mech::nudgeWrist(-WRIST_NUDGE_DEG);

    // ---------- Up + Down for 1 second: reset elevator and wrist ----------
    if (up && down) {
      if (resetHeldSince == 0) resetHeldSince = pros::millis();
      if (resetHeldSince != 1 && pros::millis() - resetHeldSince >= 1000) {
        mech::rezero();
        Controller1.rumble(".");
        resetHeldSince = 1;  // once per hold
      }
    } else {
      resetHeldSince = 0;
    }

    // ---------- X: test the auton (not on a field) ----------
    if (Controller1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X) &&
        !pros::competition::is_connected()) {
      mech::setIntake(0);
      autonomous();
      chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    }

    // ---------- Something went wrong: buzz the controller ----------
    if (buzzCooldown > 0) buzzCooldown--;
    // Elevator getting hot: one long buzz when it crosses LIFT_HOT_C.
    const bool hot = mech::liftTemp() >= LIFT_HOT_C;
    if (hot && !wasHot) Controller1.rumble("---");
    wasHot = hot;
    if (mech::takeAlert() && buzzCooldown == 0) {
      Controller1.rumble("--");
      buzzCooldown = 50;  // at most once a second
    }

    pros::delay(20);
  }
}
