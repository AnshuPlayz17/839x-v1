#include "mechanisms.hpp"

#include <algorithm>
#include <cmath>
#include <mutex>

namespace mech {
namespace {

// How it works: buttons and autons only change TARGETS (where the
// elevator and wrist should go). A background loop moves the motors there
// and runs the Y and B steps one after another.
// The lock stops two loops changing the same number at the same time.
pros::Mutex lock;

double liftTarget = 0;
double wristTarget = 0;
double wristRpm = WRIST_RPM;
int liftManual = 0;        // R1/R2 held: 1 up, -1 down, 0 none
bool liftManualBlocked = false;  // stuck while pushing: wait for the button to let go
// Resting at the bottom: drop onto the stop, tighten the string, then stop.
enum class Rest { No, Settle, Tension, Done };
Rest rest = Rest::No;
std::uint32_t restSince = 0;
double liftTempC = 0;

bool holdingStack = false;
bool scoreWristUp = true;  // B ends with the wrist up all the way (autons: false)
int intakeCmd = 0;  // 1 in, -1 out, 0 stop
bool alert = false;
bool gapStop = false;

// The steps of Y (grab) and B (score).
enum class Step {
  Idle,
  GrabLift,     // Y1: elevator up a little
  GrabGrip,     // Y2: grabber grips
  GrabWrist,    // Y3: wrist to vertical
  ScoreDip,     // B1: elevator down a little
  ScoreRelease, // B2: grabber spins out
  ScorePause,   // B3: grabber stops, wait a moment so the stack settles
  ScoreRaise,   // B4: elevator up a little
  ScoreWrist,   // B5: wrist up all the way
  HomeLift,     // L1 after B: elevator down to start
  HomeWrist,    // then wrist down to start
};
Step step = Step::Idle;
std::uint32_t stepStart = 0;
double liftNow() { return (Lift.get_position(0) + Lift.get_position(1)) / 2.0; }
double wristNow() { return Wrist.get_position() / WRIST_GEAR; }

double clampLift(double deg) { return std::clamp(deg, 0.0, LIFT_MAX_DEG); }
double clampWrist(double deg) { return std::clamp(deg, 0.0, WRIST_MAX_DEG); }

bool liftThere() { return std::fabs(liftTarget - liftNow()) <= LIFT_TOLERANCE_DEG; }
bool wristThere() { return std::fabs(wristTarget - wristNow()) <= WRIST_TOLERANCE_DEG; }

// B is running (B moves the elevator itself, even near the bottom).
bool scoringNow() {
  return step == Step::ScoreDip || step == Step::ScoreRelease || step == Step::ScorePause ||
         step == Step::ScoreRaise || step == Step::ScoreWrist;
}

void goStep(Step next) {
  step = next;
  stepStart = pros::millis();
}

// Something took too long: stop the steps, stay where we are, buzz.
void fail() {
  step = Step::Idle;
  wristRpm = WRIST_RPM;
  liftTarget = clampLift(liftNow());
  wristTarget = clampWrist(wristNow());
  alert = true;
}

// Stall check: pushing hard, not moving, not at the target = stuck.
struct StallWatch {
  std::uint32_t since = 0;
  bool check(double current, double rpm, double error, double tolerance) {
    const bool stuck = current > STALL_CURRENT_MA && std::fabs(rpm) < STALL_RPM &&
                       std::fabs(error) > tolerance;
    if (!stuck) {
      since = 0;
      return false;
    }
    if (since == 0) since = pros::millis();
    return pros::millis() - since >= STALL_MS;
  }
};

// Runs the Y and B steps. Each step waits for the one before it.
void runSteps() {
  const std::uint32_t elapsed = pros::millis() - stepStart;
  switch (step) {
    case Step::Idle:
      break;
    case Step::GrabLift:
      if (liftThere()) {
        holdingStack = true;
        goStep(Step::GrabGrip);
      } else if (elapsed > 1500) {
        fail();
      }
      break;
    case Step::GrabGrip:
      if (elapsed >= GRAB_GRIP_MS) {
        wristRpm = WRIST_GRAB_RPM;
        wristTarget = WRIST_VERTICAL;
        goStep(Step::GrabWrist);
      }
      break;
    case Step::GrabWrist:
      if (wristThere()) {
        wristRpm = WRIST_RPM;
        step = Step::Idle;  // done, grabber keeps holding
      } else if (elapsed > 3000) {
        fail();
      }
      break;
    case Step::ScoreDip:
      if (liftThere() || elapsed > 1500) {  // too long = sitting on the goal: fine
        liftTarget = clampLift(liftNow());
        holdingStack = false;
        goStep(Step::ScoreRelease);
      }
      break;
    case Step::ScoreRelease:
      if (elapsed >= SCORE_OUT_FIRST_MS) {
        goStep(Step::ScorePause);
      }
      break;
    case Step::ScorePause:
      if (elapsed >= SCORE_PAUSE_MS) {
        liftTarget = clampLift(liftNow() + SCORE_RAISE_DEG);
        goStep(Step::ScoreRaise);
      }
      break;
    case Step::ScoreRaise:
      if (liftThere() || elapsed > 1500) {
        if (!scoreWristUp) {
          step = Step::Idle;  // done, wrist stays where it is
          break;
        }
        wristTarget = clampWrist(WRIST_SCORE_DEG);
        goStep(Step::ScoreWrist);
      }
      break;
    case Step::ScoreWrist:
      // Done once the wrist is there (or after 1.5 s: stop it where it is).
      if (wristThere() || elapsed > 1500) {
        wristTarget = clampWrist(wristNow());
        step = Step::Idle;
      }
      break;
    case Step::HomeLift:
      if (liftNow() <= LIFT_REST_DEG) {  // down (it rests with the power off from here)
        wristTarget = WRIST_DOWN;
        goStep(Step::HomeWrist);
      } else if (elapsed > 4000) {
        fail();
      }
      break;
    case Step::HomeWrist:
      if (wristThere()) {
        step = Step::Idle;  // back at the start, ready for the next stack
      } else if (elapsed > 3000) {
        fail();
      }
      break;
  }
}

// Intake and grabber speeds for right now.
void runRollers() {
  int intake = 0, grabber = 0;
  if (step == Step::ScoreRelease ||
      (SCORE_SPIN_WHILE_RAISING && (step == Step::ScoreRaise || step == Step::ScoreWrist))) {
    grabber = GRABBER_RELEASE_RPM;
  } else if (holdingStack) {
    grabber = GRABBER_HOLD_RPM;
  } else if (step != Step::Idle && step != Step::HomeLift && step != Step::HomeWrist) {
    // Y or B running: intake buttons do nothing (like the old code).
  } else if (intakeCmd == 1) {
    intake = INTAKE_IN_RPM;
    grabber = GRABBER_IN_RPM;
  } else if (intakeCmd == -1) {
    intake = -INTAKE_IN_RPM;
    grabber = -GRABBER_IN_RPM;
  }
  Intake.move_velocity(intake);
  Grabber.move_velocity(grabber);
}

void task() {
  StallWatch liftStall, wristStall;
  double sentLift = NAN, sentWrist = NAN;
  std::uint32_t manualStuckSince = 0;
  int tempTick = 0;
  while (true) {
    lock.take();
    runSteps();

    // String check: the 2 elevator motors must read about the same.
    gapStop = std::fabs(Lift.get_position(0) - Lift.get_position(1)) > LIFT_GAP_DEG;
    if (gapStop) {
      if (step != Step::Idle) fail();
      liftTarget = clampLift(liftNow());
      alert = true;
    }

    // Stuck: stop pushing, stay here, buzz the controller.
    if (liftStall.check(Lift.get_current_draw(), Lift.get_actual_velocity(),
                        liftTarget - liftNow(), LIFT_TOLERANCE_DEG)) {
      if (step == Step::ScoreDip) {
        // Stack is sitting on the goal: let go right here.
        holdingStack = false;
        goStep(Step::ScoreRelease);
      } else if (step == Step::HomeLift && liftNow() < HOME_BOTTOM_SLACK_DEG) {
        // Hit the bottom a little early (string slack): bring the wrist down.
        wristTarget = WRIST_DOWN;
        goStep(Step::HomeWrist);
      } else {
        if (step != Step::Idle) fail();
        alert = true;
      }
      liftTarget = clampLift(liftNow());
    }
    if (wristStall.check(Wrist.get_current_draw(), Wrist.get_actual_velocity(),
                         wristTarget - wristNow(), WRIST_TOLERANCE_DEG)) {
      if (step != Step::Idle) fail();
      wristTarget = clampWrist(wristNow());
      alert = true;
    }

    // Temperature: the hotter of the 2 elevator motors.
    if (++tempTick >= 50) {  // twice a second is plenty
      tempTick = 0;
      liftTempC = std::max(Lift.get_temperature(0), Lift.get_temperature(1));
    }

    // R1/R2 held: full speed, but never past the top or the bottom.
    int manual = liftManualBlocked || gapStop ? 0 : liftManual;
    if ((manual > 0 && liftNow() >= LIFT_MAX_DEG) || (manual < 0 && liftNow() <= 0)) manual = 0;
    if (manual != 0) {
      if (rest != Rest::No) {
        Lift.set_brake_mode_all(pros::MotorBrake::hold);
        rest = Rest::No;
      }
      // Slow down near the end it's heading for, so it doesn't slam the stop.
      const bool nearEnd = manual > 0 ? liftNow() > LIFT_MAX_DEG - LIFT_SLOW_ZONE_DEG
                                      : liftNow() < LIFT_SLOW_ZONE_DEG;
      Lift.move(manual * (nearEnd ? LIFT_SLOW_VOLTS : LIFT_MANUAL_VOLTS));
      liftTarget = clampLift(liftNow());  // let go = hold right here
      sentLift = NAN;
      // Pushing hard but not moving: stop until the button is let go.
      if (Lift.get_current_draw() > STALL_CURRENT_MA &&
          std::fabs(Lift.get_actual_velocity()) < STALL_RPM) {
        if (manualStuckSince == 0) manualStuckSince = pros::millis();
        if (pros::millis() - manualStuckSince >= STALL_MS) {
          liftManualBlocked = true;
          alert = true;
        }
      } else {
        manualStuckSince = 0;
      }
    } else if (liftTarget <= LIFT_REST_DEG && !scoringNow() &&
               (liftNow() <= LIFT_REST_DEG || rest == Rest::Tension || rest == Rest::Done)) {
      // At the bottom: drop onto the stop, wind the string up until it pulls
      // (tight, so it can't come off the pulleys), then stop. No heat.
      manualStuckSince = 0;
      sentLift = NAN;
      const std::uint32_t inPhase = pros::millis() - restSince;
      if (pros::competition::is_disabled()) {
        rest = Rest::No;  // motors can't move while disabled: start over when enabled
      } else if (rest == Rest::No) {
        // BRAKE (not coast): it settles onto the stop without the spool
        // spinning on and paying out string.
        Lift.set_brake_mode_all(pros::MotorBrake::brake);
        Lift.brake();
        rest = Rest::Settle;
        restSince = pros::millis();
      } else if (rest == Rest::Settle && inPhase >= LIFT_SETTLE_MS) {
        Lift.move_voltage(LIFT_TENSION_MV);  // + = up
        rest = Rest::Tension;
        restSince = pros::millis();
      } else if (rest == Rest::Tension) {
        const double current = std::max(Lift.get_current_draw(0), Lift.get_current_draw(1));
        const bool tight = inPhase > 100 && current >= LIFT_TENSION_MA;  // skip the start-up surge
        // Also stop if it starts lifting off the stop, or after 1 s.
        if (tight || liftNow() > LIFT_REST_DEG || inPhase > 1000) {
          Lift.set_brake_mode_all(pros::MotorBrake::brake);  // holds the string, no power
          Lift.brake();
          rest = Rest::Done;
        }
      }
    } else {
      // Hold or go to the target (only tell the motors when it changed).
      manualStuckSince = 0;
      if (rest != Rest::No) {
        Lift.set_brake_mode_all(pros::MotorBrake::hold);
        rest = Rest::No;
      }
      if (liftTarget != sentLift) {
        Lift.move_absolute(liftTarget, LIFT_RPM);
        sentLift = liftTarget;
      }
    }
    if (wristTarget != sentWrist) {
      Wrist.move_absolute(wristTarget * WRIST_GEAR, std::min(wristRpm * WRIST_GEAR, 200.0));
      sentWrist = wristTarget;
    }
    runRollers();
    lock.give();
    pros::delay(10);
  }
}

// Manual buttons win: stop the Y/B steps (but keep holding the stack).
void stopStepsLocked() {
  step = Step::Idle;
  wristRpm = WRIST_RPM;
}

}  // namespace

void init() {
  Lift.set_brake_mode_all(pros::MotorBrake::hold);
  Wrist.set_brake_mode(pros::MotorBrake::hold);
  Intake.set_brake_mode(pros::MotorBrake::brake);
  Grabber.set_brake_mode(pros::MotorBrake::brake);
  Lift.tare_position_all();
  Wrist.tare_position();
  if (MEASURE_MODE) {
    // Measuring build: motors off and loose, so they can be moved by hand.
    // The background loop never starts, so nothing here can move.
    Lift.set_brake_mode_all(pros::MotorBrake::coast);
    Wrist.set_brake_mode(pros::MotorBrake::coast);
    return;
  }
  static pros::Task mechTask(task);
}

// Every function below holds the lock until it ends (std::lock_guard),
// so the background loop never sees a half-changed number.
using Guard = std::lock_guard<pros::Mutex>;

void grab() {
  Guard g(lock);
  if (step == Step::Idle && !holdingStack) {
    liftTarget = clampLift(liftNow() + GRAB_CLEAR_DEG);
    goStep(Step::GrabLift);
  }
}

void grip() {
  Guard g(lock);
  holdingStack = true;
}

void letGo() {
  Guard g(lock);
  holdingStack = false;
}

void score(bool wristUp, double dipDeg) {
  Guard g(lock);
  scoreWristUp = wristUp;
  if (scoringNow()) return;  // B already running
  stopStepsLocked();
  liftTarget = clampLift(liftNow() - dipDeg);
  goStep(Step::ScoreDip);
}

void cancel() {
  Guard g(lock);
  if (step != Step::Idle) {
    // First press: stop the move, stay here, keep holding.
    stopStepsLocked();
    liftTarget = clampLift(liftNow());
    wristTarget = clampWrist(wristNow());
  } else {
    // Nothing running: let go of the stack.
    holdingStack = false;
  }
}

void home() {
  Guard g(lock);
  const bool up = liftTarget > LIFT_TOLERANCE_DEG || wristTarget > WRIST_TOLERANCE_DEG;
  if (step == Step::Idle && !holdingStack && up) {
    liftTarget = 0;
    goStep(Step::HomeLift);
  }
}

void stopSteps() {
  Guard g(lock);
  stopStepsLocked();
  liftManual = 0;  // forget R1/R2 (between auton and driver)
}

void arcade(int throttle, int turn) {
  Guard g(lock);
  chassis.arcade(throttle, turn);
}

void manualLift(int direction) {
  Guard g(lock);
  if (direction == 0) {
    liftManualBlocked = false;
  } else {
    stopStepsLocked();  // the driver's buttons win over B / L1
  }
  liftManual = direction;
}

void liftTo(double deg) {
  Guard g(lock);
  stopStepsLocked();
  liftTarget = clampLift(deg);
}

void nudgeLift(double deg) {
  Guard g(lock);
  stopStepsLocked();
  liftTarget = clampLift(liftTarget + deg);
}

void wristTo(double deg) {
  Guard g(lock);
  stopStepsLocked();
  wristTarget = clampWrist(deg);
}

void nudgeWrist(double deg) {
  Guard g(lock);
  stopStepsLocked();
  wristTarget = clampWrist(wristTarget + deg);
}

void rezero() {
  Guard g(lock);
  stopStepsLocked();
  Lift.tare_position_all();
  Wrist.tare_position();
  liftTarget = 0;
  wristTarget = 0;
}

void setIntake(int direction) {
  Guard g(lock);
  intakeCmd = direction;
}

bool busy() {
  Guard g(lock);
  return step != Step::Idle;
}

bool holding() {
  Guard g(lock);
  return holdingStack;
}

bool waitUntilDone(int timeoutMs) {
  const std::uint32_t start = pros::millis();
  while (true) {
    {
      Guard g(lock);
      if (step == Step::Idle && liftThere() && wristThere()) return true;
    }  // let go of the lock before waiting
    if (pros::millis() - start >= static_cast<std::uint32_t>(timeoutMs)) return false;
    pros::delay(10);
  }
}

double liftDeg() { return liftNow(); }
double wristDeg() { return wristNow(); }

const char* liftStatus() {
  Guard g(lock);
  return gapStop ? "STRING GAP" : liftTempC >= LIFT_HOT_C ? "HOT" : "";
}

double liftCurrent() {
  return std::max(Lift.get_current_draw(0), Lift.get_current_draw(1));
}

double liftTemp() {
  Guard g(lock);
  return liftTempC;
}

bool takeAlert() {
  Guard g(lock);
  const bool v = alert;
  alert = false;
  return v;
}

}  // namespace mech
