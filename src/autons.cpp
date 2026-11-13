#include "autons.hpp"

#include <cmath>

#include "mechanisms.hpp"
#include "robot_config.hpp"

// ============================================================
// AUTONS
//
// The field, standing at your driver station (inches):
//   0, 0 = middle of the field. Walls are at -72 and 72.
//   X = right (+) / left (-).   Y = away from you (+) / toward you (-).
//   Heading: 0 = facing away, 90 = right, 180 = toward you, 270 = left.
// Front of the robot = TOGGLE MECH. Back = GRABBER.
// Grabber facing a point = back faces it, front faces the opposite way.
//
// To make an auton: drive the path, write down X, Y and heading from the
// controller at each stop, then use these:
//   goTo(x, y)              drive there intake first
//   goTo(x, y, false)       drive there grabber first (backwards)
//   turnTo(heading)         turn to face a heading
//   turnToFace(x, y)        turn the intake toward a point
//   turnBackTo(x, y)        turn the GRABBER toward a point
//   grabberTo(x, y)         put the GRABBER over a goal or stack (type its field spot)
//   mech::grab()            Y: grab the stack (then mech::waitUntilDone())
//   mech::score()           B: score the stack (then mech::waitUntilDone())
//   mech::setIntake(1)      intake on (0 = off, -1 = out)
//   mech::home()            L1: elevator down, then wrist down
//   mech::wristTo(90)       wrist to 90 (read W on the controller)
//   pros::delay(500)        wait 500 milliseconds (half a second)
//   mech::liftTo(500)       elevator to 500 (read E on the controller)
//   mech::...               more in mechanisms.hpp
// The last number on goTo/turnTo is speed in percent, like goTo(0, -48, true, 60).
// Then add your auton to the list at the bottom of this file.
// ============================================================
namespace {

void startAt(double x, double y, double heading) { chassis.setPose(x, y, heading); }

// Changes 0-100 percent to LemLib speed (0-127).
float speed(double pct) { return static_cast<float>(pct * 1.27); }

// Drive to a point. Waits until it gets there (or time runs out).
void goTo(double x, double y, bool forwards = true, double maxPct = 100, int timeoutMs = 2000) {
  chassis.moveToPoint(x, y, timeoutMs, {.forwards = forwards, .maxSpeed = speed(maxPct)}, false);
}

// Turn on the spot to a heading.
void turnTo(double heading, double maxPct = 100, int timeoutMs = 1000) {
  chassis.turnToHeading(heading, timeoutMs, {.maxSpeed = static_cast<int>(speed(maxPct))}, false);
}

// Turn the intake toward a point.
void turnToFace(double x, double y, double maxPct = 100, int timeoutMs = 1000) {
  chassis.turnToPoint(x, y, timeoutMs, {.maxSpeed = static_cast<int>(speed(maxPct))}, false);
}

// Turn the back (grabber) toward a point.
void turnBackTo(double x, double y, double maxPct = 100, int timeoutMs = 1000) {
  chassis.turnToPoint(x, y, timeoutMs,
                      {.forwards = false, .maxSpeed = static_cast<int>(speed(maxPct))}, false);
}

// Drive straight forward (or backwards if negative) from where the robot
// REALLY is, the way it REALLY faces. Aims straight ahead, so no curving.
void drive(double inches, double maxPct = 100, int timeoutMs = 2000) {
  const lemlib::Pose now = chassis.getPose();
  const double rad = now.theta * M_PI / 180.0;
  chassis.moveToPoint(now.x + inches * std::sin(rad), now.y + inches * std::cos(rad), timeoutMs,
                      {.forwards = inches > 0, .maxSpeed = speed(maxPct)}, false);
}

// Put the GRABBER (not the robot center) over a field point: a goal or a stack.
// Turns the back toward it, then backs up straight until the grabber is on it.
// Back up straight (no turn) until the grabber reaches a point's distance.
// extraIn: back up this much further (it was stopping short).
void backUpTo(double x, double y, double maxPct = 100, double extraIn = 0) {
  const lemlib::Pose now = chassis.getPose();
  const double dist = std::hypot(x - now.x, y - now.y);
  // Negative = back up. Closer than the offset already: pulls forward instead.
  drive(GRABBER_OFFSET_IN - dist - extraIn, maxPct);
}

void grabberTo(double x, double y, double maxPct = 100, double extraIn = 0) {
  turnBackTo(x, y, maxPct);
  backUpTo(x, y, maxPct, extraIn);
}

// ============================================================
// AUTONS
// ============================================================

// Auton speed in percent. Go faster once it works every time.
constexpr double AUTON_PCT = 60;

// Elevator up high enough to pass over 2 standing stacks.
void liftClear() {
  mech::liftTo(STACK_CLEAR_DEG);
  mech::waitUntilDone(2000);
}

// Auton grab for a standing stack. The grabber must already be over it with
// the elevator up (liftClear). Wrist stays where Y puts it (WRIST_VERTICAL):
// grabber pulling in, elevator down onto it, grip, back up.
void fieldGrab(double grabDeg = STACK_GRAB_DEG) {
  mech::letGo();
  mech::wristTo(WRIST_VERTICAL);  // already there after Y; makes sure
  mech::setIntake(1);  // grabber pulls in (the intake spins too)
  mech::liftTo(grabDeg);
  mech::waitUntilDone(1500);
  pros::delay(GRAB_GRIP_MS);
  mech::grip();
  mech::setIntake(0);
  liftClear();
}

// Red side. Field spots (where the GRABBER goes):
// One field tile. Was 23.54 (spots were -23.54 and -47.09); changed to 23.8,
// the real tile size, so every spot below moved out by 0.26 to 0.51 in.
constexpr double TILE = 23.8;
constexpr double GOAL_X = -TILE, GOAL_Y = -2 * TILE;        // red goal
constexpr double UPPER_X = -TILE, UPPER_Y = -TILE;          // upper standing stack
constexpr double LOWER_X = -2 * TILE, LOWER_Y = -2 * TILE;  // lower standing stack
constexpr double TOGGLE_RAM_IN = 14;   // MEASURE: aims past the toggle, so it really hits it
constexpr double TOGGLE_BACK_IN = 10;  // MEASURE: back off this far between hits
constexpr double PRELOAD_HEADING = 89; // MEASURE: front heading for the preload drop (smaller = turns more)
constexpr double PRELOAD_TURN_Y = -2 * TILE - 2;  // MEASURE: back up to here before the preload turn, 2 in past the goal row (was -49.09 with the old tile size)
constexpr double UPPER_FIX_IN = 2;     // MEASURE: ended right of UPPER, so aim this far left (turns a bit less)
constexpr double UPPER_BACK_EXTRA_IN = 7;  // MEASURE: UPPER to the goal stopped short, so back up this much more
constexpr double UPPER_GRAB_DEG = 0;       // MEASURE: elevator lower for UPPER (50 didn't grab it)
constexpr double LOWER_GRAB_DEG = 25;      // MEASURE: elevator lower for LOWER
constexpr double LOWER_BACK_EXTRA_IN = 1;  // MEASURE: LOWER to the goal: back up this much more
constexpr double PRELOAD_DIP_DEG = 90;  // MEASURE: elevator down this much before letting go of the preload (B uses 30)
constexpr double PRELOAD_BACK_EXTRA_IN = 2;  // MEASURE: preload drop stopped short, so back up this much more

// Drive into something (the toggle). It stops when it hits; 1 s max.
void ram(double inches) { drive(inches, AUTON_PCT, 1000); }

// Score: elevator down (dipDeg), spin out, back up. Wrist stays.
// Then lift the empty grabber clear.
void scoreAndClear(double dipDeg = STACK_DROP_DIP_DEG) {
  mech::score(false, dipDeg);
  mech::waitUntilDone(3000);
  liftClear();
}

constexpr double PRELOAD_CLEAR_IN = 8;  // drive this far off the goal after the preload (was a plain 8)

// First half: toggle, then the preload on the red goal.
// Every move is stop, turn, then drive straight.
void togglePreload() {
  // Front (toggle) facing the toggle on the south wall.
  startAt(START_X, START_Y, START_HEADING);  // 0, -62.7, 180

  // Toggle: two hits. Ram it, back off.
  for (int hit = 1; hit <= 2; hit++) {
    ram(TOGGLE_RAM_IN);
    drive(-TOGGLE_BACK_IN, AUTON_PCT);
  }

  mech::grab();  // preload to scoring position (same as Y)
  goTo(0, PRELOAD_TURN_Y, false, AUTON_PCT);  // reverse clear of the wall
  mech::waitUntilDone(4000);

  // Preload.
  turnTo(PRELOAD_HEADING, AUTON_PCT);  // one turn, then straight back to the goal
  backUpTo(GOAL_X, GOAL_Y, AUTON_PCT, PRELOAD_BACK_EXTRA_IN);
  scoreAndClear(PRELOAD_DIP_DEG);  // preload: elevator down a bit, then let go
  drive(PRELOAD_CLEAR_IN, AUTON_PCT);  // clear of the scored stack
}

// Second half: the UPPER and LOWER stacks onto the red goal.
// Starts where togglePreload() ends.
void twoStacks() {
  // UPPER stack.
  grabberTo(UPPER_X - UPPER_FIX_IN, UPPER_Y, AUTON_PCT);
  fieldGrab(UPPER_GRAB_DEG);
  drive(6, AUTON_PCT);
  grabberTo(GOAL_X, GOAL_Y, AUTON_PCT, UPPER_BACK_EXTRA_IN);
  scoreAndClear();
  drive(8, AUTON_PCT);

  // Drive the center onto the now-empty UPPER spot, out of the goal's way.
  turnToFace(UPPER_X, UPPER_Y, AUTON_PCT);
  goTo(UPPER_X, UPPER_Y, true, AUTON_PCT);

  // LOWER stack.
  grabberTo(LOWER_X, LOWER_Y, AUTON_PCT);
  fieldGrab(LOWER_GRAB_DEG);
  drive(6, AUTON_PCT);
  grabberTo(GOAL_X, GOAL_Y, AUTON_PCT, LOWER_BACK_EXTRA_IN);
  scoreAndClear();
  drive(4, AUTON_PCT);  // clear, then stop
}

// Toggle, score the preload, then the UPPER and LOWER stacks, all on the red goal.
void toggleStacks() {
  togglePreload();
  twoStacks();
}

// ============================================================
// SKILLS (60 seconds)
//
// Parts 1 and 2 are the red auton above. Then loader trips:
//   Part 3: loader -> pin + cup onto the red goal (its 4th, from max height)
//   Part 4: loader -> bare pin onto the tall goal, and park there
// Each part is in the auton list so it can be tested by itself.
// "Skills: ALL" runs them in order and skips part 3 if time is short.
//
// NOT TESTED. Every number marked MEASURE is a guess until the team measures it.
// ============================================================

// ---------- Loader (bottom-left, on the left wall) ----------
constexpr double LOADER_X = -66;      // MEASURE: the loader's face
constexpr double LOADER_Y = -59.7;    // MEASURE
constexpr double LOADER_HEADING = 270;  // front faces the left wall
// Where the robot's CENTER sits when its front touches the loader.
constexpr double LOADER_SPOT_X = LOADER_X + FRONT_OFFSET_IN;
constexpr double LOADER_STANDOFF_IN = 4;  // curve to this far short of the loader, then creep in
constexpr double LOADER_LEAD = 0.4;   // how round the curve into the loader is (0.3 tight - 0.6 wide)
constexpr double LOADER_CREEP_PCT = 40;  // speed for the last few inches into the loader
constexpr int LOADER_WAIT_MS = 2000;  // time for the loader person to put the pieces in
// Back out this far before turning: at the loader the robot's corner would
// hit the bottom wall if it turned on the spot.
constexpr double LOADER_BACK_IN = 3;  // MEASURE

// ---------- Tall goal ----------
constexpr double TALL_X = 0, TALL_Y = 0;
// Drive (grabber first) to here before backing onto the tall goal. It is on the
// diagonal, halfway between the two red goals, 7.7 in clear of each for an 18 in robot.
constexpr double DIAGONAL_X = -1.5 * TILE, DIAGONAL_Y = -1.5 * TILE;

// ---------- Clock ----------
constexpr int SKILLS_MS = 59500;     // stop starting new things after this
constexpr int PART3_NEEDS_MS = 10000;  // MEASURE: time part 3 takes (time it on the robot)
constexpr int PART4_NEEDS_MS = 11000;  // MEASURE: time part 4 takes
std::uint32_t skillsStart = 0;
int skillsElapsed() { return static_cast<int>(pros::millis() - skillsStart); }

// Curve to the loader and push the front against it.
void toLoader() {
  mech::home();  // elevator, then wrist, back down on the way
  chassis.moveToPose(LOADER_SPOT_X + LOADER_STANDOFF_IN, LOADER_Y, LOADER_HEADING, 3000,
                     {.lead = static_cast<float>(LOADER_LEAD), .maxSpeed = speed(AUTON_PCT)}, false);
  drive(LOADER_STANDOFF_IN + 1, LOADER_CREEP_PCT, 800);  // creep in until it touches
}

// At the loader: take the pieces in, then grab them (same as holding L1, then Y).
// CHECK: this guesses that loader pieces come in through the front intake.
void loaderPickup() {
  mech::waitUntilDone(2000);  // elevator and wrist down first
  mech::setIntake(1);
  pros::delay(LOADER_WAIT_MS);
  mech::setIntake(0);
  mech::grab();
  mech::waitUntilDone(4000);
}

// Part 3: loader -> red goal, straight back about 25 in.
// Load a pin, then a cup. The red goal must already have 3 on it.
void loaderToGoal() {
  loaderPickup();
  drive(-LOADER_BACK_IN, AUTON_PCT);
  mech::liftTo(SCORE_MAX_DEG);  // going up while it turns and backs in
  grabberTo(GOAL_X, GOAL_Y, AUTON_PCT);
  mech::waitUntilDone(2000);
  scoreAndClear();
  drive(4, AUTON_PCT);
}

// Part 4: loader -> tall goal, and stay there (parked in the midfield).
// Load one bare red/yellow pin.
void loaderToTall() {
  loaderPickup();
  drive(-LOADER_BACK_IN, AUTON_PCT);
  mech::liftTo(SCORE_MID_DEG);
  turnBackTo(DIAGONAL_X, DIAGONAL_Y, AUTON_PCT);
  goTo(DIAGONAL_X, DIAGONAL_Y, false, AUTON_PCT, 3000);
  grabberTo(TALL_X, TALL_Y, AUTON_PCT);
  mech::waitUntilDone(2000);
  mech::score(false, STACK_DROP_DIP_DEG);
  mech::waitUntilDone(3000);
  // No driving after this: the robot has to end in the midfield.
}

// ---------- The parts, each with its own start spot ----------

// Start: front on the bottom toggle, preload in the grabber.
void skillsPart1() { togglePreload(); }

// Start: where part 1 ends. Back toward the red goal (preload on it), facing right.
void skillsPart2() {
  startAt(GOAL_X + GRABBER_OFFSET_IN - PRELOAD_BACK_EXTRA_IN + PRELOAD_CLEAR_IN, PRELOAD_TURN_Y,
          PRELOAD_HEADING);
  twoStacks();
}

// Start: front touching the loader. The LOWER stack must be gone.
void skillsPart3() {
  startAt(LOADER_SPOT_X, LOADER_Y, LOADER_HEADING);
  loaderToGoal();
}

// Start: front touching the loader.
void skillsPart4() {
  startAt(LOADER_SPOT_X, LOADER_Y, LOADER_HEADING);
  loaderToTall();
}

// Everything. Start: same as part 1.
void skillsAll() {
  skillsStart = pros::millis();
  togglePreload();
  twoStacks();
  toLoader();
  // Part 3 is optional: only if there is time for it AND for part 4.
  if (skillsElapsed() + PART3_NEEDS_MS + PART4_NEEDS_MS < SKILLS_MS) {
    loaderToGoal();
    toLoader();
  }
  loaderToTall();
}

// ============================================================
// TUNING TESTS (not for matches)
// Pick one on the brain screen, put the robot on open floor, press X.
// The result shows on the bottom of the brain screen.
// ============================================================

constexpr double TUNE_DRIVE_IN = 24;  // how far the drive test goes
constexpr double TUNE_TURN_DEG = 90;  // how far the turn test turns

// Drive straight 24 in. Measure the real distance with a tape measure too.
//   Goes past, then comes back   -> raise the drive kD (robot_config.cpp)
//   Stops short                   -> raise the drive kP
//   ROBOT SAYS is right but the tape is not -> wheel size or tracking wheel is wrong
void tuneDrive() {
  startAt(0, 0, 0);
  const std::uint32_t start = pros::millis();
  drive(TUNE_DRIVE_IN, 100, 4000);
  const std::uint32_t ms = pros::millis() - start;
  pros::delay(500);  // let it settle before reading
  const lemlib::Pose end = chassis.getPose();
  pros::screen::print(pros::E_TEXT_MEDIUM, 8, "DRIVE %2.0f in: ROBOT SAYS %5.1f  SIDE %5.1f   ",
                      TUNE_DRIVE_IN, end.y, end.x);
  pros::screen::print(pros::E_TEXT_MEDIUM, 9, "TURNED %5.1f deg   TOOK %4.2f s   ", end.theta,
                      ms / 1000.0);
}

// Turn 90 degrees to the right.
//   Goes past, then comes back   -> raise the turn kD
//   Stops short                   -> raise the turn kP
void tuneTurn() {
  startAt(0, 0, 0);
  const std::uint32_t start = pros::millis();
  turnTo(TUNE_TURN_DEG, 100, 3000);
  const std::uint32_t ms = pros::millis() - start;
  pros::delay(500);  // let it settle before reading
  const lemlib::Pose end = chassis.getPose();
  pros::screen::print(pros::E_TEXT_MEDIUM, 8, "TURN %2.0f deg: ROBOT SAYS %5.1f  OFF BY %5.1f   ",
                      TUNE_TURN_DEG, end.theta, end.theta - TUNE_TURN_DEG);
  pros::screen::print(pros::E_TEXT_MEDIUM, 9, "TOOK %4.2f s                          ", ms / 1000.0);
}

}  // namespace

// The auton list. The FIRST one is picked when the robot turns on.
// Tap the brain screen to pick another.
const Auton AUTONS[] = {
    {"Red: toggle + 3 stacks", toggleStacks},
    {"Skills: ALL", skillsAll},
    {"Skills 1: toggle+preload", skillsPart1},
    {"Skills 2: two stacks", skillsPart2},
    {"Skills 3: loader>goal", skillsPart3},
    {"Skills 4: tall goal+park", skillsPart4},
    {"Tune: drive 24in", tuneDrive},
    {"Tune: turn 90", tuneTurn},
};
const int AUTON_COUNT = sizeof(AUTONS) / sizeof(AUTONS[0]);
