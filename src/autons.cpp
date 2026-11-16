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
// SKILLS (60 seconds). The team's plan.
//
//   Part 1: the red auton above (bottom toggle, preload + 2 stacks on the red goal)
//   Part 2: cross the bottom of the field to the loader
//   Part 3: 2 loader stacks onto goal A (24, -48), the goal left of the loader
//   Part 4: 4 loader trips onto goal B (48, -24): a bare pin, then 3 stacks
//   Part 5: 12 in off goal B, up to the top toggle (0, 72), hit it twice,
//           then park with the grabber on the tall goal (0, 0)
//
// Front of the robot = TOGGLE MECH. It faces the loader at every loader stop.
// After every drop the robot drives off and the elevator + wrist go back
// down to the intake position (L1 press).
//
// NOT TESTED. MEASURE = a guess until the team checks it on the robot.
// The drawing of this path is made from these same numbers.
// ============================================================

// ---------- Loader (bottom wall, right side) ----------
constexpr double LOADER_X = 60;         // team: where the front touches the loader
constexpr double LOADER_Y = -64;        // team
constexpr double LOADER_HEADING = 180;  // front (toggle mech) faces the loader
constexpr float LOADER_LEAD = 0.4f;     // how round the curve into the loader is (0.3 tight, 0.6 wide)
constexpr int LOADER_MOVE_MS = 3000;    // time limit for the drive to the loader
constexpr int LOADER_INTAKE_MS = 1500;  // team: intake runs 1.5 s
constexpr double LOADER_BACK_IN = 6;    // team: back off the loader 6 in before turning
constexpr double GOAL_OFF_IN = 6;       // team: drive 6 in off the goal after a drop
constexpr double FINAL_OFF_IN = 12;     // team: 12 in off the last goal, then go to the toggle

// ---------- Goals ----------
constexpr double GOAL_A_X = TILE, GOAL_A_Y = -2 * TILE;  // team said (24, -48)
constexpr double GOAL_B_X = 2 * TILE, GOAL_B_Y = -TILE;  // team said (48, -24)

// Elevator spots for one stack (E on the controller):
//   carry: raise to this while backing onto the goal
//   down:  drop: elevator down to this, grabber lets go
//   up:    then up to this before driving off
// Higher stacks need higher numbers.
struct DropSpots {
  double carry, down, up;
};
constexpr DropSpots GOAL_A_1ST = {700, 610, 800};     // team's numbers
constexpr DropSpots GOAL_A_2ND = {900, 860, 1000};    // team's numbers
constexpr DropSpots GOAL_B_2ND = {700, 610, 800};     // MEASURE: guess (B only has the pin on it)
constexpr DropSpots GOAL_B_3RD = {900, 860, 1000};    // MEASURE: guess
constexpr DropSpots GOAL_B_4TH = {1100, 1060, 1200};  // MEASURE: guess, +200 like A's 1st to 2nd

// ---------- Part 2: crossing to the loader ----------
// Along the bottom: between the goal row (y -48) and the wall stacks (y -69).
constexpr double CROSS_DOWN_X = -44;     // first go down on the left of the red goal (-40 cut the corner into it)
constexpr double CROSS_LANE_Y = -58.6;   // halfway between the goals and the wall stacks
constexpr double CROSS_END_X = 40;       // then curve up into the loader from here
constexpr float CHAIN_MIN_SPEED = 50;    // 0-127: keeps rolling through a corner instead of stopping
constexpr float CHAIN_EXIT_IN = 6;       // hand off to the next move this far before the corner

// ---------- Part 5: to the top toggle, then the tall goal ----------
// Up the right side, across above the blue pin (24, 0), then up the middle.
constexpr double RIGHT_LANE_X = 58.6;   // halfway between goal B and the wall stack (69, -24)
constexpr double ACROSS_Y = 11.4;       // halfway between the blue pin (24, 0) and the stack (24, 24)
constexpr double UP_LANE_X = 11.4;      // halfway between the blue pin (0, 24) and the stack (24, 24)
constexpr double TOP_TOGGLE_X = 0;
constexpr double TOP_TOGGLE_Y = -START_Y;  // same distance from the top toggle as the start is from the bottom one
constexpr double PARK_TURN_Y = 14;      // turn the grabber toward the tall goal here, on UP_LANE_X
constexpr double TALL_X = 0, TALL_Y = 0;

// ---------- Clock ----------
constexpr bool CLOCK_CHECK = true;      // false = always do every loader trip
constexpr int SKILLS_MS = 59500;        // everything must be done by here
constexpr int LOADER_TRIP_MS = 11000;   // MEASURE: one loader trip (time it on the robot)
constexpr int FINISH_MS = 16000;        // MEASURE: part 5 (time it on the robot)

std::uint32_t skillsStart = 0;  // 0 = not running Skills: ALL (testing a part)

// Is there time for this much more, plus part 5?
bool timeFor(int ms) {
  if (!CLOCK_CHECK || skillsStart == 0) return true;
  return static_cast<int>(pros::millis() - skillsStart) + ms + FINISH_MS <= SKILLS_MS;
}

// Drive through a point without stopping (hands off to the next move).
void passThrough(double x, double y, bool forwards = true) {
  chassis.moveToPoint(x, y, 3000,
                      {.forwards = forwards, .maxSpeed = speed(AUTON_PCT),
                       .minSpeed = CHAIN_MIN_SPEED, .earlyExitRange = CHAIN_EXIT_IN},
                      false);
}

// Curve to the loader so the front (toggle mech) faces it, square.
// The elevator and wrist go down on the way.
void toLoader() {
  mech::home();
  chassis.moveToPose(LOADER_X, LOADER_Y + FRONT_OFFSET_IN, LOADER_HEADING, LOADER_MOVE_MS,
                     {.lead = LOADER_LEAD, .maxSpeed = speed(AUTON_PCT)}, false);
}

// At the loader: intake 1.5 s, grab it (same as Y), back off 6 in.
void loaderPickup() {
  mech::waitUntilDone(2000);  // elevator and wrist down first
  mech::setIntake(1);
  pros::delay(LOADER_INTAKE_MS);
  mech::setIntake(0);
  mech::grab();  // Y: elevator up a little, grip, wrist up (runs while it backs off)
  drive(-LOADER_BACK_IN, AUTON_PCT);
}

// Drop: elevator down to spots.down, grabber lets go, then up to spots.up.
void drop(const DropSpots& spots) {
  mech::score(false, mech::liftDeg() - spots.down);
  mech::waitUntilDone(3000);
  mech::liftTo(spots.up);
  mech::waitUntilDone(2000);
}

// Drive off the goal, elevator and wrist back down to the intake position.
void offGoal(double inches) {
  drive(inches, AUTON_PCT);
  mech::home();
}

// One loader trip with a stack (pin + cup): loader -> goal.
void loaderStack(double goalX, double goalY, const DropSpots& spots) {
  loaderPickup();
  turnBackTo(goalX, goalY, AUTON_PCT);
  mech::waitUntilDone(3000);  // Y done: holding it, wrist up
  mech::liftTo(spots.carry);  // going up while it backs onto the goal
  backUpTo(goalX, goalY, AUTON_PCT);
  mech::waitUntilDone(2000);
  drop(spots);
  offGoal(GOAL_OFF_IN);
}

// One loader trip with a single bare pin, dropped like the preload.
void loaderPin(double goalX, double goalY) {
  loaderPickup();
  turnBackTo(goalX, goalY, AUTON_PCT);
  mech::waitUntilDone(3000);
  backUpTo(goalX, goalY, AUTON_PCT, PRELOAD_BACK_EXTRA_IN);
  mech::score(false, PRELOAD_DIP_DEG);  // same as the preload
  mech::waitUntilDone(3000);
  offGoal(GOAL_OFF_IN);
}

// ---------- The parts ----------

void crossToLoader() {
  mech::home();
  passThrough(CROSS_DOWN_X, CROSS_LANE_Y);
  passThrough(CROSS_END_X, CROSS_LANE_Y);
  toLoader();
}

// Returns false when the clock says stop and go to part 5.
bool goalATrips() {
  loaderStack(GOAL_A_X, GOAL_A_Y, GOAL_A_1ST);
  if (!timeFor(LOADER_TRIP_MS)) return false;
  toLoader();
  loaderStack(GOAL_A_X, GOAL_A_Y, GOAL_A_2ND);
  return true;
}

bool goalBTrips() {
  const DropSpots stacks[] = {GOAL_B_2ND, GOAL_B_3RD, GOAL_B_4TH};
  if (!timeFor(LOADER_TRIP_MS)) return false;
  toLoader();
  loaderPin(GOAL_B_X, GOAL_B_Y);
  for (const DropSpots& spots : stacks) {
    if (!timeFor(LOADER_TRIP_MS)) return false;
    toLoader();
    loaderStack(GOAL_B_X, GOAL_B_Y, spots);
  }
  return true;
}

void finish() {
  offGoal(FINAL_OFF_IN - GOAL_OFF_IN);  // already 6 in off after the drop: 12 in total
  passThrough(RIGHT_LANE_X, ACROSS_Y);
  passThrough(UP_LANE_X, ACROSS_Y);
  passThrough(UP_LANE_X, TOP_TOGGLE_Y - 2 * TOGGLE_BACK_IN);
  chassis.moveToPose(TOP_TOGGLE_X, TOP_TOGGLE_Y, 0, 2000, {.lead = 0.3f, .maxSpeed = speed(AUTON_PCT)},
                     false);
  // Top toggle: two hits, like the start.
  for (int hit = 1; hit <= 2; hit++) {
    ram(TOGGLE_RAM_IN);
    drive(-TOGGLE_BACK_IN, AUTON_PCT);
  }
  // Back down the middle lane, then the grabber onto the tall goal (parked in the midfield).
  passThrough(UP_LANE_X, TOP_TOGGLE_Y - 2 * TOGGLE_BACK_IN, false);  // sideways first: the blue pin (0, 24) is straight behind
  goTo(UP_LANE_X, PARK_TURN_Y, false, AUTON_PCT, 3000);
  grabberTo(TALL_X, TALL_Y, AUTON_PCT);
}

// Start spots for testing a part by itself (read off the drawing).
constexpr double PART2_START_X = -33.8, PART2_START_Y = -38.5, PART2_START_H = 312;  // where part 1 ends
constexpr double PART5_START_X = 54.8, PART5_START_Y = -38.1, PART5_START_H = 153;   // 6 in off goal B after the last drop

// Start: where part 1 ends (red goal behind the robot, 3 on it).
void skillsCross() {
  startAt(PART2_START_X, PART2_START_Y, PART2_START_H);
  crossToLoader();
}

// Start: front (toggle mech) touching the loader, square.
void skillsGoalA() {
  startAt(LOADER_X, LOADER_Y + FRONT_OFFSET_IN, LOADER_HEADING);
  goalATrips();
}

// Start: front touching the loader, square. Goal B empty.
void skillsGoalB() {
  startAt(LOADER_X, LOADER_Y + FRONT_OFFSET_IN, LOADER_HEADING);
  // goalBTrips() starts with a drive to the loader; already there, so it just squares up.
  goalBTrips();
}

// Start: 6 in off goal B, pointing away from it (where the last drop ends).
void skillsFinish() {
  startAt(PART5_START_X, PART5_START_Y, PART5_START_H);
  finish();
}

// Everything. Start: same as the red auton.
void skillsAll() {
  skillsStart = pros::millis();
  togglePreload();
  twoStacks();
  crossToLoader();
  if (goalATrips()) goalBTrips();
  finish();
  skillsStart = 0;
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
    {"Skills 1: red auton", toggleStacks},
    {"Skills 2: cross to loader", skillsCross},
    {"Skills 3: goal A x2", skillsGoalA},
    {"Skills 4: goal B x4", skillsGoalB},
    {"Skills 5: toggle + park", skillsFinish},
    {"Tune: drive 24in", tuneDrive},
    {"Tune: turn 90", tuneTurn},
};
const int AUTON_COUNT = sizeof(AUTONS) / sizeof(AUTONS[0]);
