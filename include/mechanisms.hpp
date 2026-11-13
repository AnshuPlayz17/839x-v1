#pragma once

#include "robot_config.hpp"

// Elevator, wrist, intake and grabber. How they work is in mechanisms.cpp.
namespace mech {

void init();  // call once when the robot turns on (elevator and wrist down)

// ---------- Y and B ----------
void grab();    // Y: elevator up a little, grip, wrist up
// B: elevator down by dipDeg, grabber spins out, elevator up a little.
// wristUp = true: then the wrist goes up all the way (autons use false).
void score(bool wristUp = true, double dipDeg = STACK_DROP_DIP_DEG);
void cancel();  // A: stop a Y / B move. Nothing running: let go of the stack
void home();    // L1: elevator, then wrist, back down to the start
void stopSteps();  // stop a Y / B move, keep holding

// ---------- Grabber ----------
void grip();   // grabber holds (spins in slowly)
void letGo();  // grabber stops holding
void setIntake(int direction);  // 1 in, -1 out, 0 stop (intake + grabber)

// ---------- Elevator and wrist ----------
void manualLift(int direction);  // R1/R2 held: 1 up, -1 down, 0 none
void liftTo(double deg);         // elevator to a spot (E on the controller)
void nudgeLift(double deg);      // elevator up/down a little
void wristTo(double deg);        // wrist to an angle (W on the controller)
void nudgeWrist(double deg);     // wrist up/down a little
void rezero();                   // where they are now becomes 0 (both down first!)

void arcade(int throttle, int turn);

// ---------- Checks ----------
bool busy();     // a Y / B / L1 move is running
bool holding();  // holding a stack
// Waits until the move is done. false = time ran out.
bool waitUntilDone(int timeoutMs = 3000);
double liftDeg();
double wristDeg();
const char* liftStatus();  // "STRING GAP", "HOT" or ""
double liftCurrent();      // mA
double liftTemp();         // deg C
bool takeAlert();          // true once when something went wrong

}  // namespace mech
