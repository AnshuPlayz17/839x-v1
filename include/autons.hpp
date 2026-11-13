#pragma once

// One auton: the name shown on the brain screen, and the function that runs it.
struct Auton {
  const char* name;
  void (*run)();
};

// The auton list is at the bottom of autons.cpp.
extern const Auton AUTONS[];
extern const int AUTON_COUNT;
