# 839X (Caution Tape Xcalibur)

Robot code for 839X. PROS 4 + LemLib. Game: Override.

## Where things are

| File | What's in it |
|---|---|
| `include/robot_config.hpp` | All the numbers: score heights, wrist angles, speeds, start spot |
| `include/build.hpp` | The build number shown on the brain and controller |
| `src/robot_config.cpp` | Ports and drive tuning (kP, kD) |
| `src/autons.cpp` | The autons and the tuning tests. Add new ones here |
| `src/mechanisms.cpp` | How the elevator, wrist, intake and grabber work (Y and B steps) |
| `src/main.cpp` | Driver controls (list at the top) |

## Every time you turn the robot on

- Elevator all the way down, wrist down.
- Put it on the auton start spot.
- Keep it still while it calibrates.
- Check the build number on the brain screen.

## Autons

1. **Red: toggle + 3 stacks**: bottom toggle, preload, then two field stacks on the red goal
2. **Tune: drive 24in**: drive tuning test, result on the brain screen
3. **Tune: turn 90**: turn tuning test, result on the brain screen

Tap the brain screen to pick one. Press X to test it (not on a field).

## Upload

`git pull`, then `pros mu` in the VS Code PROS terminal, or the PROS Build & Upload button. Slot 1.
