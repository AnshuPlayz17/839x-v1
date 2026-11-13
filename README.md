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

## Measuring build

`MEASURE_MODE` in `include/robot_config.hpp` is `true` right now: the elevator, wrist, intake and grabber motors stay off and autons do not run. The brain screen shows the numbers to write down. Set it to `false` only after every `GUESS` in that file has a real number.

## Autons

1. **Red: toggle + 3 stacks**: bottom toggle, preload, then two field stacks on the red goal
2. **Skills: ALL**: the 60 second skills run (not tested)
3. **Skills 1 to 4**: the skills run in parts, each with its own start spot (see `src/autons.cpp`)
4. **Tune: drive 24in**: drive tuning test, result on the brain screen
5. **Tune: turn 90**: turn tuning test, result on the brain screen

Tap the brain screen to pick one. Press X to test it (not on a field).

## Upload

`git pull`, then `pros mu` in the VS Code PROS terminal, or the PROS Build & Upload button. Slot 1.
