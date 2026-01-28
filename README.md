# 2026-rebuilt-robot

## Notes

This is a great repo.

## To-Do

- Field Orientation
  - Fix field orientation controls which are based on alliance. Sometimes driving is inverted.
- PID For Flywheel Motors
  - We need to add some kind of PID for flywheel motors as the speed of the motors are inconsistent when fuel goes through flywheel.

- PathPlanner
  - Test and expeirment with paths on the fly and running paths in teleop
  - Make and test with autonomous paths
  - Configure max velocity and acceleration
  - Before competition, make sure to actually set the correct values for settings that are used in pathplanner (like robot weight) or else we cooked.

- RotateToHub
  - FIX SPEED OF ROTATION IN DIFFERENT DIRECTIONS !!!
  - Will need to go back and check the PID values and max speed.
  - The center hub pose in RotateToHub is off by an insignificant amount.
  - Take into account ball trajectory while robot is moving because robot will give ball the same velocity as the direction the robot is moving.

- Check if height of robot to camera offset matters for Photon Vision.

- We need to replace [TunerConstants.java](/src/main/java/frc/robot/generated/TunerConstants.java) with generated code once the robot is built.

## POTENTIAL BUTTON BINDINGS

Left Joystick - Translation
Right Joystick - Rotation

Left Trigger - Intake (activate intake)
Right Trigger - Shoot (activate flywheel, kicker, hopper)

A - Automatic Rotate to Hub

Right Bumper - Bring Intake Down
Left Bumper - Raise Intake Up

## Team Documentation

<https://github.com/Wildcat5e/documentation>
