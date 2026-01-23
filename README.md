# 2026-rebuilt-robot

## Notes

We need to replace [TunerConstants.java](/src/main/java/frc/robot/generated/TunerConstants.java) with generated code once the robot is built.

### RotateToHub

Will need to go back and check the PID values and max speed.

The center hub pose in RotateToHub is off by an insignificant amount.

Take into account ball trajectory while robot is moving because robot will give ball the same velocity as the direction the robot is moving.

### Dashboard

Figure out settings on dashboard for quickly changing settings without needed to redeploy.

### LIST OF POSSIBLE THINGS TO WORK ON

1. Start planning and writing pseudo-code for the subsystems that will be on the robot for this year, like intake and shooting mechanism
2. Start mapping out potential autos or commands that would be used in autonomous
3. Make a method that calculates distance between hub and robot for knowing whether the robot is within shooting range, use LED lights or put on shuffleboard
4. Figure out how to add configurable values in Glass like Shuffleboard.

Check if height of robot to camera offset matters for Photon Vision.

POTENTIAL BUTTON BINDINGS
Left Joystick - Translation
Right Joystick - Rotation

Left Trigger - Intake (activate intake)
Right Trigger - Shoot (activate flywheel, kicker, hopper)

A - Automatic Rotate to Hub

Right Bumper - Bring Intake Down
Left Bumper - Raise Intake Up

## Team Documentation

<https://github.com/Wildcat5e/documentation>
