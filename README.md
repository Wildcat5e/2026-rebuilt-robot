# 2026-rebuilt-robot

## Notes

We need to replace [TunerConstants.java](/src/main/java/frc/robot/generated/TunerConstants.java) with generated code once the robot is built.

The applyDeadzone function in the alternative-deadzones branch doesn't do constant folding because any value can be a parameter.

We may want to disable controller rotation while rotating to hub.

Figure out settings on dashboard for quickly changing settings without needed to redeploy.

- [ ] Consider handling divide by zero with arctan in rotate to hub.

LIST OF POSSIBLE THINGS TO WORK ON:

1. Start planning and writing pseudo-code for the subsystems that will be on the robot for this year, like intake and shooting mechanism
2. Start mapping out potential autos or commands that would be used in autonomous
3. Make a method that calculates distance between hub and robot for knowing whether the robot is within shooting range, use LED lights or put on shuffleboard
4. Figure out how to add configurable values in Glass like Shuffleboard.

## Team Documentation

<https://github.com/Wildcat5e/documentation>
