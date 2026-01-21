# 2026-rebuilt-robot

## Notes

We need to replace [TunerConstants.java](/src/main/java/frc/robot/generated/TunerConstants.java) with generated code once the robot is built.

The applyDeadzone function in the alternative-deadzones branch doesn't do constant folding because any value can be a parameter.

We may want to disable controller rotation while rotating to hub.

Figure out settings on dashboard for quickly changing settings without needed to redeploy.

Consider handling divide by zero with arctan in rotate to hub.

## Team Documentation

<https://github.com/Wildcat5e/documentation>
