# 2026-rebuilt-robot

## Notes

This is a great repo.

## To-Do

- PID For Flywheel Motors
    - We need to add some kind of PID for flywheel motors as the speed of the motors are inconsistent when fuel goes through flywheel.

- PathPlanner
    - Figure out how to disable rotation on pathplanner auto for rotate to hub command to work.
    - Test and experiment with paths on the fly and running paths in teleop
    - Make and test with autonomous paths
    - Configure max velocity and acceleration
    - Before competition, make sure to actually set the correct values for settings that are used in pathplanner (like robot weight) or else we cooked.

- RotateToHub/Shooting While Moving
    - Will need to go back and check the PID values and max speed.
    - Check math in [ShootingCalculator](/src/main/java/frc/robot/subsystems/ShootingCalculator.java) (can we test in the WPILib 3D simulator?).
    - Convert the custom `Vector2d` objects in [ShootingCalculator](/src/main/java/frc/robot/subsystems/ShootingCalculator.java) to `Translation2d` objects.
    - The center hub pose in RotateToHub is off by an insignificant amount.
    - Remove `SmartDashboard.putNumber()` debug info once no longer needed and before competition.

- Field Orientation
    - Field orientation fixed, fix controller axis to robot axis speed setup.

- Photon Vision
    - Check if std dev numbers make sense, especially for new adaptive setup.
    - Check if height of robot to camera offset matters for Photon Vision.

- Check if there's any automatic limit on voltage being passed in, worried about code breaking and being far away from hub, so voltage > 12 is passed in which is bad
- Remember to remove/use the sys id motor tests in controller, uses back + x
- Double check where alliance needs to be set (in robot) before competition
- We need to replace [TunerConstants.java](/src/main/java/frc/robot/generated/TunerConstants.java) with generated code once the robot is built.

## POTENTIAL BUTTON BINDINGS

| Button         | Action                                    |
|----------------|-------------------------------------------|
| Left Joystick  | Translation                               |
| Right Joystick | Rotation                                  |
| Left Trigger   | Intake (activate intake)                  |
| Right Trigger  | Shoot (activate flywheel, kicker, hopper) |
| A              | Automatic Rotate to Hub                   |
| Right Bumper   | Bring Intake Down                         |
| Left Bumper    | Raise Intake Up                           |

## Team Documentation

<https://github.com/Wildcat5e/documentation>
