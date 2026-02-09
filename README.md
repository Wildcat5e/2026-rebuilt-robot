# 2026-rebuilt-robot

## Notes

This is a great repo.

## To-Do

- PathPlanner
    - Configure max velocity and acceleration
    - Before competition, make sure to actually set the correct values for settings that are used in pathplanner (like robot weight) or else we cooked.

- RotateToHub/Shooting While Moving
    - Find a way to simulate shooting the fuel to test the math in [ShootingCalculator.java](/src/main/java/frc/robot/subsystems/ShootingCalculator.java). Preferably in a 3D sim, if possible.
    - Will need to go back and check the PID values and max speed.
    - The center hub pose in RotateToHub is off by an insignificant amount.
    - Remove `SmartDashboard.putNumber()` debug info once no longer needed and before competition.

- Photon Vision
    - Check if std dev numbers make sense, especially for new adaptive setup.
    - Check if height of robot to camera offset matters for Photon Vision.

- Controller
    - Nicholas needs to add documentation about axis being flipped in controller

- Change Controller to whatever one we use rather than MultiController before competition.
- Check if there's any automatic limit on voltage being passed in, worried about code breaking and being far away from hub, so voltage > 12 is passed in which is bad
- Remember to remove/use the sys id motor tests in controller, uses back + x
- Double check where alliance needs to be set (in robot) before competition
- We need to replace [TunerConstants.java](/src/main/java/frc/robot/generated/TunerConstants.java) with generated code once the robot is built.

## ROBOT TESTING LIST

- Test intake arm drop (voltage)
- Find voltages for intake, hopper, kicker to all function together for moving fuel to flywheel
- Test shooter voltages, map out distances and flywheel speed where fuel can be shot into hub
- Test final shooting command implementation (dynamic vs static)
- Run SysID and test feedforward on flywheel and how it impacts shooting a lot of fuel
- Test shooting on the move and make any changes necessary
- Test on the fly paths in teleop
- Test/develop autos

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
