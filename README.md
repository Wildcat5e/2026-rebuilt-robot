# 2026-rebuilt-robot

## NOTES

This is a great repo.

## TO-DO

- **PathPlanner**
    - Configure max velocity and acceleration.
    - Before competition, make sure to set correct values for settings used in PathPlanner (like robot weight), or else we're *cooked*.

- **[RotateToHub](/src/main/java/frc/robot/commands/RotateToHub.java)/[ShootingCalculator](/src/main/java/frc/robot/subsystems/ShootingCalculator.java)**
    - Check PID and max speed values.
    - Remove `SmartDashboard.putNumber()` debug info once no longer needed and before competition.

- **[PhotonVision](/src/main/java/frc/robot/subsystems/PhotonVision.java)**
    - Check if standard deviation numbers make sense, especially for new adaptive setup.
    - Check if height-of-robot-to-camera offset matters for Photon Vision.

- **[Controller](/src/main/java/frc/robot/subsystems/Controller.java)**
    - Nicholas needs to add documentation about axis being flipped in `Controller`.
    - Nicholas needs to refactor `Controller` to incorporate keybinds and setting drivetrain control instead of in `Robot`

- **[Paths](/src/main/java/frc/robot/commands/Paths.java)**
    - See if we want to use `end()` in the `Paths()` constructor (see [Nicholas's comment](https://github.com/Wildcat5e/2026-rebuilt-robot/pull/11/changes#r2895846586)).

- **General Stuff**
    - Before competition, change `Controller` to whichever one we use, rather than `MultiController`.
    - Verify voltage limiting is implemented to prevent exceeding 12V, for example when robot is far from target, or in similar edge cases.
    - Double check where the Alliance needs to be set (in `Robot`) before competition.
    - Update [Drivetrain.java](/src/main/java/frc/robot/subsystems/Drivetrain.java) with new generated code once the robot is built.
    - Update [TunerConstants.java](/src/main/java/frc/robot/generated/TunerConstants.java) with new generated code once the robot is built.
    - Update CAD with straight cut instead of diagonal corners on frame rail that attaches to swerve module.

## POTENTIAL BUTTON BINDINGS

| Button         | Action                                    |
|----------------|-------------------------------------------|
| Left Joystick  | Translation                               |
| Right Joystick | Rotation                                  |
| Left Trigger   | Activate intake                           |
| Right Trigger  | Shoot (activate flywheel, kicker, hopper) |
| A              | Automatic Rotate to Hub                   |
| Right Bumper   | Bring Intake Down                         |
| Left Bumper    | Raise Intake Up                           |

### Flight Stick

| Button            | Action                                    |
|-------------------|-------------------------------------------|
| Joystick          | Translation                               |
| Joystick Rotation | Rotation                                  |
| Left Trigger      | Activate intake                           |
| Trigger           | Shoot (activate flywheel, kicker, hopper) |
| A                 | Automatic Rotate to Hub                   |
| Right Bumper      | Bring Intake Down                         |
| Left Bumper       | Raise Intake Up                           |

## DOCUMENTATION

<https://github.com/Wildcat5e/documentation>
