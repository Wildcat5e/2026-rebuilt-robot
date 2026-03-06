# 2026-rebuilt-robot

## NOTES

This is a great repo.

## TO-DO

- **PathPlanner**
    - Configure max velocity and acceleration.
    - Before competition, make sure to set correct values for settings used in PathPlanner (like robot weight), or else we're *cooked*.

- **RotateToHub/Shooting While Moving**
    - Check PID and max speed values.
    - Remove `SmartDashboard.putNumber()` debug info once no longer needed and before competition.

- **Photon Vision**
    - Check if standard deviation numbers make sense, especially for new adaptive setup.
    - Check if height-of-robot-to-camera offset matters for Photon Vision.

- **Controller**
    - Nicholas needs to add documentation about axis being flipped in controller.

- **Paths.java**
    - See if we want to use `end()` in the `Paths()` constructor (see [Nicholas's comment](https://github.com/Wildcat5e/2026-rebuilt-robot/pull/11/changes#r2895846586)).

- **General Stuff**
    - Before competition, change Controller to whichever one we use, rather than MultiController.
    - Check if there's any automatic limit on voltage being passed in. We should ensure that it's not possible for the Robot being extremely far from the target, or some similar situation, to set a voltage greater than 12V.
    - Double check where the Alliance needs to be set (in `Robot`) before competition.
    - Replace [TunerConstants.java](/src/main/java/frc/robot/generated/TunerConstants.java) with generated code once the robot is built.
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

## Team Documentation

<https://github.com/Wildcat5e/documentation>
