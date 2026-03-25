package frc.robot.controller;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface Controller {
    Translation2d translation();

    double rotation();

    Trigger runIntake();

    Trigger shootFuel();

    Trigger lowerIntake();

    Trigger raiseIntake();

    Trigger aimHandler();

    Trigger manualFlywheel();

    Trigger seedFieldCentric();

    Trigger reverse();

    Trigger povUp();

    Trigger povDown();

    Trigger povLeft();

    Trigger povRight();

    Trigger forwardSysIdQuasi();

    Trigger backwardSysIdQuasi();

    Trigger forwardSysIdDynamic();

    Trigger backwardSysIdDynamic();
}
