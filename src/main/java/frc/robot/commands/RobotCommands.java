package frc.robot.commands;

import frc.robot.RobotContainer;

public interface RobotCommands {
    AutoAlign autoAlign = new AutoAlign(RobotContainer.drivetrain);
    RotateToHub rotateToHub = new RotateToHub();
    Paths paths = new Paths();
}
