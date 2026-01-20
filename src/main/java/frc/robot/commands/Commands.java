package frc.robot.commands;

import frc.robot.RobotContainer;

public interface Commands {
    AutoAlign autoAlign = new AutoAlign(RobotContainer.drivetrain);
    RotateToHub rotateToHub = new RotateToHub(RobotContainer.drivetrain);
}
