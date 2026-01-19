package frc.robot.commands;

import frc.robot.RobotContainer;

public interface Commands {
    public static final AutoAlign autoAlign = new AutoAlign(RobotContainer.drivetrain);
}
