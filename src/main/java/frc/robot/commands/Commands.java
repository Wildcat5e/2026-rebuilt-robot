package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public interface Commands {
    AutoAlign autoAlign = new AutoAlign(RobotContainer.drivetrain);
    RotateToHub rotateToHub = new RotateToHub();

}
