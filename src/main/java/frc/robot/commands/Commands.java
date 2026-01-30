package frc.robot.commands;

import frc.robot.subsystems.Controller;

public interface Commands {
    AutoAlign autoAlign = new AutoAlign(Controller.drivetrain);
    RotateToHub rotateToHub = new RotateToHub();
}
