package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;

public class RobotCommands {
    public final RotateToHub rotateToHub;
    public final RotateToHub rotateToHubShootingCalc;
    // public final Paths paths;
    public final ShootFuel shootFuel;

    public RobotCommands(Drivetrain drivetrain, Flywheel flywheel, Intake intake, Hopper hopper) {
        rotateToHub = new RotateToHub(drivetrain, false);
        rotateToHubShootingCalc = new RotateToHub(drivetrain, true);
        // paths = new Paths(drivetrain);
        shootFuel = new ShootFuel(flywheel, hopper, intake, drivetrain);
    }
}
