package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;

public class RobotCommands {
    public final RotateToHub rotateToHub;
    public final RotateToHub rotateToHubShootingCalc;
    public final Paths paths;
    public final Flywheel flywheel;
    public final Hopper hopper;
    public final ShootFuel shootFuel;
    public final Intake intake;

    public RobotCommands(Drivetrain drivetrain) {
        rotateToHub = new RotateToHub(drivetrain, false);
        rotateToHubShootingCalc = new RotateToHub(drivetrain, true);
        paths = new Paths(drivetrain);
        flywheel = new Flywheel(drivetrain);
        hopper = new Hopper();
        shootFuel = new ShootFuel(flywheel, hopper, drivetrain);
        intake = new Intake();
    }
}
