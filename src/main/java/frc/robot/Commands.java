package frc.robot;

import frc.robot.commands.Paths;
import frc.robot.commands.RotateToHub;
import frc.robot.commands.ShootFuel;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;

public class Commands {
    private final Drivetrain drivetrain;

    public final RotateToHub rotateToHub;
    public final Paths paths;
    public final Flywheel flywheel;
    public final Hopper hopper;
    public final ShootFuel shootFuel;

    public Commands(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        rotateToHub = new RotateToHub(drivetrain);
        paths = new Paths(drivetrain);
        flywheel = new Flywheel(drivetrain);
        hopper = new Hopper();
        shootFuel = new ShootFuel(flywheel, hopper, drivetrain);
    }
}
