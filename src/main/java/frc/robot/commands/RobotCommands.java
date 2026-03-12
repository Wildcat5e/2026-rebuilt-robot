package frc.robot.commands;

import static frc.robot.Utilities.*;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;

public class RobotCommands {
    public final AimAtTarget aimAtHub;
    public final AimAtTarget aimAtTarget;
    // public final Paths paths;
    public final ShootFuel shootFuel;

    public RobotCommands(Drivetrain drivetrain, SwerveRequest.FieldCentric swerveRequest, Flywheel flywheel,
        Hopper hopper) {
        aimAtHub = new AimAtTarget(drivetrain, swerveRequest, getHubPosition());
        aimAtTarget = new AimAtTarget(drivetrain, swerveRequest, getHubPosition());
        // paths = new Paths(drivetrain);
        shootFuel = new ShootFuel(flywheel, hopper, drivetrain);
    }
}
