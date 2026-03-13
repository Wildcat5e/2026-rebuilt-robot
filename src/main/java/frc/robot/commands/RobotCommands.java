package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.Utilities;
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
        aimAtHub = new AimAtTarget(drivetrain, swerveRequest, Utilities::getHubPosition);
        aimAtTarget = new AimAtTarget(drivetrain, swerveRequest, Utilities::getHubPosition);
        // paths = new Paths(drivetrain);
        shootFuel = new ShootFuel(flywheel, hopper, drivetrain);
    }
}
