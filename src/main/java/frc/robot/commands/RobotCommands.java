package frc.robot.commands;

import static frc.robot.Utilities.*;
import static frc.robot.Constants.*;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Utilities;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;

public class RobotCommands {
    public final AimAtTarget aimAtHub;
    public final AimAtTarget aimAtUpperHome;
    public final AimAtTarget aimAtLowerHome;
    public final AimHandler aimHandler;
    public final Paths paths;
    public final ShootFuel shootFuel;

    public RobotCommands(Drivetrain drivetrain, SwerveRequest.FieldCentric swerveRequest, Flywheel flywheel,
        Hopper hopper) {
        aimAtHub =
            new AimAtTarget(drivetrain, swerveRequest, flywheel, Utilities::getHubPosition, HUB_FLYWHEEL_SPEEDS_MAP);
        aimAtUpperHome =
            new AimAtTarget(drivetrain, swerveRequest, flywheel, Utilities::getUpperHome, HOME_FLYWHEEL_SPEEDS_MAP);
        aimAtLowerHome =
            new AimAtTarget(drivetrain, swerveRequest, flywheel, Utilities::getLowerHome, HOME_FLYWHEEL_SPEEDS_MAP);
        aimHandler = new AimHandler(drivetrain, aimAtHub, aimAtUpperHome, aimAtLowerHome);
        paths = new Paths(drivetrain);
        shootFuel = new ShootFuel(flywheel, hopper, drivetrain);
    }
}
