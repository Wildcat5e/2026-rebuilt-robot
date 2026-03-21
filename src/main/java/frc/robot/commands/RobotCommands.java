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
    // public final Paths paths;
    public final ShootFuel shootFuel;

    public RobotCommands(Drivetrain drivetrain, SwerveRequest.FieldCentric swerveRequest, Flywheel flywheel,
        Hopper hopper) {
        aimAtHub =
            new AimAtTarget(drivetrain, swerveRequest, flywheel, Utilities::getHubPosition, HUB_FLYWHEEL_SPEEDS_MAP);
        aimAtUpperHome =
            new AimAtTarget(drivetrain, swerveRequest, flywheel, RobotCommands::getUpperHome, HOME_FLYWHEEL_SPEEDS_MAP);
        aimAtLowerHome =
            new AimAtTarget(drivetrain, swerveRequest, flywheel, RobotCommands::getLowerHome, HOME_FLYWHEEL_SPEEDS_MAP);
        aimHandler = new AimHandler(drivetrain, aimAtHub, aimAtUpperHome, aimAtLowerHome);
        // paths = new Paths(drivetrain);
        shootFuel = new ShootFuel(flywheel, hopper, drivetrain);
    }

    public record PositionAndFlywheelSpeedMap(Translation2d target, InterpolatingDoubleTreeMap flywheelSpeedMap) {}


    /** @return Translation2d of the Upper Home for the current Alliance. */
    static Translation2d getUpperHome() {
        return Robot.isBlueAlliance ? Constants.UPPER_HOMES.get(0) : Constants.UPPER_HOMES.get(1);
    }

    /** @return Translation2d of the Lower Home for the current Alliance. */
    static Translation2d getLowerHome() {
        return Robot.isBlueAlliance ? Constants.LOWER_HOMES.get(0) : Constants.LOWER_HOMES.get(1);
    }
}
