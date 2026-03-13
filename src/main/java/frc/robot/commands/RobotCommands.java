package frc.robot.commands;

import static frc.robot.Utilities.*;
import java.util.List;
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

public class RobotCommands {
    public final AimAtTarget aimAtHub;
    public final AimAtTarget aimAtLeftHome;
    public final AimAtTarget aimAtRightHome;
    public final TargetHubOrHome targetHubOrHome;
    public final Paths paths;
    public final ShootFuel shootFuel;

    public RobotCommands(Drivetrain drivetrain, Flywheel flywheel, Hopper hopper,
        SwerveRequest.FieldCentric swerveRequest) {
        aimAtHub = new AimAtTarget(drivetrain, swerveRequest, Utilities::getHubPosition, HUB_FLYWHEEL_SPEEDS_MAP); // @formatter:off
        aimAtLeftHome = new AimAtTarget(drivetrain, swerveRequest, RobotCommands::getLeftHome, HOME_FLYWHEEL_SPEEDS_MAP);
        aimAtRightHome = new AimAtTarget(drivetrain, swerveRequest, RobotCommands::getRightHome, HOME_FLYWHEEL_SPEEDS_MAP); // @formatter:on
        targetHubOrHome = new TargetHubOrHome(drivetrain, aimAtHub, aimAtLeftHome, aimAtRightHome);
        paths = new Paths(drivetrain);
        shootFuel = new ShootFuel(flywheel, hopper, drivetrain);
    }

    public record PositionAndFlywheelSpeedMap(Translation2d target, InterpolatingDoubleTreeMap flywheelSpeedMap) {}


    /** @return Translation2d of the Left Home for the current alliance. */
    static Translation2d getLeftHome() {
        return Robot.isBlueAlliance ? Constants.LEFT_HOMES.get(0) : Constants.LEFT_HOMES.get(1);
    }

    /** @return Translation2d of the Left Home for the current alliance. */
    static Translation2d getRightHome() {
        return Robot.isBlueAlliance ? Constants.RIGHT_HOMES.get(0) : Constants.RIGHT_HOMES.get(1);
    }
}
