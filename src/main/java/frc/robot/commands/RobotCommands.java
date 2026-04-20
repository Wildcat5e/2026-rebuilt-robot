package frc.robot.commands;

import static frc.robot.Constants.*;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.DashboardManager;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.utilities.FieldUtils;

public class RobotCommands {
    // public final AimAtTarget aimAtHub;
    // public final AimAtTarget aimAtUpperHome;
    // public final AimAtTarget aimAtLowerHome;
    // public final AimHandler aimHandler;
    public final Paths paths;
    public final ShootFuel shootFuel;

    public RobotCommands(Drivetrain drivetrain, SwerveRequest.FieldCentric swerveRequest, Flywheel flywheel,
        Hopper hopper) {
        // aimAtHub = new AimAtTarget(drivetrain, swerveRequest, flywheel, FieldUtils::getHubPosition,
        //     HUB_FLYWHEEL_SPEEDS_MAP, DashboardManager::getFlywheelSpeedMultiplier);
        // aimAtUpperHome = new AimAtTarget(drivetrain, swerveRequest, flywheel, FieldUtils::getUpperHome,
        //     HOME_FLYWHEEL_SPEEDS_MAP, DashboardManager::getHomeFlywheelSpeedMultiplier);
        // aimAtLowerHome = new AimAtTarget(drivetrain, swerveRequest, flywheel, FieldUtils::getLowerHome,
        //     HOME_FLYWHEEL_SPEEDS_MAP, DashboardManager::getHomeFlywheelSpeedMultiplier);
        // aimHandler = new AimHandler(drivetrain, aimAtHub, aimAtUpperHome, aimAtLowerHome);
        paths = new Paths(drivetrain);
        shootFuel = new ShootFuel(flywheel, hopper);
    }
}
