package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.DashboardManager;
import frc.robot.Robot;
import frc.robot.Utilities;
import frc.robot.subsystems.ShootingCalculator;

import java.util.function.Supplier;

import static frc.robot.Constants.HOME_FLYWHEEL_SPEEDS_MAP;
import static frc.robot.Constants.HUB_FLYWHEEL_SPEEDS_MAP;
import static frc.robot.Utilities.getTargetDistance;

public class AimAtTarget extends Command {
    // Limit max speed to less than main controller just for safety
    private static final double MAX_ANGULAR_SPEED = Constants.MAX_ANGULAR_SPEED - Math.PI;

    public static final PIDController PID_CONTROLLER = new PIDController(5.0, 0, 0);

    static {
        PID_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
    }

    private final SwerveRequest.FieldCentric swerveRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);


    private final Robot                      robot;
    private final Supplier<Translation2d>    targetSupplier;
    private final InterpolatingDoubleTreeMap flywheelSpeedMap;

    private Translation2d target;
    private Pose2d        currentPose;
    private double        targetHeading;

    private AimAtTarget(Robot robot,
                        Supplier<Translation2d> targetSupplier,
                        InterpolatingDoubleTreeMap flywheelSpeedMap) {
        this.robot            = robot;
        this.targetSupplier   = targetSupplier;
        this.flywheelSpeedMap = flywheelSpeedMap;
        registerTelemetry();
        // Do not add the drivetrain -- AimAtTarget controls rotation, the Driver retains control of translation.
        addRequirements(robot.flywheel);
    }

    public static AimAtTarget hub(Robot robot) {
        return new AimAtTarget(robot, Utilities::getHubPosition, HUB_FLYWHEEL_SPEEDS_MAP);
    }

    public static AimAtTarget upperHome(Robot robot) {
        return new AimAtTarget(robot, Utilities::getUpperHome, HOME_FLYWHEEL_SPEEDS_MAP);
    }

    public static AimAtTarget lowerHome(Robot robot) {
        return new AimAtTarget(robot, Utilities::getLowerHome, HOME_FLYWHEEL_SPEEDS_MAP);
    }

    @Override public void initialize() {
        PID_CONTROLLER.reset();
        target               = targetSupplier.get();
        robot.manualRotation = false;
    }

    @Override public void execute() {
        currentPose = robot.drivetrain.getState().Pose;
        var shotSolution = ShootingCalculator.calculate(robot.drivetrain, target, flywheelSpeedMap);
        targetHeading = shotSolution.robotHeading();
        // --- 1. Feedforward ---
        double feedforwardVelocity = getFeedforwardVelocity(currentPose, target);

        // --- 2. Calculate PID (Reactive) ---
        double pidVelocity = PID_CONTROLLER.calculate(currentPose.getRotation().getRadians(), targetHeading);

        // --- 3. Combine and Cap ---
        // FF does the physics tracking, PID cleans up the physical errors
        double totalVelocity  = feedforwardVelocity + pidVelocity;
        double cappedVelocity = Math.max(Math.min(totalVelocity, MAX_ANGULAR_SPEED), -MAX_ANGULAR_SPEED);

        applyRotation(cappedVelocity);
        robot.flywheel.setFlywheelSpeed(shotSolution.flywheelSpeed());
    }

    @Override public void end(boolean interrupted) {
        robot.flywheel.stopFlywheel();
        // This is pretty important: Clear the override when the command ends
        PPHolonomicDriveController.clearRotationFeedbackOverride();
        robot.manualRotation = true;
    }

    private double getFeedforwardVelocity(Pose2d currentPose, Translation2d target) {
        // --- 1. Calculate Feedforward (Predictive) ---
        // Get field-centric speeds
        ChassisSpeeds robotVel = robot.drivetrain.getState().Speeds;
        ChassisSpeeds fieldVel = ChassisSpeeds.fromRobotRelativeSpeeds(robotVel, currentPose.getRotation());

        // Get the vector pointing from the robot to the target
        Translation2d delta      = target.minus(currentPose.getTranslation());
        double        distanceSq = Math.pow(getTargetDistance(robot.drivetrain, target), 2);

        if (distanceSq > 0.01) {
            return (fieldVel.vxMetersPerSecond * delta.getY() - fieldVel.vyMetersPerSecond * delta.getX()) / distanceSq;
        } else {return 0;}
    }

    private void applyRotation(double cappedVelocity) {
        // --- 4. Apply to Drivetrain & PathPlanner ---
        if (!DriverStation.isAutonomous()) { // Need to confirm that this works when using a path during teleop.
            robot.drivetrain.setControl(swerveRequest.withRotationalRate(cappedVelocity));
        }
        // Feed the calculated tracking velocity to PathPlanner
        PPHolonomicDriveController.overrideRotationFeedback(() -> cappedVelocity);
    }

    private void registerTelemetry() { // @formatter:off
        DashboardManager.setupRotateToHub(
            () -> this.currentPose != null ? this.currentPose : new Pose2d(),
            () -> this.targetHeading,
            () -> this.currentPose != null ? this.targetHeading - this.currentPose.getRotation().getRadians() : 0.0
        ); // @formatter:on
    }
}
