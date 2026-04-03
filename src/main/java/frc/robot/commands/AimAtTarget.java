package frc.robot.commands;

import static frc.robot.utilities.TargetingUtils.*;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
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
import frc.robot.controller.Controller;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.ShootingCalculator;

public class AimAtTarget extends Command {
    // Limit max speed to less than main controller just for safety
    private static final double MAX_ANGULAR_SPEED = Constants.MAX_ANGULAR_SPEED - Math.PI;
    public static final PIDController PID_CONTROLLER = new PIDController(5.0, 0, 0);
    static {
        PID_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
    }
    private final Drivetrain drivetrain;
    private final Flywheel flywheel;
    private final SwerveRequest.FieldCentric swerveRequest;
    private final Supplier<Translation2d> targetSupplier;
    private Translation2d target;
    private final InterpolatingDoubleTreeMap flywheelSpeedMap;
    private DoubleSupplier flywheelSpeedMultiplier;
    private Pose2d currentPose;
    private double targetHeading;

    public AimAtTarget(Drivetrain drivetrain, SwerveRequest.FieldCentric swerveRequest, Flywheel flywheel,
        Supplier<Translation2d> targetSupplier, InterpolatingDoubleTreeMap flywheelSpeedMap,
        DoubleSupplier flywheelSpeedMultiplier) {
        this.drivetrain = drivetrain;
        this.flywheel = flywheel;
        this.swerveRequest = swerveRequest;
        this.targetSupplier = targetSupplier;
        this.flywheelSpeedMap = flywheelSpeedMap;
        this.flywheelSpeedMultiplier = flywheelSpeedMultiplier;
        registerTelemetry();
    }

    @Override
    public void initialize() {
        Controller.allowControllerRotation = false;
        PID_CONTROLLER.reset();
        target = targetSupplier.get();
    }

    @Override
    public void execute() {
        currentPose = drivetrain.getState().Pose;
        var shotSolution = ShootingCalculator.calculate(drivetrain, target, flywheelSpeedMap);
        targetHeading = shotSolution.robotHeading();
        // --- 1. Feedforward ---
        double feedforwardVelocity = getFeedforwardVelocity(currentPose, target);

        // --- 2. Calculate PID (Reactive) ---
        double pidVelocity = PID_CONTROLLER.calculate(currentPose.getRotation().getRadians(), targetHeading);

        // --- 3. Combine and Cap ---
        // FF does the physics tracking, PID cleans up the physical errors
        double totalVelocity = feedforwardVelocity + pidVelocity;
        double cappedVelocity = Math.max(Math.min(totalVelocity, MAX_ANGULAR_SPEED), -MAX_ANGULAR_SPEED);

        applyRotation(cappedVelocity);
        flywheel.setFlywheelSpeed(
            flywheelSpeedMap.get(getTargetDistance(drivetrain, target)) * flywheelSpeedMultiplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.stopFlywheel();
        Controller.allowControllerRotation = true;
        // This is pretty important: Clear the override when the command ends
        PPHolonomicDriveController.clearRotationFeedbackOverride();
    }

    private double getFeedforwardVelocity(Pose2d currentPose, Translation2d target) {
        // --- 1. Calculate Feedforward (Predictive) ---
        // Get field-centric speeds
        ChassisSpeeds robotVel = drivetrain.getState().Speeds;
        ChassisSpeeds fieldVel = ChassisSpeeds.fromRobotRelativeSpeeds(robotVel, currentPose.getRotation());

        // Get the vector pointing from the robot to the target
        Translation2d delta = target.minus(currentPose.getTranslation());
        double distanceSq = Math.pow(getTargetDistance(drivetrain, target), 2);

        if (distanceSq > 0.01) {
            return (fieldVel.vxMetersPerSecond * delta.getY() - fieldVel.vyMetersPerSecond * delta.getX()) / distanceSq;
        } else return 0;
    }

    private void applyRotation(double cappedVelocity) {
        // --- 4. Apply to Drivetrain & PathPlanner ---
        if (!DriverStation.isAutonomous()) { // Need to confirm that this works when using a path during teleop.
            drivetrain.setControl(swerveRequest.withRotationalRate(cappedVelocity));
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
