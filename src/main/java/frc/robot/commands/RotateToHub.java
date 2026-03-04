package frc.robot.commands;

import static frc.robot.Utilities.*;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ShootingCalculator;

public class RotateToHub extends Command {
    private static final double MAX_ANGULAR_SPEED = 2 * Math.PI;
    private static final double MAX_ANGULAR_ACCEL = 3 * Math.PI;
    public static final ProfiledPIDController PID_CONTROLLER =
        new ProfiledPIDController(20, 0, 0, new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED, MAX_ANGULAR_ACCEL));

    private final Drivetrain drivetrain;
    private final boolean useShootingCalculator;

    public RotateToHub(Drivetrain drivetrain, boolean useShootingCalculator) {
        this.drivetrain = drivetrain;
        this.useShootingCalculator = useShootingCalculator;
        PID_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
    }

    public RotateToHub(Drivetrain drivetrain) {
        this(drivetrain, false);
    }

    @Override
    public void initialize() {
        Controller.allowControllerRotation = false;
        PID_CONTROLLER.reset(getRobotRotationState(drivetrain));
        SmartDashboard.putBoolean("Enable Shooting Calculator", useShootingCalculator);
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getState().Pose;
        double targetHeading = useShootingCalculator ? ShootingCalculator.calculate(drivetrain).robotHeading()
            : getRobotToHubAngle(drivetrain);

        // --- 1. Calculate Feedforward (Predictive) ---
        // Get field-centric speeds
        ChassisSpeeds robotVel = drivetrain.getState().Speeds;
        ChassisSpeeds fieldVel = ChassisSpeeds.fromRobotRelativeSpeeds(robotVel, currentPose.getRotation());

        Translation2d hubPos = getHubPosition();
        double dx = hubPos.getX() - currentPose.getX();
        double dy = hubPos.getY() - currentPose.getY();
        double distanceSq = (dx * dx) + (dy * dy);

        double feedforwardOmega = 0;
        if (distanceSq > 0.01) { // Prevent division by zero if we are exactly on the hub
            feedforwardOmega = (fieldVel.vyMetersPerSecond * dx - fieldVel.vxMetersPerSecond * dy) / distanceSq;
        }

        // --- 2. Calculate PID (Reactive) ---
        double pidOmega = PID_CONTROLLER.calculate(currentPose.getRotation().getRadians(), targetHeading);

        // --- 3. Combine and Cap ---
        // FF does the physics tracking, PID cleans up the physical errors
        double totalVelocity = feedforwardOmega + pidOmega;
        final double cappedVelocity = Math.max(Math.min(totalVelocity, MAX_ANGULAR_SPEED), -MAX_ANGULAR_SPEED);

        // --- 4. Apply to Drivetrain & PathPlanner ---
        drivetrain.setControl(Robot.swerveRequest.withRotationalRate(cappedVelocity));

        // Feed the calculated tracking velocity to PathPlanner
        PPHolonomicDriveController.overrideRotationFeedback(() -> cappedVelocity);

        // Debugging
        SmartDashboard.putNumber("Robot Rotation", currentPose.getRotation().getDegrees());
        SmartDashboard.putNumber("Target Heading", Math.toDegrees(targetHeading));
        SmartDashboard.putNumber("FF Omega", feedforwardOmega);
        SmartDashboard.putNumber("PID Omega", pidOmega);
    }

    @Override
    public void end(boolean interrupted) {
        Controller.allowControllerRotation = true;
        // Important: Clear the override when the command ends
        PPHolonomicDriveController.clearRotationFeedbackOverride();
    }

    public static TrapezoidProfile.State getRobotRotationState(Drivetrain drivetrain) {
        var currentState = drivetrain.getState();
        return new TrapezoidProfile.State(currentState.Pose.getRotation().getRadians(),
            currentState.Speeds.omegaRadiansPerSecond);
    }
}
