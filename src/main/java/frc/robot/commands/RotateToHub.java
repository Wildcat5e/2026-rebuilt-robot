package frc.robot.commands;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ShootingCalculator;
import frc.robot.Constants;

public class RotateToHub extends Command {
    // Limit RotateToHub's max speed to less than main controller just for safety
    private static final double MAX_ANGULAR_SPEED = Constants.MAX_ANGULAR_SPEED - Math.PI;
    public static final PIDController PID_CONTROLLER = new PIDController(5.0, 0, 0);

    private final Drivetrain drivetrain;
    private boolean useShootingCalculator;

    public RotateToHub(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        PID_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
    }

    public RotateToHub(Drivetrain drivetrain, boolean useShootingCalculator) {
        this(drivetrain);
        this.useShootingCalculator = useShootingCalculator;
    }

    @Override
    public void initialize() {
        Controller.allowControllerRotation = false;
        PID_CONTROLLER.reset();
        SmartDashboard.putBoolean("Enable Shooting Calculator", useShootingCalculator);
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getState().Pose;
        double targetHeading = useShootingCalculator ? ShootingCalculator.calculate(drivetrain).robotHeading()
            : getRobotToHubAngle(drivetrain);

        // Debug
        SmartDashboard.putNumber("Robot Rotation", currentPose.getRotation().getDegrees());
        SmartDashboard.putNumber("Robot To Hub Angle", Math.toDegrees(targetHeading));
        SmartDashboard.putNumber("Angle Difference",
            Math.toDegrees(targetHeading - currentPose.getRotation().getRadians()));

        // --- 1. Calculate Feedforward (Predictive) ---
        // Get field-centric speeds
        ChassisSpeeds robotVel = drivetrain.getState().Speeds;
        ChassisSpeeds fieldVel = ChassisSpeeds.fromRobotRelativeSpeeds(robotVel, currentPose.getRotation());

        // Get the vector pointing from the robot to the hub
        Translation2d delta = getHubPosition().minus(currentPose.getTranslation());
        double distanceSq = Math.pow(getHubDistance(drivetrain), 2);

        double feedforwardOmega = distanceSq > 0.01
            ? fieldVel.vxMetersPerSecond * delta.getY() - fieldVel.vyMetersPerSecond * delta.getX() / distanceSq
            : 0;

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
        // This is pretty important: Clear the override when the command ends
        PPHolonomicDriveController.clearRotationFeedbackOverride();
    }
}
