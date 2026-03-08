package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ShootingCalculator;
import frc.robot.Constants;
import frc.robot.controller.Controller;
import static frc.robot.Utilities.*;

public class RotateToHub extends Command {
    // Limit RotateToHub's max speed to less than main controller just for safety
    private static final double MAX_ANGULAR_SPEED = Constants.MAX_ANGULAR_SPEED - Math.PI;
    public static final PIDController PID_CONTROLLER = new PIDController(5.0, 0, 0);

    private final Drivetrain drivetrain;
    private static final SwerveRequest.FieldCentric swerveRequest = new SwerveRequest.FieldCentric();
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
        SmartDashboard.putNumber("Robot Rotation", round(currentPose.getRotation().getDegrees(), 2));
        SmartDashboard.putNumber("Target Heading", round(Math.toDegrees(targetHeading), 2));
        SmartDashboard.putNumber("Angle Difference",
            round(Math.toDegrees(targetHeading - currentPose.getRotation().getRadians()), 2));

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
        if (!DriverStation.isAutonomous()) { // Need to confirm that this works when using a path during teleop.
            drivetrain.setControl(swerveRequest.withRotationalRate(cappedVelocity));
        }
        // Feed the calculated tracking velocity to PathPlanner
        PPHolonomicDriveController.overrideRotationFeedback(() -> cappedVelocity);
    }

    @Override
    public void end(boolean interrupted) {
        Controller.allowControllerRotation = true;
        // This is pretty important: Clear the override when the command ends
        PPHolonomicDriveController.clearRotationFeedbackOverride();
    }
}
