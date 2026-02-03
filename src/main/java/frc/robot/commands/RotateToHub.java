package frc.robot.commands;

import static frc.robot.Utilities.*;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.ShootingCalculator;

public class RotateToHub extends Command {
    private static final double MAX_ANGULAR_SPEED = 2 * Math.PI; // Constants.MAX_ANGULAR_SPEED - Math.PI
    private static final double MAX_ANGULAR_ACCEL = 3 * Math.PI; // Constants.MAX_ANGULAR_ACCEL - Math.PI
    public static final ProfiledPIDController PID_CONTROLLER =
        new ProfiledPIDController(20, 0, 0, new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED, MAX_ANGULAR_ACCEL));
    double velocity;

    public RotateToHub() {
        PID_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        Controller.allowControllerRotation = false;
        PID_CONTROLLER.reset(getRobotRotationState());
    }

    @Override
    public void execute() {
        Pose2d currentPose = Controller.drivetrain.getState().Pose;
        double targetAngle = ShootingCalculator.calculate().robotHeading;
        // debug
        SmartDashboard.putNumber("Robot Rotation", currentPose.getRotation().getDegrees());
        SmartDashboard.putNumber("Robot To Hub Angle", Math.toDegrees(targetAngle));

        velocity = PID_CONTROLLER.calculate(currentPose.getRotation().getRadians(), targetAngle);
        velocity = Math.max(Math.min(velocity, MAX_ANGULAR_SPEED), -MAX_ANGULAR_SPEED); // cap output speed

        Controller.drivetrain.setControl(Controller.swerveRequest.withRotationalRate(velocity));
        PPHolonomicDriveController.overrideRotationFeedback(() -> {
            // Calculate feedback from your custom PID controller
            return velocity;
        });
    }

    @Override
    public void end(boolean interrupted) {
        Controller.allowControllerRotation = true;
        PPHolonomicDriveController.clearRotationFeedbackOverride();
    }

    public static TrapezoidProfile.State getRobotRotationState() {
        var currentState = Controller.drivetrain.getState();
        return new TrapezoidProfile.State(currentState.Pose.getRotation().getRadians(),
            currentState.Speeds.omegaRadiansPerSecond);
    }
}
