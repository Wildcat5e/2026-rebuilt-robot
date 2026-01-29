package frc.robot.commands;

import static frc.robot.Utilities.*;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class RotateToHub extends Command {
    private static final double MAX_ANGULAR_SPEED = 2 * Math.PI; // rad/s
    public static final ProfiledPIDController PID_CONTROLLER = new ProfiledPIDController(20, 0, 0,
        new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED, RobotContainer.MAX_ANGULAR_ACCEL - Math.PI));

    public RotateToHub() {
        PID_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        RobotContainer.Config.allowControllerRotation = false;
    }

    @Override
    public void execute() {
        Pose2d currentPose = RobotContainer.drivetrain.getState().Pose;
        double robotToHubAngle = getRobotToHubAngle();
        // debug
        SmartDashboard.putNumber("Robot Rotation", currentPose.getRotation().getDegrees());
        SmartDashboard.putNumber("Robot To Hub Angle", Math.toDegrees(robotToHubAngle));

        double velocity = PID_CONTROLLER.calculate(currentPose.getRotation().getRadians(), robotToHubAngle);
        velocity = Math.max(Math.min(velocity, MAX_ANGULAR_SPEED), -MAX_ANGULAR_SPEED); // cap output speed

        RobotContainer.drivetrain.setControl(RobotContainer.swerveRequest.withRotationalRate(velocity));
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.Config.allowControllerRotation = true;
    }
}
