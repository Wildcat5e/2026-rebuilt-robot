package frc.robot.commands;

import static frc.robot.Utilities.*;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ShootingCalculator;

public class RotateToHub extends Command {
    private static final double MAX_ANGULAR_SPEED = 2 * Math.PI; // Constants.MAX_ANGULAR_SPEED - Math.PI
    private static final double MAX_ANGULAR_ACCEL = 3 * Math.PI; // Constants.MAX_ANGULAR_ACCEL - Math.PI
    public static final ProfiledPIDController PID_CONTROLLER =
        new ProfiledPIDController(20, 0, 0, new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED, MAX_ANGULAR_ACCEL));
    private final Drivetrain drivetrain;
    private boolean useShootingCalculator = false; // debug var

    public RotateToHub(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        PID_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        Controller.allowControllerRotation = false;
        PID_CONTROLLER.reset(getRobotRotationState(drivetrain));
        SmartDashboard.putBoolean("Enable Schooting Calculator", useShootingCalculator); // debug
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getState().Pose;
        // ternary is for debug
        double targetHeading = useShootingCalculator ? ShootingCalculator.calculate(drivetrain).robotHeading()
            : getRobotToHubAngle(drivetrain);
        // debug
        SmartDashboard.putNumber("Robot Rotation", currentPose.getRotation().getDegrees());
        SmartDashboard.putNumber("Robot To Hub Angle", Math.toDegrees(targetHeading));

        // Calculate rotational velocity to apply to get to the correct heading. This is the bad old way.
        double velocity = PID_CONTROLLER.calculate(currentPose.getRotation().getRadians(), targetHeading);
        velocity = Math.max(Math.min(velocity, MAX_ANGULAR_SPEED), -MAX_ANGULAR_SPEED); // cap output speed

        drivetrain.setControl(Robot.swerveRequest.withRotationalRate(velocity));
        // PPHolonomicDriveController.overrideRotationFeedback(() -> 
        //     return somethingToDoFeedforwardOrSomething;
        // );
    }

    @Override
    public void end(boolean interrupted) {
        Controller.allowControllerRotation = true;
        PPHolonomicDriveController.clearRotationFeedbackOverride();
    }

    public static TrapezoidProfile.State getRobotRotationState(Drivetrain drivetrain) {
        var currentState = drivetrain.getState();
        return new TrapezoidProfile.State(currentState.Pose.getRotation().getRadians(),
            currentState.Speeds.omegaRadiansPerSecond);
    }
}
