package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class RotateToHub extends Command {
    private static final double MAX_ANGULAR_SPEED = Math.PI; // rad/s
    public static final ProfiledPIDController PID_CONTROLLER = new ProfiledPIDController(20, 0, 0,
        new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED, RobotContainer.MAX_ANGULAR_ACCEL - Math.PI));
    private double counter; // debug

    public RotateToHub() {
        PID_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        RobotContainer.Config.allowControllerRotation = false;
        counter = 0; // debug
    }

    @Override
    public void execute() {
        counter++; // debug
        // POSE OF CENTER OF HUB (4.625, 4.025)
        double hubXPose = 4.625;
        double hubYPose = 4.025;

        Pose2d currentPose = RobotContainer.drivetrain.getState().Pose;
        double angleOfRobotToHub = Math.atan2((hubYPose - currentPose.getY()), (hubXPose - currentPose.getX()));

        double outputSpeeds = PID_CONTROLLER.calculate(currentPose.getRotation().getRadians(), angleOfRobotToHub);
        if (counter % 20 == 0) { // debug
            System.out.println(currentPose.getRotation());
            System.out.printf("angleOfRobotToHub(Rads: %.2f, Deg: %.2f)\n", angleOfRobotToHub,
                Math.toDegrees(angleOfRobotToHub));
        }
        RobotContainer.drivetrain
            .setControl(RobotContainer.drive.withRotationalRate(Math.min(outputSpeeds, MAX_ANGULAR_SPEED)));
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.Config.allowControllerRotation = true;
    }
}
