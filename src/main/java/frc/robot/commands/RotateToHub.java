package frc.robot.commands;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class RotateToHub extends Command {
    private static final double MAX_ANGULAR_SPEED = 2;
    private static final PPHolonomicDriveController HOLONOMIC_DRIVE_CONTROLLER =
        new PPHolonomicDriveController(new PIDConstants(0, 0, 0), new PIDConstants(50, 0, 0));
    private static final PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
    private double counter;

    @Override
    public void initialize() {
        counter = 0;
    }

    @Override
    public void execute() {
        counter++;
        // POSE OF CENTER OF HUB (4.625, 4.025)
        double hubXPose = 4.625;
        double hubYPose = 4.025;

        Pose2d currentPose = RobotContainer.drivetrain.getState().Pose;
        double angleOfRobotToHub = Math.atan2((hubYPose - currentPose.getY()), (hubXPose - currentPose.getX()));

        goalState.pose = new Pose2d(0, 0, Rotation2d.fromRadians(angleOfRobotToHub));
        ChassisSpeeds outputSpeeds = HOLONOMIC_DRIVE_CONTROLLER.calculateRobotRelativeSpeeds(currentPose, goalState);
        if (counter % 50 == 0) {
            System.out.println(currentPose.getRotation());
            System.out.printf("angleOfRobotToHub(Rads: %.2f, Deg: %.2f)\n", angleOfRobotToHub,
                Math.toDegrees(angleOfRobotToHub));
        }
        RobotContainer.drivetrain.setControl(
            RobotContainer.drive.withRotationalRate(Math.min(outputSpeeds.omegaRadiansPerSecond, MAX_ANGULAR_SPEED))
                .withDesaturateWheelSpeeds(isFinished()));
    }
}
