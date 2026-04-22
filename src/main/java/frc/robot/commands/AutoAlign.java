package frc.robot.commands;

import java.util.List;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class AutoAlign extends Command {
    private static final double POSITION_TOLERANCE = 0.025;
    private static final double ROTATION_TOLERANCE = 0.025;

    private static final PPHolonomicDriveController HOLONOMIC_DRIVE_CONTROLLER =
        new PPHolonomicDriveController(new PIDConstants(5, .5, .2), new PIDConstants(5, .5, .2));

    private static final double MAX_DISTANCE = 3.0;
    private static final double TIME_LIMIT_SECONDS = 3.0;
    private final Timer timer = new Timer();

    private final Drivetrain drivetrain;
    private final SwerveRequest.FieldCentric swerveRequest = new SwerveRequest.FieldCentric();

    private final PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();

    private static final Pose2d CENTER_HUB = new Pose2d(3.0, 4.0, new Rotation2d(0));
    public static final List<Pose2d> TAG_POSE_LIST = List.of(CENTER_HUB);

    private boolean tooFar = false;

    public AutoAlign(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        timer.restart();
        tooFar = false;

        Pose2d currentPose = drivetrain.getState().Pose;
        Pose2d nearestTagPose = currentPose.nearest(TAG_POSE_LIST);
        double distance = currentPose.getTranslation().getDistance(nearestTagPose.getTranslation());
        goalState.pose = nearestTagPose;

        if (distance > MAX_DISTANCE) {
            System.out.println("_\nTOO FAR DUMMY\n_");
            tooFar = true;
        }
    }

    @Override
    public void execute() {
        if (tooFar) return;

        Pose2d currentPose = drivetrain.getState().Pose;
        ChassisSpeeds outputSpeeds = HOLONOMIC_DRIVE_CONTROLLER.calculateRobotRelativeSpeeds(currentPose, goalState);
        drivetrain.setControl(swerveRequest.withVelocityX(outputSpeeds.vxMetersPerSecond)
            .withVelocityY(outputSpeeds.vyMetersPerSecond).withRotationalRate(outputSpeeds.omegaRadiansPerSecond));
    }

    @Override
    public boolean isFinished() {
        if (tooFar) return true; // Abort condition handled correctly here

        if (timer.hasElapsed(TIME_LIMIT_SECONDS)) {
            System.out.println("_\nTIME LIMIT HIT\n_");
            return true;
        }

        if (isWithinTolerance()) {
            System.out.println("_\nTOLERANCE HIT\n_");
            return true;
        }

        return false;
    }

    private boolean isWithinTolerance() {
        Pose2d currentPose = drivetrain.getState().Pose;
        double positionDistance = currentPose.getTranslation().getDistance(goalState.pose.getTranslation());
        double rotationDistance = Math.abs(currentPose.getRotation().minus(goalState.pose.getRotation()).getRadians());
        return positionDistance < POSITION_TOLERANCE && rotationDistance < ROTATION_TOLERANCE;
    }
}
