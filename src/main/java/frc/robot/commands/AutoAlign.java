package frc.robot.commands;

import java.util.List;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drivetrain;


// might need to schedule this command !!
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign extends Command {
    private final Drivetrain drivetrain;
    private static final double POSITION_TOLERANCE = 0.025;
    private static final double ROTATION_TOLERANCE = 0.025; // @formatter:off
    private static final PPHolonomicDriveController HOLONOMIC_DRIVE_CONTROLLER = new PPHolonomicDriveController(
        new PIDConstants(5, .5, .2), // @formatter:on
        new PIDConstants(5, .5, .2));
    /** Max distance to allow autoalign to work from, unknown units */
    private static final double MAX_DISTANCE = 3;
    private static final double TIME_LIMIT_MILLIS = 3000;
    private long startTime;
    private static final PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();

    private static final Pose2d CENTER_HUB = new Pose2d(3.0, 4.0, new Rotation2d(0));
    public static final List<Pose2d> TAG_POSE_LIST = List.of(CENTER_HUB);


    public AutoAlign(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        Pose2d currentPose = drivetrain.getState().Pose;
        Pose2d nearestTagPose = currentPose.nearest(TAG_POSE_LIST);
        double distance = currentPose.getTranslation().getDistance(nearestTagPose.getTranslation());
        goalState.pose = nearestTagPose;
        if (distance > MAX_DISTANCE) {
            CommandScheduler.getInstance().cancel(this);
            System.out.println("_");
            System.out.println("TOO FAR DUMMY");
            System.out.println("_");
        }
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getState().Pose;
        ChassisSpeeds outputSpeeds = HOLONOMIC_DRIVE_CONTROLLER.calculateRobotRelativeSpeeds(currentPose, goalState);
        // this uses a private var in drivetrain that was made public to work with this, maybe alt impl needed?
        drivetrain.setControl(drivetrain.m_pathApplyRobotSpeeds.withSpeeds(outputSpeeds));
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        if (isOverTimeLimit()) {
            System.out.println("_");
            System.out.println("TIME LIMIT HIT");
            System.out.println("_");
            return true;
        }
        if (isWithinTolerance()) {
            System.out.println("_");
            System.out.println("TOLERANCE HIT");
            System.out.println("_");
            return true;
        }
        return false;

    }

    private boolean isOverTimeLimit() {
        return System.currentTimeMillis() - startTime >= TIME_LIMIT_MILLIS;
    }

    private boolean isWithinTolerance() {
        Pose2d currentPose = drivetrain.getState().Pose;
        double positionDistance = currentPose.getTranslation().getDistance(goalState.pose.getTranslation());
        double rotationDistance = Math.abs(currentPose.getRotation().minus(goalState.pose.getRotation()).getRadians());
        return (positionDistance < POSITION_TOLERANCE && rotationDistance < ROTATION_TOLERANCE);
    }

}
