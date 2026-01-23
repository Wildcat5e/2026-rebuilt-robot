package frc.robot.commands;

import java.util.List;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

// might need to schedule this command !!
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotateToHub extends Command {
    private final Drivetrain drivetrain;
    private static final double POSITION_TOLERANCE = 0.025;
    private static final double ROTATION_TOLERANCE = 0.025; // @formatter:off
    private static final PPHolonomicDriveController HOLONOMIC_DRIVE_CONTROLLER = new PPHolonomicDriveController(
        new PIDConstants(0, 0, 0), // @formatter:on
        new PIDConstants(10, 0, 0));
    /** Max distance to allow autoalign to work from, unknown units */
    private static final double MAX_DISTANCE = 3;
    private static final double TIME_LIMIT_MILLIS = 3000;
    private long startTime;
    private static final PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
    double counter;
    private static final Pose2d CENTER_HUB = new Pose2d(3.0, 4.0, new Rotation2d(0));
    public static final List<Pose2d> TAG_POSE_LIST = List.of(CENTER_HUB);

    public RotateToHub(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        // addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();

        Pose2d currentPose = drivetrain.getState().Pose;

    }

    @Override
    public void execute() {
        counter++;
        Pose2d currentPose = drivetrain.getState().Pose;

        // POSE OF CENTER OF HUB (4.625, 4.025)
        double hubXPose = 4.625;
        double hubYPose = 4.025;
        double robotXPose = currentPose.getX();
        double robotYPose = currentPose.getY();

        // consider divide by 0 error
        double angleOfRobotToHub = (180 / Math.PI) * Math.atan((hubYPose - robotYPose) / (hubXPose - robotXPose));



        goalState.pose = new Pose2d(robotXPose, robotYPose, Rotation2d.fromDegrees(angleOfRobotToHub));
        // if close enough then dont run pid
        ChassisSpeeds outputSpeeds = HOLONOMIC_DRIVE_CONTROLLER.calculateRobotRelativeSpeeds(currentPose, goalState);
        // this uses a private var in drivetrain that was made public to work with this,
        // maybe alt impl needed?
        if (counter % 50 == 0) {
            System.out.println(currentPose);
            System.out.println(angleOfRobotToHub);
        }
        drivetrain.setControl(RobotContainer.drive.withRotationalRate(outputSpeeds.omegaRadiansPerSecond));
    }
}
