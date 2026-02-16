package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;

/**
 * import static frc.robot.Utilities.*;
 */
public interface Utilities {
    /** @return Translation2d of the Hub for the current alliance. */
    static Translation2d getHubPosition() {
        return Robot.alliance == Alliance.Blue ? Constants.BLUE_HUB_POSITION : Constants.RED_HUB_POSITION;
    }

    static double getHubDistance(Drivetrain drivetrain) {
        Translation2d hubPosition = getHubPosition();
        Translation2d robotPosition = drivetrain.getState().Pose.getTranslation();
        double distanceRobotHub = hubPosition.getDistance(robotPosition);
        return distanceRobotHub;
    }

    static boolean withinShootingDistance(Drivetrain drivetrain) {
        double distance = getHubDistance(drivetrain);
        return 1 < distance && distance < 3;
    }

    /** Returns in radians. */
    static double getRobotToHubAngle(Drivetrain drivetrain) {
        Translation2d hubPosition = getHubPosition();
        Pose2d currentPose = drivetrain.getState().Pose;
        double angleOfRobotToHub =
            Math.atan2((hubPosition.getY() - currentPose.getY()), (hubPosition.getX() - currentPose.getX()));
        return angleOfRobotToHub;
    }

    static boolean withinShootingAngle(Drivetrain drivetrain) {
        double robotRotation = drivetrain.getState().Pose.getRotation().getRadians();

        // Debugging
        SmartDashboard.putNumber("robot angle", Math.toDegrees(robotRotation));
        SmartDashboard.putNumber("correct angle", Math.toDegrees(getRobotToHubAngle(drivetrain)));
        return Math.abs(getRobotToHubAngle(drivetrain) - robotRotation) < 4;
    }

    static boolean inHome(Drivetrain drivetrain) {
        double robotXPosition = drivetrain.getState().Pose.getX();

        // ON BLUE SIDE, IF THE X COORDINATE OF ROBOT IS LESS THAN 4.0,
        // THEN THE ROBOT IS IN HOME (BLUE ALLIANCE ZONE)
        if (Robot.alliance == Alliance.Blue) {
            return robotXPosition < Constants.BLUE_X_AXIS_HOME_THRESHOLD;
        }
        // RED SIDE, IF X COORDINATE OF ROBOT IS GREATER THAN 12.54,
        // THEN THE ROBOT IS IN HOME (RED ALLIANCE ZONE)
        else {
            return robotXPosition > Constants.RED_X_AXIS_HOME_THRESHOLD;
        }
    }

}
