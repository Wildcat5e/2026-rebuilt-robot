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

    /** These values will need to be adjusted based on the actual robot's shooting distance capabilities. */
    static boolean withinShootingDistance(Drivetrain drivetrain) {
        double distance = getHubDistance(drivetrain);
        return Constants.MINIMUM_SHOOTING_DISTANCE < distance && distance < Constants.MAXIMUM_SHOOTING_DISTANCE;
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
        return Math.abs(getRobotToHubAngle(drivetrain) - robotRotation) < 4;
    }

    static boolean inHome(Drivetrain drivetrain) {
        double robotXPosition = drivetrain.getState().Pose.getX();

        // If the Robot is on the Blue Alliance, check if it's to the left of the Blue Home Zone threshold.
        // If the Robot is on the Red Alliance, check if it's to the right of the Red Home Zone threshold.
        return Robot.alliance == Alliance.Blue ? robotXPosition < Constants.BLUE_X_AXIS_HOME_THRESHOLD
            : robotXPosition > Constants.RED_X_AXIS_HOME_THRESHOLD;
    }
}
