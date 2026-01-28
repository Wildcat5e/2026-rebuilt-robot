package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * import static frc.robot.Utilities.*;
 */
public interface Utilities {

    static public double getHubDistance() {
        // translation2d is just pose2d without rotation factor
        Translation2d hubTranslation = new Translation2d(4.625, 4.025);
        Translation2d robotTranslation = RobotContainer.drivetrain.getState().Pose.getTranslation();
        double distanceRobotHub = hubTranslation.getDistance(robotTranslation);
        return distanceRobotHub;
    }

    static public boolean withinShootingDistance() {
        if (1 < getHubDistance() && getHubDistance() < 3) {
            return true;
        } else {
            return false;
        }
    }

    /** Returns in radians. */
    static public double getRobotToHubAngle() {
        double hubXPose = 4.625;
        double hubYPose = 4.025;

        Pose2d currentPose = RobotContainer.drivetrain.getState().Pose;
        double angleOfRobotToHub = Math.atan2((hubYPose - currentPose.getY()), (hubXPose - currentPose.getX()));
        return angleOfRobotToHub;
    }

    static public boolean withinShootingAngle() {
        // in degrees
        double robotRotation = RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
        // NEED TO TEST DEGREE VALUE
        SmartDashboard.putNumber("robot angle", robotRotation);
        SmartDashboard.putNumber("correct angle", Math.toDegrees(getRobotToHubAngle()));
        if (Math.abs(Math.toDegrees(getRobotToHubAngle()) - robotRotation) < 4) {
            return true;
        } else {
            return false;
        }
    }
}
