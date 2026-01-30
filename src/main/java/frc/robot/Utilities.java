package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Controller;

/**
 * import static frc.robot.Utilities.*;
 */
public interface Utilities {
    static double getHubDistance() {
        // translation2d is just pose2d without rotation factor
        Translation2d hubTranslation = new Translation2d(4.625, 4.025);
        Translation2d robotTranslation = Controller.drivetrain.getState().Pose.getTranslation();
        double distanceRobotHub = hubTranslation.getDistance(robotTranslation);
        return distanceRobotHub;
    }

    static boolean withinShootingDistance() {
        double distance = getHubDistance();
        return (1 < distance && distance < 3);
    }

    /** Returns in radians. */
    static double getRobotToHubAngle() {
        double hubXPose = 4.625;
        double hubYPose = 4.025;

        Pose2d currentPose = Controller.drivetrain.getState().Pose;
        double angleOfRobotToHub = Math.atan2((hubYPose - currentPose.getY()), (hubXPose - currentPose.getX()));
        return angleOfRobotToHub;
    }

    static boolean withinShootingAngle() {
        double robotRotation = Controller.drivetrain.getState().Pose.getRotation().getRadians();
        // NEED TO TEST DEGREE VALUE
        // debug
        SmartDashboard.putNumber("robot angle", Math.toDegrees(robotRotation));
        SmartDashboard.putNumber("correct angle", Math.toDegrees(getRobotToHubAngle()));
        return Math.abs(getRobotToHubAngle() - robotRotation) < 4;
    }
}
