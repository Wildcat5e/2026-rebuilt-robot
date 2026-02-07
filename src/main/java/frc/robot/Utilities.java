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
    static Translation2d getHubTranslation() {
        return Robot.alliance == Alliance.Blue ? Constants.BLUE_HUB_TRANSLATION : Constants.RED_HUB_TRANSLATION;
    }

    static double getHubDistance(Drivetrain drivetrain) {
        Translation2d hubTranslation = getHubTranslation();
        Translation2d robotTranslation = drivetrain.getState().Pose.getTranslation();
        double distanceRobotHub = hubTranslation.getDistance(robotTranslation);
        return distanceRobotHub;
    }

    static boolean withinShootingDistance(Drivetrain drivetrain) {
        double distance = getHubDistance(drivetrain);
        return 1 < distance && distance < 3;
    }

    /** Returns in radians. */
    static double getRobotToHubAngle(Drivetrain drivetrain) {
        Translation2d hubTranslation = getHubTranslation();
        Pose2d currentPose = drivetrain.getState().Pose;
        double angleOfRobotToHub =
            Math.atan2((hubTranslation.getY() - currentPose.getY()), (hubTranslation.getX() - currentPose.getX()));
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
        double BLUE_X_AXIS_BARRIER = 4.0;
        double RED_X_AXIS_BARRIER = 12.54;
        double robotXPose = drivetrain.getState().Pose.getX();

        // ON BLUE SIDE, IF THE X COORDINATE OF ROBOT IS LESS THAN 4.0,
        // THEN THE ROBOT IS IN HOME (BLUE ALLIANCE ZONE)
        if (Robot.alliance == Alliance.Blue) {
            return robotXPose < BLUE_X_AXIS_BARRIER;
        }
        // RED SIDE, IF X COORDINATE OF ROBOT IS GREATER THAN 12.54,
        // THEN THE ROBOT IS IN HOME (RED ALLIANCE ZONE)
        else {
            return robotXPose > RED_X_AXIS_BARRIER;
        }
    }

}
