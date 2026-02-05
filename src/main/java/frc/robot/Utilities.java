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
    static double getHubDistance(Drivetrain drivetrain) {
        // translation2d is just pose2d without rotation factor
        Translation2d hubTranslation = new Translation2d(4.625, 4.03);
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

        double hubXPose;
        double hubYPose;

        if (Robot.alliance == Alliance.Blue) {
            hubXPose = 4.625;
            hubYPose = 4.03;
        } else {
            hubXPose = 11.915;
            hubYPose = 4.03;
        }


        Pose2d currentPose = drivetrain.getState().Pose;
        double angleOfRobotToHub = Math.atan2((hubYPose - currentPose.getY()), (hubXPose - currentPose.getX()));
        return angleOfRobotToHub;
    }

    static boolean withinShootingAngle(Drivetrain drivetrain) {
        double robotRotation = drivetrain.getState().Pose.getRotation().getRadians();
        // NEED TO TEST DEGREE VALUE
        // debug
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
