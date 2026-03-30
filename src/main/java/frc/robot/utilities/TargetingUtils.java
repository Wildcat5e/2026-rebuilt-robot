package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Drivetrain;

/**
 * import static frc.robot.utilities.TargetingUtils.*;
 */
public interface TargetingUtils {

    /**
     * @param drivetrain
     * @param target The target from which to calculate the robot's distance.
     * 
     * @return The distance in meters from the robot to the specified target.
     */
    static double getTargetDistance(Drivetrain drivetrain, Translation2d target) {
        Translation2d robotPosition = drivetrain.getState().Pose.getTranslation();
        return target.getDistance(robotPosition);
    }

    /**
     * @param drivetrain
     * @param target The target to which to calculate the robot's angle.
     * 
     * @return The angle, in radians, from the robot to the specified target.
     */
    static double getRobotToTargetAngle(Drivetrain drivetrain, Translation2d target) {
        Pose2d currentPose = drivetrain.getState().Pose;
        return Math.atan2((target.getY() - currentPose.getY()), (target.getX() - currentPose.getX()));
    }
}
