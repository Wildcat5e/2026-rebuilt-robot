package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

import static edu.wpi.first.wpilibj.DriverStation.*;
import static java.lang.Math.atan2;

/**
 * Represents the hub (target) on the field for each alliance.
 */
public class Hub {
    /** The blue alliance hub position on the field (in meters). */
    public final static Hub BLUE = new Hub(new Translation2d(4.625, 4.03));

    /** The red alliance hub position on the field (in meters). */
    public final static Hub RED = new Hub(new Translation2d(11.915, 4.03));

    /** The hub for the current alliance. Throws an exception if the driver station has not initialized the alliance. */
    public static Hub home() {
        return DriverStation.getAlliance().orElseThrow() == Alliance.Blue ? BLUE : RED;
    }

    /** The position of the hub on the field. */
    final Translation2d location;

    /** Private constructor to create a Hub at a specific position. Only used to make BLUE and RED hubs */
    private Hub(Translation2d pose) {
        this.location = pose;
    }

    /** Calculates the angle from the robot's pose to the hub. */
    public double angleTo(Pose2d robotPose) {
        return atan2(location.getY() - robotPose.getY(), location.getX() - robotPose.getX());
    }

    /** Calculates the distance from the robot's pose to the hub. */
    double distanceTo(Pose2d robotPose) {
        return location.getDistance(robotPose.getTranslation());
    }
}
