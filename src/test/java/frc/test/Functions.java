package frc.test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** A collection of helper functions for tests. */
public class Functions {

    /** Helper method to create Pose2d from x, y, and radians. */
    public static Pose2d pose2d(double x, double y, double radians) {
        return new Pose2d(x, y, Rotation2d.fromRadians(radians));
    }

}
