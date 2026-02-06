package frc.robot.subsystems;

import static frc.robot.Utilities.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * A helper class to calculate shooting parameters while moving.<br>
 * <br>
 * Assumptions: <br>
 * 1) Robot position and velocity are Field-Centric.<br>
 * 2) Hood angle is fixed.<br>
 * 3) A lookup table exists for static shooting.<br>
 */
public interface ShootingCalculator {
    double FIXED_HOOD_ANGLE_RADIANS = 10; // PLACE HOLDER VALUE !!

    // Returned by calculate()
    public static record ShotSolution(double flywheelSpeed, double robotHeading) {}

    /**
     * Calculates the necessary robot heading and shot speed to hit the target while moving.
     * 
     * @param drivetrain
     * @return ShotSolution containing new heading and speed
     */
    static ShotSolution calculate(Drivetrain drivetrain) {
        ChassisSpeeds robotVel = drivetrain.getState().Speeds;
        // Convert robot centric speeds to field centric speeds
        robotVel = ChassisSpeeds.fromRobotRelativeSpeeds(robotVel, drivetrain.getState().Pose.getRotation());
        double robotVelX = robotVel.vxMetersPerSecond;
        double robotVelY = robotVel.vyMetersPerSecond;

        // 1. Calculate Vector to Target (Distance and Angle)
        double distanceToTarget = getHubDistance(drivetrain);
        double angleToTarget = getRobotToHubAngle(drivetrain);

        // 2. Look up the Ideal "Static" Shot Speed This is the speed you would shoot if standing perfectly still at
        // this distance.
        double staticSpeed = getStaticSpeedFromTable(distanceToTarget);

        // 3. Decompose Static Shot into Horizontal Component
        // We only care about the horizontal plane for vector subtraction.
        // horizontal_speed = total_speed * cos(hood_angle)
        double staticSpeedHorizontal = staticSpeed * Math.cos(FIXED_HOOD_ANGLE_RADIANS);

        // 4. Create the Static Vector
        // This vector points directly at the hub with the magnitude calculated above.
        double staticVx = staticSpeedHorizontal * Math.cos(angleToTarget);
        double staticVy = staticSpeedHorizontal * Math.sin(angleToTarget);

        // 5. Calculate the Shot Vector (Vector Subtraction)
        // V_shot = V_static - V_robot
        double shotVx = staticVx - robotVelX;
        double shotVy = staticVy - robotVelY;

        // 6. Extract Outputs

        // Calculate the new Heading (Lead Angle)
        double newHeadingRadians = Math.atan2(-shotVy, -shotVx);

        // Calculate the new Shot Speed
        // First get the horizontal magnitude
        double newShotHorizontalSpeed = Math.hypot(shotVx, shotVy);

        // Then convert back to full flywheel speed (divide by cos(hood))
        // S_total = V_horizontal / cos(theta)
        double newFlywheelSpeed = newShotHorizontalSpeed / Math.cos(FIXED_HOOD_ANGLE_RADIANS);

        return new ShotSolution(newFlywheelSpeed, newHeadingRadians);
    }

    /**
     * Mock Lookup Table
     */
    private static double getStaticSpeedFromTable(double distance) {
        // Probably going to use InterpolatingDoubleTreeMap or similar.

        // Dummy formula: Speed increases with distance
        return 10.0 + (distance * 2.0);
    }
}
