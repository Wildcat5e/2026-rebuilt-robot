package frc.robot.subsystems;

import java.util.Map;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.DashboardManager;
import frc.robot.Robot;
import static frc.robot.Utilities.*;

/**
 * A helper class to calculate shooting parameters while moving.<br>
 * <br>
 * Assumptions: <br>
 * 1) Robot position and velocity are Field-Centric.<br>
 * 2) Hood angle is fixed.<br>
 * 3) A lookup table exists for static shooting.<br>
 */
public interface ShootingCalculator {
    /** Lookup table mapping distance from the hub to the ideal static flywheel speed. */
    // @formatter:off
    static final InterpolatingDoubleTreeMap FLYWHEEL_SPEEDS_MAP =
    InterpolatingDoubleTreeMap.ofEntries(
        // Map.entry(Distance in Meters, Flywheel Speed in m/s)
        Map.entry(2.28, 14.35),
        Map.entry(3.09, 14.85),
        Map.entry(4.14, 16.39),
        Map.entry(4.9, 17.55)); // @formatter:on

    // Returned by calculate()
    static record ShotSolution(double flywheelSpeed, double robotHeading) {}

    /**
     * Calculates the necessary robot heading and shot speed to hit the target while moving.
     * 
     * @param drivetrain
     * @param flywheelSpeedMult
     * @return ShotSolution containing new heading and speed
     */
    static ShotSolution calculate(Drivetrain drivetrain, Translation2d target) {
        ChassisSpeeds robotVel = drivetrain.getState().Speeds;
        // Convert robot centric speeds to field centric speeds
        robotVel = ChassisSpeeds.fromRobotRelativeSpeeds(robotVel, drivetrain.getState().Pose.getRotation());
        Translation2d robotVector = new Translation2d(robotVel.vxMetersPerSecond, robotVel.vyMetersPerSecond);

        // 1. Look up the Ideal "Static" Shot Speed based on current distance from Hub.
        double staticSpeed = FLYWHEEL_SPEEDS_MAP.get(getTargetDistance(drivetrain, target));

        // 2. Decompose Static Shot into Horizontal Component (3D -> 2D Plane).
        double staticSpeedHorizontal = staticSpeed * Math.cos(Constants.HOOD_ANGLE_RADIANS);

        // 3. Create the Static Vector pointing directly at the hub.
        double angleToTarget = getRobotToTargetAngle(drivetrain, target);
        Translation2d staticVector = new Translation2d(staticSpeedHorizontal, new Rotation2d(angleToTarget));

        // 4. Calculate the Shot Vector (Vector Subtraction: V_shot = V_static - V_robot)
        Translation2d shotVector = staticVector.minus(robotVector);

        // 5. Extract Outputs
        double targetHeading = shotVector.getAngle().getRadians();
        double newShotHorizontalSpeed = shotVector.getNorm();

        // 6. Convert back to full 3D flywheel speed (2D Plane -> 3D)
        double newFlywheelSpeed = newShotHorizontalSpeed / Math.cos(Constants.HOOD_ANGLE_RADIANS);

        if (!Robot.IS_COMPETITION) {
            // 7. Multiply by flywheel speed multiplier from dashboard
            newFlywheelSpeed *= DashboardManager.getFlywheelSpeedMultiplier();
        }

        return new ShotSolution(newFlywheelSpeed, targetHeading);
    }
}
