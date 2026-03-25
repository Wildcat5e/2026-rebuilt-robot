package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.DashboardManager;

import static frc.robot.Utilities.getRobotToTargetAngle;
import static frc.robot.Utilities.getTargetDistance;

/**
 * A helper class to calculate shooting parameters while moving.<br>
 * <br>
 * The math assumes that: <br>
 * 1) Robot position and velocity are Field-Centric (some conversion is necessary for this).<br>
 * 2) Hood angle is fixed.<br>
 * 3) A lookup table exists for static shooting.<br>
 */
public interface ShootingCalculator {
    // Returned by calculate()
    record ShotSolution(double flywheelSpeed, double robotHeading) {}

    /**
     * Calculates the necessary robot heading and shot speed to hit the target while moving.
     *
     * @return ShotSolution containing new heading and speed
     */
    static ShotSolution calculate(Drivetrain drivetrain, Translation2d target,
                                  InterpolatingDoubleTreeMap flywheelSpeedMap) {
        ChassisSpeeds robotVel = drivetrain.getState().Speeds;
        // Convert robot centric speeds to field centric speeds
        robotVel = ChassisSpeeds.fromRobotRelativeSpeeds(robotVel, drivetrain.getState().Pose.getRotation());
        Translation2d robotVector = new Translation2d(robotVel.vxMetersPerSecond, robotVel.vyMetersPerSecond);

        // 1. Look up the Ideal "Static" Shot Speed based on current distance from Hub.
        double staticSpeed = flywheelSpeedMap.get(getTargetDistance(drivetrain, target));

        // 2. Decompose Static Shot into Horizontal Component (3D -> 2D Plane).
        double staticSpeedHorizontal = staticSpeed * Math.cos(Constants.HOOD_ANGLE_RADIANS);

        // 3. Create the Static Vector pointing directly at the hub.
        double        angleToTarget = getRobotToTargetAngle(drivetrain, target);
        Translation2d staticVector  = new Translation2d(staticSpeedHorizontal, new Rotation2d(angleToTarget));

        // 4. Calculate the Shot Vector (Vector Subtraction: V_shot = V_static - V_robot)
        Translation2d shotVector = staticVector.minus(robotVector);

        // 5. Extract outputs and offset shooter angle from robot.
        double targetHeading          = shotVector.getAngle().getRadians() + Constants.SHOOTER_ROTATION_OFFSET;
        double newShotHorizontalSpeed = shotVector.getNorm();

        // 6. Convert back to full 3D flywheel speed (2D Plane -> 3D)
        double newFlywheelSpeed = newShotHorizontalSpeed / Math.cos(Constants.HOOD_ANGLE_RADIANS);

        // 7. Multiply by flywheel speed multiplier from dashboard
        newFlywheelSpeed *= DashboardManager.getFlywheelSpeedMultiplier();

        return new ShotSolution(newFlywheelSpeed, targetHeading);
    }
}
