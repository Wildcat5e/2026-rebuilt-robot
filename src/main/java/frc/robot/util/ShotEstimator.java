// @formatter:off
package frc.robot.util;

import java.util.Map;
import java.util.function.Supplier;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.Drivetrain;

/**
 * Estimate Shot Solution while moving.
 */
public class ShotEstimator {

    static final double FIXED_HOOD_ANGLE_RADIANS = 10;

    /**
     * A provisional lookup table mapping distance from hub (meters) to flywheel speed (m/s).
     * TODO: Replace with real data.
     */
    static final InterpolatingDoubleTreeMap FLYWHEEL_SPEEDS_BY_HUB_DISTANCE =
            InterpolatingDoubleTreeMap.ofEntries(
                    Map.entry(1.0, 5.0),
                    Map.entry(1.5, 6.5),
                    Map.entry(2.0, 8.0),
                    Map.entry(2.5, 9.5),
                    Map.entry(3.0, 11.0)
            );

    final Supplier<SwerveDriveState> swerveDriveStateSupplier;
    final Hub hub;
    final InterpolatingDoubleTreeMap flywheelSpeedsByHubDistance;

    /**
     * Creates a default ShotEstimator using the drivetrain state, a sampled
     * lookup table, and the hub for the current alliance.
     */
    public ShotEstimator(Drivetrain drivetrain) {
        this(drivetrain::getState, Hub.home(), FLYWHEEL_SPEEDS_BY_HUB_DISTANCE);
    }

    ShotEstimator(
            Supplier<SwerveDriveState> swerveModuleStateSupplier,
            Hub hub,
            InterpolatingDoubleTreeMap distanceFromHubToFlyweelSpeed
    ) {
        this.swerveDriveStateSupplier = swerveModuleStateSupplier;
        this.flywheelSpeedsByHubDistance = distanceFromHubToFlyweelSpeed;
        this.hub = hub;
    }

    public Solution solve() {
        // Retrieve the current robot state ONLY once per solve call
        var robotState = swerveDriveStateSupplier.get();

        // Calculate speed required to hit target from current position
        var distanceToHub = hub.distanceTo(robotState.Pose);
        var flywheelSpeed = flywheelSpeedsByHubDistance.get(distanceToHub);

        // TODO -- I don't understand why we need to consider the hood angle -- please explain
        var staticSpeedHorizontal = flywheelSpeed * Math.cos(FIXED_HOOD_ANGLE_RADIANS);

        // Calculate the static vector components
        var hubAngle = hub.angleTo(robotState.Pose);
        var staticVx = staticSpeedHorizontal * Math.cos(hubAngle);
        var staticVy = staticSpeedHorizontal * Math.sin(hubAngle);

        var fieldSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(robotState.Speeds, robotState.Pose.getRotation());

        var deltaY = staticVy - fieldSpeed.vyMetersPerSecond;
        var deltaX = staticVx - fieldSpeed.vxMetersPerSecond;

        return new Solution(
                Math.hypot(deltaX, deltaY),
                distanceToHub,
                Math.atan2(deltaY, deltaX)
        );
    }

    /**
     * A solution containing the required flywheel speed, distance to target, and robot heading to hit the target.
     */
    public record Solution(
            double flywheelSpeed,
            double distanceToTarget,
            double robotHeadingRadians
    ) {}

}
