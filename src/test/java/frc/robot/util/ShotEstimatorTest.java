package frc.robot.util;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.Test;

import java.util.Map;

import static com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import static frc.test.Functions.pose2d;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.mock;

class ShotEstimatorTest {

    // Creates mock object we can use to test different scenarios
    SwerveDriveState swerveDriveState = mock(SwerveDrivetrain.SwerveDriveState.class);

    // Create shot estimator, injecting the swerve drive state supplier, the hub,
    // and a simple hub distance to flywheel speed lookup table
    ShotEstimator estimator = new ShotEstimator(
            () -> swerveDriveState,
            Hub.BLUE,
            InterpolatingDoubleTreeMap.ofEntries(
                    Map.entry(1.0, 2.0),
                    Map.entry(2.0, 4.0),
                    Map.entry(3.0, 8.0)
            ));

    @Test void exploreLookupTable() {
        // Values less than minimum distance should return minimum speed
        assertEquals(2.0, estimator.flywheelSpeedsByHubDistance.get(0.0), .01);

        // Values within the range should interpolate linearly
        assertEquals(2.0, estimator.flywheelSpeedsByHubDistance.get(1.0), .01);
        assertEquals(6.0, estimator.flywheelSpeedsByHubDistance.get(2.5), .01);

        // Exact values in the table should return exact speeds
        assertEquals(3.0, estimator.flywheelSpeedsByHubDistance.get(1.5), .01);
        assertEquals(4.0, estimator.flywheelSpeedsByHubDistance.get(2.0), .01);
        assertEquals(8.0, estimator.flywheelSpeedsByHubDistance.get(3.0), .01);

        // Values greater than maximum distance should return maximum speed
        assertEquals(8.0, estimator.flywheelSpeedsByHubDistance.get(4.0), .01);
    }

    @Test void whenAllValuesZero() {
        swerveDriveState.Speeds = new ChassisSpeeds(0, 0, 0);
        swerveDriveState.Pose = pose2d(0, 0, 0);

        var solution = estimator.solve();

        assertEquals(6.71, solution.flywheelSpeed(), .01);
        assertEquals(6.13, solution.distanceToTarget(), .01);
        assertEquals(-2.42, solution.robotHeadingRadians(), .01);
    }

    @Test void whenAllValuesNonZero() {
        swerveDriveState.Speeds = new ChassisSpeeds(2, 1, 0);
        swerveDriveState.Pose = pose2d(3, 4, Math.PI / 2);

        var solution = estimator.solve();

        assertEquals(2.68, solution.flywheelSpeed(), .01);
        assertEquals(1.62, solution.distanceToTarget(), .01);
        assertEquals(-2.27, solution.robotHeadingRadians(), .01);
    }
}
