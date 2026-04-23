package frc.robot;

import static edu.wpi.first.units.Units.*;
import java.util.List;
import java.util.Map;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.generated.TunerConstants;

public interface Constants {
    /** kSpeedAt12Volts desired top speed in m/s */
    double MAX_LINEAR_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    /** 540 deg/sec max angular velocity in rad/sec */
    double MAX_ANGULAR_SPEED = DegreesPerSecond.of(540).in(RadiansPerSecond);
    /** 720 deg/sec^2 max angular acceleration in rad/sec^2 */
    double MAX_ANGULAR_ACCEL = DegreesPerSecondPerSecond.of(720).in(RadiansPerSecondPerSecond);

    /** The angle of the shooter's hood, as measured from the horizontal at the point where the ball leaves the hood. */
    double HOOD_ANGLE_RADIANS = Math.toRadians(64);

    // These values will need to be adjusted based on the actual robot's shooting distance capabilities
    double MINIMUM_SHOOTING_DISTANCE = 1.0; // meters
    double MAXIMUM_SHOOTING_DISTANCE = 3.0; // meters

    /** Field layout. Change depending on whether field uses AndyMark or Welded specs */
    AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    Translation2d BLUE_HUB_POSITION = new Translation2d(4.625, 4.03);
    Translation2d RED_HUB_POSITION = new Translation2d(11.915, 4.03);

    /** x-coordinate Threshold for Blue Alliance Home Zone */
    double BLUE_X_AXIS_HOME_THRESHOLD = 4.625;
    /** x-coordinate Threshold for Red Alliance Home Zone */
    double RED_X_AXIS_HOME_THRESHOLD = 11.915;

    /** Index 0 is Blue. Upper homes are Blue Left and Red Right. */
    List<Translation2d> UPPER_HOMES = List.of(new Translation2d(2, 6.25), new Translation2d(14.5, 6.25));
    /** Index 0 is Blue. Lower homes are Blue Right and Red Left. */
    List<Translation2d> LOWER_HOMES = List.of(new Translation2d(2, 1.75), new Translation2d(14.5, 1.75));

    /** Lookup table mapping distance from the Hub to the ideal static flywheel RPS. */
    // @formatter:off
    InterpolatingDoubleTreeMap HUB_FLYWHEEL_RPS_MAP =
    InterpolatingDoubleTreeMap.ofEntries(
        // Map.entry(Distance in Meters, Flywheel RPS)
        Map.entry(1.78, 46.5),
        Map.entry(1.98, 47.5),
        Map.entry(2.20, 49.0),
        Map.entry(2.40, 50.0),
        Map.entry(2.60, 51.0),
        Map.entry(2.80, 52.5),
        Map.entry(3.00, 54.0),
        Map.entry(3.19, 56.0),
        Map.entry(3.40, 57.0),
        Map.entry(3.58, 58.0),
        Map.entry(3.80, 59.0),
        Map.entry(4.00, 62.5),
        Map.entry(4.20, 65.0),
        Map.entry(4.50, 66.0),
        Map.entry(4.85, 69.0),
        Map.entry(4.91, 72.4),
        Map.entry(5.02, 73.2),
        Map.entry(5.18, 74.8));
    /** Lookup table mapping distance from the current Alliance Home to the ideal static flywheel RPS. */
    InterpolatingDoubleTreeMap HOME_FLYWHEEL_RPS_MAP =
    InterpolatingDoubleTreeMap.ofEntries(
        // Map.entry(Distance in Meters, Flywheel RPS)
        Map.entry(1.78, 39.5),
        Map.entry(1.98, 40.4),
        Map.entry(2.20, 41.7),
        Map.entry(2.40, 42.5),
        Map.entry(2.60, 43.4),
        Map.entry(2.80, 44.6),
        Map.entry(3.00, 45.9),
        Map.entry(3.19, 47.6),
        Map.entry(3.40, 48.5),
        Map.entry(3.58, 49.3),
        Map.entry(3.80, 50.2),
        Map.entry(4.00, 53.1),
        Map.entry(4.20, 55.2),
        Map.entry(4.50, 56.1),
        Map.entry(4.85, 58.6),
        Map.entry(4.91, 61.5),
        Map.entry(5.02, 62.2),
        Map.entry(5.18, 63.6)
    ); // @formatter:on
}
